#!/usr/bin/env python3
"""
STM32 USB CDC RC forward 监控脚本（24 字节定长帧）

功能：
  - 固定监听 /dev/ttyACM1（默认）
  - 帧同步 / 校验 / 解码
  - 检测乱码、坏包、丢帧、抖动、速率跳变、长时间无帧
  - 连续输出 1s 窗口统计

拨杆值约定:
  - 左/右拨杆合法值: 1(上), 2(中), 3(下)（来自 DJI 协议）。
  - 兼容模式: --allow-zero-switch 允许(sw_left=0, sw_right=0) 不计作坏包（旧上位机兼容）。
"""

from __future__ import annotations

import argparse
import struct
import time
from collections import deque
from typing import Optional, Tuple

import serial

FRAME_LEN = 24
MAGIC = 0xA55A
VERSION = 2
FMT = "<HBBHIhhhhBBhH"
SYNC = struct.pack("<H", MAGIC)
VALID_SWITCH_VALUES = (1, 2, 3)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def parse_frame(raw: bytes) -> Tuple[Optional[Tuple], Optional[str]]:
    if len(raw) != FRAME_LEN:
        return None, "len"
    fields = struct.unpack(FMT, raw)
    magic, version, payload_len, seq, tick_ms, lx, ly, rx, ry, sw_left, sw_right, wheel, crc16 = fields
    if magic != MAGIC:
        return None, "magic"
    if payload_len != FRAME_LEN:
        return None, "payload_len"
    if version != VERSION:
        return None, "version"
    if crc16_modbus(raw[:-2]) != crc16:
        return None, "crc"
    return fields, None


def _fmt(v: int) -> str:
    return f"{v:>6d}"


def _is_valid_switch(sw_left: int, sw_right: int, *, allow_zero_switch: bool) -> bool:
    if allow_zero_switch and sw_left == 0 and sw_right == 0:
        return True
    return (sw_left in VALID_SWITCH_VALUES) and (sw_right in VALID_SWITCH_VALUES)


def read_frames(
    port: str,
    baud: int,
    duration_sec: Optional[float],
    summary_interval: float,
    jitter_alert_ms: float,
    rate_drop_ratio: float,
    bad_window_ratio: float,
    no_data_timeout: float,
    max_garbage_window: int,
    verbose: bool,
    raw: bool,
    show_seq: bool,
    allow_zero_switch: bool
) -> None:
    with serial.Serial(port, baudrate=baud, timeout=0.05) as ser:
        print(f"[open] {port} {baud} 8N1")

        buf = bytearray()
        stats = {
            "total": 0,
            "good": 0,
            "bad": 0,
            "bad_magic": 0,
            "bad_len": 0,
            "bad_payload_len": 0,
            "bad_ver": 0,
            "bad_crc": 0,
            "bad_switch": 0,
            "dup_seq": 0,
            "lost": 0,
            "resync_bytes": 0,
            "anomaly": 0,
            "bytes": 0,
            "bad_magic_runs": 0,
            "last_seq": None,
            "last_valid_ts": None,
            "last_raw_ts": time.time(),
            "last_raw_warn_ts": 0.0,
            "last_valid_warn_ts": 0.0,
            "start_ts": time.time(),
            "last_report_ts": time.time(),
            "window_good": 0,
            "window_total": 0,
            "window_bad": 0,
            "last_window_good": 0,
            "last_window_total": 0,
            "last_window_ts": time.time(),
            "jitter": deque(maxlen=100),
            "jitter_max_seen": 0.0,
            "baseline_rate": None,
            "rate_drop_count": 0,
            "garbage_streak": 0,
        }

        end_ts = stats["start_ts"] + duration_sec if duration_sec is not None else None

        while True:
            now = time.time()
            if end_ts is not None and now >= end_ts:
                break

            chunk = ser.read(4096)
            if chunk:
                buf.extend(chunk)
                stats["bytes"] += len(chunk)
                stats["garbage_streak"] = 0
                stats["last_raw_ts"] = time.time()
            else:
                # 无原始字节和无有效帧分开告警，便于区分线缆断开/编码异常
                if (now - stats["last_raw_ts"]) > no_data_timeout and (
                    (now - stats["last_raw_warn_ts"]) > no_data_timeout
                ):
                    stats["anomaly"] += 1
                    stats["last_raw_warn_ts"] = now
                    print(f"[WARN] {no_data_timeout:.1f}s 内无任何原始字节")
                if stats["last_valid_ts"] is not None and (now - stats["last_valid_ts"]) > no_data_timeout and (
                    (now - stats["last_valid_warn_ts"]) > no_data_timeout
                ):
                    stats["anomaly"] += 1
                    stats["last_valid_warn_ts"] = now
                    print(f"[WARN] {no_data_timeout:.1f}s 内无有效帧")
                continue

            while len(buf) >= FRAME_LEN:
                search_range = max(0, len(buf) - FRAME_LEN + 1)
                idx = buf.find(SYNC, 0, search_range + 1)
                if idx < 0:
                    if len(buf) > FRAME_LEN - 1:
                        drop = len(buf) - (FRAME_LEN - 1)
                        stats["resync_bytes"] += drop
                        if drop > max_garbage_window:
                            stats["bad_magic"] += 1
                            if drop >= 2 * max_garbage_window:
                                stats["bad_magic_runs"] += 1
                        if stats["garbage_streak"] == 0:
                            stats["garbage_streak"] = 1
                        else:
                            stats["garbage_streak"] += 1
                        if verbose:
                            print(f"[sync] 丢弃 {drop} 字节后继续同步")
                        if len(buf) > FRAME_LEN - 1:
                            del buf[:len(buf) - (FRAME_LEN - 1)]
                    break

                if idx > 0:
                    stats["resync_bytes"] += idx
                    if idx > max_garbage_window:
                        stats["bad_magic"] += 1
                        stats["bad_magic_runs"] += 1
                        if verbose:
                            print(f"[sync] 偏移 {idx} 字节，可能存在乱码/串口抖动")
                    del buf[:idx]
                    stats["garbage_streak"] = 0

                    if len(buf) < FRAME_LEN:
                        break

                raw_frame = bytes(buf[:FRAME_LEN])
                del buf[:FRAME_LEN]
                stats["total"] += 1
                stats["window_total"] += 1

                frame, reason = parse_frame(raw_frame)
                if frame is None:
                    stats["bad"] += 1
                    stats["window_bad"] += 1
                    if reason == "len":
                        stats["bad_len"] += 1
                    elif reason == "magic":
                        stats["bad_magic"] += 1
                    elif reason == "payload_len":
                        stats["bad_payload_len"] += 1
                    elif reason == "version":
                        stats["bad_ver"] += 1
                    elif reason == "crc":
                        stats["bad_crc"] += 1
                    continue

                _, _, _, seq, tick_ms, lx, ly, rx, ry, sw_left, sw_right, wheel, crc = frame

                # 序号与丢帧检测
                last_seq = stats["last_seq"]
                if last_seq is None:
                    gap = 1
                else:
                    gap = (seq - last_seq) & 0xFFFF

                if gap == 0:
                    stats["dup_seq"] += 1
                elif gap > 1:
                    stats["lost"] += (gap - 1)
                    if gap > 3:
                        stats["anomaly"] += 1
                stats["last_seq"] = seq

                if not _is_valid_switch(sw_left, sw_right, allow_zero_switch=allow_zero_switch):
                    stats["bad_switch"] += 1

                # 计算帧间隔抖动（ms）
                now = time.time()
                last_valid_ts = stats["last_valid_ts"]
                if last_valid_ts is not None:
                    jitter_ms = (now - last_valid_ts) * 1000.0
                    stats["jitter"].append(jitter_ms)
                    if jitter_ms > stats["jitter_max_seen"]:
                        stats["jitter_max_seen"] = jitter_ms
                stats["last_valid_ts"] = now

                stats["good"] += 1
                stats["window_good"] += 1

                if show_seq or verbose:
                    tag = ""
                    if gap > 1:
                        tag = f" GAP+{gap - 1}"
                    if not _is_valid_switch(sw_left, sw_right, allow_zero_switch=allow_zero_switch):
                        tag += " SW_INVALID"
                    print(
                        f"seq={seq:5d} t={tick_ms:10d} "
                        f"lx={_fmt(lx)} ly={_fmt(ly)} rx={_fmt(rx)} ry={_fmt(ry)} "
                        f"wheel={_fmt(wheel)} swL={sw_left} swR={sw_right} "
                        f"crc=0x{crc:04x}{tag}"
                    )

                if raw:
                    print(raw_frame.hex(" "))

            # 周期统计
            now = time.time()
            elapsed_since_last = now - stats["last_report_ts"]
            if elapsed_since_last >= summary_interval:
                elapsed_total = now - stats["start_ts"]
                total_rate = stats["good"] / max(1e-9, elapsed_total)
                window_rate = (stats["window_good"] - stats["last_window_good"]) / max(
                    1e-9, now - stats["last_window_ts"]
                )
                window_total = stats["window_total"] - stats["last_window_total"]

                if stats["jitter"]:
                    jmin = min(stats["jitter"])
                    jmax = max(stats["jitter"])
                    javg = sum(stats["jitter"]) / len(stats["jitter"])
                else:
                    jmin = jmax = javg = 0.0

                if (
                    window_total >= 10
                    and stats["window_bad"] > 0
                    and (stats["window_bad"] / window_total) > bad_window_ratio
                ):
                    stats["anomaly"] += 1
                    print(
                        f"[WARN] 窗口坏包比例异常: {stats['window_bad']}/{window_total} "
                        f"({stats['window_bad']/window_total:.1%}) > {bad_window_ratio:.0%}"
                    )

                if stats["baseline_rate"] is None and stats["good"] >= 30:
                    stats["baseline_rate"] = window_rate
                if (
                    stats["baseline_rate"] is not None
                    and window_rate < stats["baseline_rate"] * (1.0 - rate_drop_ratio)
                    and window_rate > 1.0
                ):
                    stats["anomaly"] += 1
                    stats["rate_drop_count"] += 1
                    print(
                        f"[WARN] 速率下滑: window={window_rate:.1f}/s, "
                        f"baseline={stats['baseline_rate']:.1f}/s, lost={stats['lost']}"
                    )
                    if stats["baseline_rate"] is not None:
                        # 降低敏感度，采用 EMA 更新
                        stats["baseline_rate"] = stats["baseline_rate"] * 0.9 + window_rate * 0.1

                if stats["jitter"] and stats["jitter"][-1] > jitter_alert_ms:
                    stats["anomaly"] += 1
                    print(f"[WARN] 抖动异常: 近期max={jmax:.1f}ms, 阈值={jitter_alert_ms:.1f}ms")

                print(
                    f"[STAT] total={stats['total']} good={stats['good']} "
                    f"bad={stats['bad']} bad(len={stats['bad_len']} plen={stats['bad_payload_len']} "
                    f"ver={stats['bad_ver']} crc={stats['bad_crc']} magic={stats['bad_magic']} "
                    f"bad_magic_runs={stats['bad_magic_runs']} "
                    f"switch={stats['bad_switch']} dup={stats['dup_seq']}) "
                    f"lost={stats['lost']} resync={stats['resync_bytes']} bytes={stats['bytes']} "
                    f"rate={total_rate:.1f}/s window={window_rate:.1f}/s "
                    f"jitter={jmin:.1f}/{javg:.1f}/{jmax:.1f}ms "
                    f"anomaly={stats['anomaly']} rate_drop={stats['rate_drop_count']}"
                )

                stats["last_report_ts"] = now
                stats["last_window_good"] = stats["window_good"]
                stats["last_window_total"] = stats["window_total"]
                stats["window_bad"] = 0
                stats["last_window_ts"] = now

        if duration_sec is not None:
            print(f"[done] 监听时长={duration_sec:.1f}s，异常总数={stats['anomaly']}")


def self_test() -> None:
    assert struct.calcsize(FMT) == FRAME_LEN

    for seq, wheel in enumerate((-660, 0, 660), start=1):
        frame_no_crc = struct.pack(
            FMT,
            MAGIC,
            VERSION,
            FRAME_LEN,
            seq,
            1234,
            -100,
            200,
            -300,
            0,
            2,
            3,
            wheel,
            0,
        )
        frame_ok = bytearray(frame_no_crc[:-2])
        frame_ok.extend(struct.pack("<H", crc16_modbus(frame_ok)))
        parsed, reason = parse_frame(bytes(frame_ok))
        assert reason is None
        assert parsed is not None
        assert parsed[11] == wheel

        frame_bad = bytearray(frame_ok)
        frame_bad[-1] ^= 0xFF
        assert parse_frame(bytes(frame_bad))[0] is None

    print("[self-test] 拨轮负值/零值/正值及坏帧解析检查通过")


def main() -> None:
    p = argparse.ArgumentParser(description="监听 STM32 CDC RC forward 帧（默认监听 /dev/ttyACM1）")
    p.add_argument("port", nargs="?", default="/dev/ttyACM1", help="串口设备，默认 /dev/ttyACM1")
    p.add_argument("-b", "--baud", type=int, default=115200)
    p.add_argument("-t", "--duration", type=float, default=None, help="监听时长（秒），不填则持续监听")
    p.add_argument("--summary-interval", type=float, default=1.0, help="统计输出间隔（秒）")
    p.add_argument("--jitter-alert", type=float, default=20.0, help="抖动告警阈值（ms）")
    p.add_argument("--rate-drop-ratio", type=float, default=0.35, help="速率下滑比例告警（例如 0.35）")
    p.add_argument("--bad-window-ratio", type=float, default=0.25, help="1s窗口坏包比例告警（例如 0.25）")
    p.add_argument("--no-data-timeout", type=float, default=2.0, help="无有效帧超时告警（秒）")
    p.add_argument("--max-garbage-window", type=int, default=12, help="同步偏移丢弃超过该字节视为异常")
    p.add_argument("-v", "--verbose", action="store_true", help="打印每帧解码信息")
    p.add_argument("--raw", action="store_true", help="打印原始16进制")
    p.add_argument("--show-seq", action="store_true", help="打印每帧日志（便于回放对齐）")
    p.add_argument("--allow-zero-switch", action="store_true", help="兼容 sw_left=0, sw_right=0 的旧版上位机取值")
    p.add_argument("--self-test", action="store_true", help="运行自检后退出")
    args = p.parse_args()

    if args.self_test:
        self_test()
        return

    read_frames(
        port=args.port,
        baud=args.baud,
        duration_sec=args.duration,
        summary_interval=args.summary_interval,
        jitter_alert_ms=args.jitter_alert,
        rate_drop_ratio=args.rate_drop_ratio,
        bad_window_ratio=args.bad_window_ratio,
        no_data_timeout=args.no_data_timeout,
        max_garbage_window=args.max_garbage_window,
        verbose=args.verbose,
        raw=args.raw,
        show_seq=args.show_seq or args.verbose,
        allow_zero_switch=args.allow_zero_switch,
    )


if __name__ == "__main__":
    main()
