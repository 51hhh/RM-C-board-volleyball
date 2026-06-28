## 构建
cd cmake-build-debug
cmake --build . -j4

## 烧录
probe-rs download cmake-build-debug/LED.elf --chip STM32F407IGHx


## 复位
probe-rs reset --chip STM32F407IGHx --protocol swd