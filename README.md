# STM32F4 template
This is a cmake project template for using the STM32-base project with a STM32F4 series device based on
[STM32-base project](https://github.com/STM32-base/STM32-base).

## Build Instructions

### 1. Prerequisites
- ARM GNU toolchain (`arm-none-eabi-*`) installed.
- CMake (3.30+).
- Ninja (recommended generator), or use CLion.

### 2. Configure toolchain path
Edit `cmake/ArmToolchain.cmake` and set:
```cmake
set(ARM_DIR  "put the ARM toolchain path here(folder before bin)")
```

### 3. (Optional) Select target device
Edit `cmake/Device.cmake` if needed:
```cmake
set(DEVICE STM32F401xE)
```

### 4. Configure and build
From project root:
```powershell
cmake -S . -B build -G Ninja
cmake --build build
```

### 5. Build outputs
After a successful build, outputs are in `build/`:
- `stm32-template.elf`
- `stm32-template.hex`
- `stm32-template.bin`
- `stm32-template.map`
