cmake_minimum_required(VERSION 3.30)
include(cmake/ArmToolchain.cmake)

project(stm32-elevator C)

set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
enable_language(ASM)

set(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
include(cmake/Device.cmake)
include(cmake/SourceList.cmake)
include(cmake/IncludeList.cmake)

set(LINKER_SCRIPTS_BASE_DIR ${PROJECT_PATH}/STM32-base/linker/)
set(STARTUP_BASE_DIR        ${PROJECT_PATH}/STM32-base/startup/)

# ─────────────────────────────────────────
#  Helper function: build one target
# ─────────────────────────────────────────
function(add_elevator_target TARGET_NAME BOARD_FLAG)

    add_executable(${TARGET_NAME}
        ${SOURCE_LIST}
        ${STARTUP_BASE_DIR}/${SERIES_FOLDER}/${MAPPED_DEVICE}.s
    )

    target_include_directories(${TARGET_NAME} PUBLIC ${INCLUDE_LIST})

    target_compile_definitions(${TARGET_NAME} PUBLIC
        ${COMPILER_DEFINE}
        IS_MASTER_BOARD=${BOARD_FLAG}   # ← هنا بنحدد Master أو Slave
    )

    target_compile_options(${TARGET_NAME} PUBLIC
        ${CPU_PARAMETERS}
        -Wall
        -Wpedantic
        -Wno-unused-parameter
    )

    target_link_options(${TARGET_NAME} PRIVATE
        ${CPU_PARAMETERS}
        -L${LINKER_SCRIPTS_BASE_DIR}
        -T${LINKER_SCRIPTS_BASE_DIR}/${SERIES_FOLDER}/${DEVICE}.ld
        -Xlinker --gc-sections
        -Wl,-Map=${TARGET_NAME}.map
        --specs=nano.specs
    )

    # Print size
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
        COMMENT "Size of ${TARGET_NAME}:"
    )

    # Generate .hex
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${TARGET_NAME}>
            ${TARGET_NAME}.hex
        COMMENT "Generating ${TARGET_NAME}.hex"
    )

    # Generate .bin
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${TARGET_NAME}>
            ${TARGET_NAME}.bin
        COMMENT "Generating ${TARGET_NAME}.bin"
    )

endfunction()

# ─────────────────────────────────────────
#  Build الاتنين
# ─────────────────────────────────────────
add_elevator_target(elevator_master  1)   # IS_MASTER_BOARD=1
add_elevator_target(elevator_slave   0)   # IS_MASTER_BOARD=0