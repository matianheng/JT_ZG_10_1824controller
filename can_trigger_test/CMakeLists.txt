cmake_minimum_required(VERSION 3.20)
project(can_trigger_test C ASM)
add_executable(firmware src/startup.c src/main_can.c)
target_link_options(firmware PRIVATE -T ${CMAKE_SOURCE_DIR}/STM32F407VE_APP.ld -Wl,-Map=firmware.map)
add_custom_command(TARGET firmware POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:firmware> dl1824.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:firmware>
  COMMENT "生成 dl1824.bin（基址 0x08010000）")
