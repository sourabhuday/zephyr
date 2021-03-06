
#nios2 core/system level headers
zephyr_include_directories(
  include
  altera_hal/HAL/inc
  )

zephyr_compile_definitions(ZEPHYR_RTOS)
add_subdirectory(altera_hal/HAL/src)

if(CONFIG_ALTERA_AVALON_TIMER)
  zephyr_include_directories(
    drivers/altera_avalon_timer/inc
    drivers/altera_avalon_timer/HAL/inc
    )
  add_subdirectory(drivers/altera_avalon_timer/HAL/src)
endif()

if(CONFIG_ALTERA_AVALON_SYSID)
  zephyr_include_directories(
    drivers/altera_avalon_sysid/inc
    drivers/altera_avalon_sysid/HAL/inc
    )
  add_subdirectory(drivers/altera_avalon_sysid/HAL/src)
endif()

if(CONFIG_ALTERA_AVALON_UART)
  zephyr_include_directories(
    drivers/altera_avalon_uart/inc
    drivers/altera_avalon_uart/HAL/inc
    drivers/altera_avalon_uart/LWHAL/inc
    )
  add_subdirectory(
    drivers/altera_avalon_uart/HAL/src
    drivers/altera_avalon_uart/LWHAL/src
    )
endif()

if(CONFIG_UART_ALTERA_JTAG)
  zephyr_compile_definitions(ALTERA_AVALON_JTAG_UART_SMALL)
  zephyr_include_directories(
    drivers/altera_avalon_jtag_uart/inc
    drivers/altera_avalon_jtag_uart/HAL/inc
    drivers/altera_avalon_jtag_uart/LWHAL/inc
    )
  add_subdirectory(
    drivers/altera_avalon_jtag_uart/HAL/src
    drivers/altera_avalon_jtag_uart/LWHAL/src
    )
endif()

if(CONFIG_ALTERA_AVALON_SGDMA)
  zephyr_include_directories(
    drivers/altera_avalon_sgdma/inc
    drivers/altera_avalon_sgdma/HAL/inc
    )
  add_subdirectory(drivers/altera_avalon_sgdma/HAL/src)
endif()

if(CONFIG_ALTERA_AVALON_I2C)
  zephyr_include_directories(
    drivers/altera_avalon_i2c/inc
    drivers/altera_avalon_i2c/HAL/inc
    )
  add_subdirectory(drivers/altera_avalon_i2c/HAL/src)
endif()

if(CONFIG_ALTERA_AVALON_SPI)
  zephyr_include_directories(
    drivers/altera_avalon_spi/inc
    drivers/altera_avalon_spi/HAL/inc
    )
  add_subdirectory(drivers/altera_avalon_spi/HAL/src)
endif()

if(CONFIG_ALTERA_AVALON_QSPI)
  zephyr_include_directories(
    drivers/altera_epcq_controller/inc
    drivers/altera_epcq_controller/HAL/inc
    )
  add_subdirectory(drivers/altera_epcq_controller/HAL/src)
endif()

if(CONFIG_ALTERA_AVALON_PIO)
  zephyr_include_directories(drivers/altera_avalon_pio/inc)
endif()

