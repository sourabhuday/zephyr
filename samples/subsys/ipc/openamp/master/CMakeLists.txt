set(BOARD lpcxpresso54114)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(NONE)

enable_language(C ASM)

# Location of external dependencies:
set(ZEPHYR_KERNEL_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/zephyr/kernel/libkernel.a")
set(PLATFORM_DIR          "${CMAKE_CURRENT_SOURCE_DIR}/../platform")

target_include_directories(app PRIVATE $ENV{ZEPHYR_BASE}/drivers)
target_include_directories(app PRIVATE $ENV{ZEPHYR_BASE}/samples/subsys/ipc/openamp/remote/build/zephyr)

target_sources(app PRIVATE src/main_master.c)
if(CONFIG_SOC_SERIES_LPC54XXX)
	target_sources(app PRIVATE $ENV{ZEPHYR_BASE}/ext/hal/nxp/mcux/devices/LPC54114/incbin.S)
	target_compile_options(app PRIVATE -Wa,-I$ENV{ZEPHYR_BASE}/samples/subsys/ipc/openamp/remote/build/zephyr)
endif()

include($ENV{ZEPHYR_BASE}/cmake/open-amp.cmake)

add_dependencies(app open-amp)

target_include_directories(app PRIVATE ${LIBMETAL_INCLUDE_DIR} ${OPENAMP_INCLUDE_DIR} ${PLATFORM_DIR})

target_link_libraries(app
	${OPENAMP_LIBRARY}
	${LIBMETAL_LIBRARY}
	${ZEPHYR_KERNEL_LIBRARY})

