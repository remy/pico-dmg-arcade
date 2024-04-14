# First target: executable that uses regular software encode loop

set(PROJECT gameboy)

include(${PICO_SDK_PATH}/lib/tinyusb/hw/bsp/family_support.cmake)

add_executable(${PROJECT}
	main.c
)

target_sources(${PROJECT} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/usb_gamepad.c
  ${CMAKE_CURRENT_SOURCE_DIR}/main.c
)

family_initialize_project(${PROJECT} ${CMAKE_CURRENT_LIST_DIR})

# pico_enable_stdio_usb(gameboy 1)
pico_enable_stdio_uart(gameboy 1)

pico_generate_pio_header(${PROJECT} ${CMAKE_CURRENT_LIST_DIR}/gb.pio)

# target_compile_options(${PROJECT} PRIVATE -Wall)

target_include_directories(${PROJECT} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        )

target_compile_definitions(${PROJECT} PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
  DVI_VERTICAL_REPEAT=4
)

target_link_libraries(${PROJECT}
	pico_stdlib
  pico_multicore
	libdvi
  libsprite
  tinyusb_host
  tinyusb_board
)

pico_add_extra_outputs(${PROJECT})
