# First target: executable that uses regular software encode loop

add_executable(gameboy
	main.c
)

pico_enable_stdio_usb(gameboy 1)
# pico_enable_stdio_uart(gameboy 0)

pico_generate_pio_header(gameboy ${CMAKE_CURRENT_LIST_DIR}/gb.pio)

# target_compile_options(gameboy PRIVATE -Wall)

target_compile_definitions(gameboy PRIVATE
	DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
  # DVI_VERTICAL_REPEAT=4
)

target_link_libraries(gameboy
	pico_stdlib
  pico_multicore
	libdvi
  libsprite
)

pico_add_extra_outputs(gameboy)

