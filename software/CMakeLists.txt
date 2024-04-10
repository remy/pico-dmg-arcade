cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(picodvi C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()


add_compile_options(
      -Wall
      -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
      -Wno-unused-function # we have some for the docs that aren't called
)

# To change the default serial config for all apps, pass e.g.
#   cmake -DDVI_DEFAULT_SERIAL_CONFIG=pimoroni_demo_hdmi_cfg ..
# then rebuild. This is applied via target_compile_definitions on each app
# individually.
set(DVI_DEFAULT_SERIAL_CONFIG "pico_sock_cfg" CACHE STRING
	"Select a default pin configuration from common_dvi_pin_configs.h")

include_directories(
	assets
	include
	)

add_subdirectory(libdvi)
add_subdirectory(libsprite)

add_subdirectory(apps)
