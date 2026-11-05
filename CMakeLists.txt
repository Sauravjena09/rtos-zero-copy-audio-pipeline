cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(audio_pipeline LANGUAGES C ASM)

target_include_directories(app PRIVATE src)

target_compile_options(app PRIVATE
    -Wextra
    -Wshadow
    -Wconversion
    -Wundef
    -Wstrict-prototypes
)

target_sources(app PRIVATE
    src/main.c
    src/audio_pipeline.c
    src/dsp_filters.c
)

add_subdirectory(filter)
