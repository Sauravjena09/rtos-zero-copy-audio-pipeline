cmake_minimum_required(VERSION 3.20.0)
find_package(ZEPHYR REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(audio_pipeline LANGUAGES C ASM)

target_sources(app PRIVATE
    src/main.c
    src/audio_pipeline.c
    src/dsp_filters.c
    )