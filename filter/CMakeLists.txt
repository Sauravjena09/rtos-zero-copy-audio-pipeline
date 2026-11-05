# Filter module CMakeLists.txt
target_sources(app PRIVATE
    pdm_decimator.c
)

target_include_directories(app PRIVATE
    .
)
