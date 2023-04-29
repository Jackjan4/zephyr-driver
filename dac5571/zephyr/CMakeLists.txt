if(CONFIG_DAC5571)

  # Add dac5571.h to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    dac5571.c
    )

endif()