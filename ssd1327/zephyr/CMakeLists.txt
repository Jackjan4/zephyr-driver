if(CONFIG_SSD1327)

  # Add ssd1327.h to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    ssd1327.c
    )

endif()