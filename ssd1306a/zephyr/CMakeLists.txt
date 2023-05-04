if(CONFIG_SSD1306A)

  # Add ssd1306a.h to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    ssd1306a.c
    )

endif()