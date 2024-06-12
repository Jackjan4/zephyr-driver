if(CONFIG_SL025B)

  # Add sl025b.h to the set of global include paths.
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    sl025b.c
    )

endif()