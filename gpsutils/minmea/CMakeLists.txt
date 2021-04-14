if (CONFIG_GPSUTILS_MINMEA_LIB)
  nuttx_add_user_library(minmea minmea.c)
  target_compile_options(minmea PRIVATE -std=c99)
endif()
