if (CONFIG_GRAPHICS_SLCD)
  nuttx_add_user_library(slcd)
  target_sources(slcd PRIVATE cslcd.cxx slcd_mapping.cxx slcd_trapezoids.cxx)
endif()
