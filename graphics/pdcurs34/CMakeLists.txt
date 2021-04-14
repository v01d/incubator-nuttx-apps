if (CONFIG_GRAPHICS_PDCURSES)
  nuttx_add_user_library(pdcurs34)

  add_subdirectory(nuttx)
  add_subdirectory(pdcurses)

  target_include_directories(pdcurs34 PRIVATE include)
  target_sources(pdcurs34 PRIVATE SRCS)
endif()
