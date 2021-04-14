if(CONFIG_CANUTILS_LIBCANUTILS)
  nuttx_add_user_library(libcanutils lib.c)
  target_include_directories(libcanutils INTERFACE ${CMAKE_CURRENT_LIST_DIR})
endif()
