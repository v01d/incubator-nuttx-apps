if (CONFIG_INDUSTRY_ABNT_CODI_LIB)
  nuttx_add_user_library(abnt_codi abnt_codi.c)
  target_compile_options(abnt_codi PRIVATE -std=c99)
endif()
