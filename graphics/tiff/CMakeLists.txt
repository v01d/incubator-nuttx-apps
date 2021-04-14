if (CONFIG_TIFF)
  nuttx_add_user_library(tiff)
  target_sources(tiff PRIVATE
    tiff_addstrip.c tiff_finalize.c tiff_initialize.c tiff_utils.c)
endif()
