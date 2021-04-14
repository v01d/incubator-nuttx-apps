if (CONFIG_TESTING_NXFFS)
  nuttx_add_application(
    NAME nxffs
    SRCS nxffs_main.c)
endif()
