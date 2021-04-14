if (CONFIG_SYSTEM_CFGDATA)
  nuttx_add_application(
    NAME cfgdata
    PRIORITY ${CONFIG_SYSTEM_CFGDATA_PRIORITY}
    STACKSIZE ${CONFIG_SYSTEM_CFGDATA_STACKSIZE}
    SRCS cfgdata_main.c)
endif()
