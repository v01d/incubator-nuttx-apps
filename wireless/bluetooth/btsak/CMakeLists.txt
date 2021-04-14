if (CONFIG_BTSAK)
  nuttx_add_application(
    NAME ${CONFIG_BTSAK_PROGNAME}
    PRIORITY ${CONFIG_BTSAK_PRIORITY}
    STACKSIZE ${CONFIG_BTSAK_STACKSIZE}
    SRCS btsak_main.c btsak_advertise.c btsak_features.c
    btsak_gatt.c btsak_info.c btsak_scan.c btsak_security.c)
endif()
