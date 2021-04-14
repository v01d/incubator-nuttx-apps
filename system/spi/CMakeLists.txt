if (CONFIG_SYSTEM_SPITOOL)
  nuttx_add_application(
    NAME ${CONFIG_SPITOOL_PROGNAME}
    PRIORITY ${CONFIG_SPITOOL_PRIORITY}
    STACKSIZE ${CONFIG_SPITOOL_STACKSIZE}
    SRCS spi_main.c spi_bus.c spi_devif.c spi_exch.c spi_common.c)
endif()
