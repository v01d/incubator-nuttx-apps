if (CONFIG_IEEE802154_I8SHARK)
  nuttx_add_application(
    NAME i8shark
    SRCS i8shark_main.c)
endif()
