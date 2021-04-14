if (CONFIG_TESTING_SMART)
  nuttx_add_application(
    NAME smart
    SRCS smart_main.c)
endif()
