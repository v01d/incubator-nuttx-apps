if (CONFIG_TESTING_SMART_TEST)
  nuttx_add_application(
    NAME smart_test
    STACKSIZE 4096
    SRCS smart_test.c)
endif()
