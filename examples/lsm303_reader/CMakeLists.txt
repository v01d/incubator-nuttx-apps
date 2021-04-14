if (CONFIG_EXAMPLES_LSM303_READER)
  nuttx_add_application(
    NAME lsm303_reader
    SRCS lsm303_reader_main.c
  )
endif()
