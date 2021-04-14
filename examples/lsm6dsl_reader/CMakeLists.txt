if (CONFIG_EXAMPLES_LSM6DSL_READER)
  nuttx_add_application(
    NAME lsm6dsl_reader
    SRCS lsm6dsl_reader_main.c
  )
endif()
