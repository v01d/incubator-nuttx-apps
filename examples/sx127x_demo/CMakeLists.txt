if (CONFIG_EXAMPLES_SX127X)
  nuttx_add_application(
    SRCS sx127x_demo.c
    NAME sx127x
  )
endif()
