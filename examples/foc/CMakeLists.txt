if (CONFIG_EXAMPLES_FOC)
  set(SRCS)

  # fixed16 support

  if (CONFIG_INDUSTRY_FOC_FIXED16)
    list(APPEND SRCS foc_fixed16_thr.c)
  endif()

  # float32 support

  if(CONFIG_INDUSTRY_FOC_FLOAT)
    list(APPEND SRCS foc_float_thr.c)
  endif()

  nuttx_add_application(
    NAME foc
    SRCS foc_main.c foc_device.c foc_mq.c ${SRCS}
  )
endif()
