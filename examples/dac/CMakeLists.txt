if(CONFIG_EXAMPLES_DAC)
	nuttx_add_application(
		NAME dac
    SRCS dac_main.c
  )
endif()
