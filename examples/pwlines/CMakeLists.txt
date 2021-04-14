if(CONFIG_EXAMPLES_PWLINES)
  nuttx_add_application(
    NAME ${CONFIG_EXAMPLES_PWLINES_PROGNAME}
    PRIORITY ${CONFIG_EXAMPLES_PWLINES_CLIENT_PRIO}
    STACKSIZE ${CONFIG_EXAMPLES_PWLINES_CLIENT_STACKSIZE}
    SRCS pwlines_main.c pwlines_events.c pwlines_motion.c pwlines_update.c
  )
endif()
