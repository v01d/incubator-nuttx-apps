if (CONFIG_EXAMPLES_PWFB)
  nuttx_add_application(
    NAME ${CONFIG_EXAMPLES_PWFB_PROGNAME}
    PRIORITY ${CONFIG_EXAMPLES_PWFB_CLIENT_PRIO}
    STACKSIZE ${CONFIG_EXAMPLES_PWFB_CLIENT_STACKSIZE}
    SRCS pwfb_main.c pwfb_events.c pwfb_motion.c
  )

  target_include_directories(apps_${CONFIG_EXAMPLES_PWFB_PROGNAME}
    ${NUTTX_APPS_DIR}/graphics/nxglyphs)
endif()
