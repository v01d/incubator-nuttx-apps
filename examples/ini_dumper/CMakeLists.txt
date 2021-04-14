if(CONFIG_EXAMPLES_INI_DUMPER)
  nuttx_add_application(
    NAME ${CONFIG_EXAMPLES_INI_DUMPER_PROGNAME}
    PRIORITY ${CONFIG_EXAMPLES_INI_DUMPER_PRIORITY}
    STACKSIZE ${CONFIG_EXAMPLES_INI_DUMPER_STACKSIZE}
    SRCS ini_dumper_main.c
  )

  # needed to make compiler happy about wrong function pointers
  # different function pointer is used depending on that define
  # and that is defined in ini.h

  target_add_definitions(apps_${CONFIG_EXAMPLES_INI_DUMPER_PROGNAME}
    -DINI_HANDLER_LINENO=1
  )
endif()
