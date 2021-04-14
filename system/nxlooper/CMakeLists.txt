if (CONFIG_SYSTEM_NXLOOPER)
  nuttx_add_user_library(nxlooper nxlooper.c)

  if (CONFIG_NXLOOPER_COMMAND_LINE)
    nuttx_add_application(
      NAME nxlooper
      PRIORITY SCHED_PRIORITY_DEFAULT
      STACKSIZE ${CONFIG_NXLOOPER_MAINTHREAD_STACKSIZE}
      SRCS nxlooper_main.c)
  endif()
endif()
