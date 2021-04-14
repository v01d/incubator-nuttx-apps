if(CONFIG_CANUTILS_SLCAN)
  nuttx_add_application(
    NAME slcan
    STACKSIZE 3048
    SRCS slcan.c
  )
endif()
