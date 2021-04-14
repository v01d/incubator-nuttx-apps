if(CONFIG_CANUTILS_CANDUMP)
  nuttx_add_application(
    NAME candump
    SRCS nsh_main.c
    DEPENDS libcanutils
    STACKSIZE 3072
  )
endif()
