if(CONFIG_CANUTILS_CANSEND)
  nuttx_add_application(
    NAME cansend
    STACKSIZE 2048
    SRCS cansend.c
    DEPENDS libcanutils
  )
endif()
