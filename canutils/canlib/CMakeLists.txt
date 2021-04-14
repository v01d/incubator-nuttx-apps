if(CONFIG_CANUTILS_CANLIB)
  nuttx_add_user_library(canlib
    canlib_getbaud.c canlib_setbaud.c
    canlib_getloopback.c canlib_setloopback.c
    canlib_getsilent.c canlib_setsilent.c
  )
endif()
