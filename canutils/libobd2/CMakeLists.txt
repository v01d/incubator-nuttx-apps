if(CONFIG_CANUTILS_LIBOBD2)
  nuttx_add_user_libary(libobd2
    obd2.c obd_sendrequest.c obd_waitresponse.c obd_decodepid.c
  )
endif()
