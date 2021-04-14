if (CONFIG_SYSTEM_ADBD)
  FetchContent_Declare(adb
    GIT_REPOSITORY https://github.com/spiriou/microADB.git
    GIT_TAG b7025c67b866925d1e64c016a844a6a3392557a4)
  FetchContent_Populate(adb)
  FetchContent_GetProperties(adb SOURCE_DIR ADB_SOURCE_DIR)

  # Files

  set(SRCS
    adb_main.c adb_banner.c
    ${ADB_SOURCE_DIR}/adb_client.c ${ADB_SOURCE_DIR}/adb_frame.c
    ${ADB_SOURCE_DIR}/hal/hal_uv.c ${ADB_SOURCE_DIR}/hal/hal_uv_packet.c)


  if (CONFIG_ADBD_TCP_SERVER)
    list(APPEND SRCS ${ADB_SOURCE_DIR}/hal/hal_uv_client_tcp.c)
  endif()

  if (CONFIG_ADBD_USB_SERVER)
    list(APPEND SRCS ${ADB_SOURCE_DIR}/hal/hal_uv_client_usb.c)
  endif()

  if (CONFIG_ADBD_AUTHENTICATION)
    list(APPEND SRCS ${ADB_SOURCE_DIR}/adb_auth_key.c)
  endif()

  if (CONFIG_ADBD_FILE_SERVICE)
    list(APPEND SRCS ${ADB_SOURCE_DIR}/file_sync_service.c)
  endif()

  if (CONFIG_ADBD_LOGCAT_SERVICE)
    list(APPEND SRCS logcat_service.c)
  endif()

  if (CONFIG_ADBD_SHELL_SERVICE)
    list(APPEND SRCS shell_service.c shell_pipe.c)
  endif()

  nuttx_add_application(
    NAME ${CONFIG_ADBD_PROGNAME}
    PRIORITY ${CONFIG_ADBD_PRIORITY}
    STACKSIZE ${CONFIG_ADBD_STACKSIZE}
    SRCS ${SRCS})

  target_compile_options(libapps_${CONFIG_ADBD_PROGNAME} PRIVATE -D__NUTTX__=1)
  target_include_directories(libapps_${CONFIG_ADBD_PROGNAME} PRIVATE ${ADB_SOURCE_DIR})
endif()
