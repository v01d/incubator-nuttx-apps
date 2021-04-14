if (CONFIG_FSUTILS_FLASH_ERASEALL OR
    CONFIG_FSUTILS_INIFILE OR
    CONFIG_FSUTILS_INIH OR
    CONFIG_FSUTILS_IPCFG OR
    CONFIG_FSUTILS_MKFATFS OR
    CONFIG_FSUTILS_MKSMARTFS OR
    CONFIG_FSUTILS_PASSWD)

  # TODO: define CONFIG_FSUTILS to detect when
  # either is set

  nuttx_add_user_library(libfsutils)

  add_subdirectory(flash_eraseall)
  add_subdirectory(inifile)
  add_subdirectory(mkfatfs)
  add_subdirectory(mksmartfs)
  add_subdirectory(passwd)
endif()
