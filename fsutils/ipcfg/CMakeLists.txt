if (CONFIG_FSUTILS_IPCFG)
  set(SRCS ipcfg.c)

  if (CONFIG_IPCFG_BINARY)
    list(APPEND SRCS ipcfg_binary.c)
  else()
    list(APPEND SRCS ipcfg_text.c)
  endif()

  target_sources(libfsutils PRIVATE ${SRCS})
endif()
