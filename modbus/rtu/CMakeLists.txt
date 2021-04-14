if (CONFIG_MB_RTU_ENABLED OR CONFIG_MB_RTU_MASTER)
  set(SRCS mbcrc.c)

  if (CONFIG_MB_RTU_ENABLED)
    list(APPEND SRCS  mbrtu.c)
  endif()

  if (CONFIG_MB_RTU_MASTER)
    list(APPEND SRCS mbrtu_m.c)
  endif()

  target_sources(modbus PRIVATE ${SRCS})
endif()
