if (CONFIG_MODBUS)
  nuttx_add_user_library(modbus)

  set(SRCS)

  if (CONFIG_MODBUS_SLAVE)
    list(APPEND SRCS mb.c)
  endif()

  if (CONFIG_MB_RTU_MASTER)
    list(APPEND SRCS mb_m.c)
  endif()

  target_sources(modbus PRIVATE ${SRCS})

  add_subdirectory(ascii)
  add_subdirectory(functions)
  add_subdirectory(nuttx)
  add_subdirectory(rtu)
  add_subdirectory(tcp)
endif()
