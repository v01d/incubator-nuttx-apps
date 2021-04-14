set(SRCS
  mbfunccoils.c mbfuncdiag.c mbfuncdisc.c mbfuncholding.c
  mbfuncinput.c mbfuncother.c mbutils.c)

if (CONFIG_MB_ASCII_MASTER)
  list(APPEND SRCS mbfunccoils_m.c mbfuncdisc_m.c mbfuncholding_m.c
    mbfuncinput_m.c)
else()
  if (CONFIG_MB_RTU_MASTER)
    list(APPEND SRCS mbfunccoils_m.c mbfuncdisc_m.c mbfuncholding_m.c
      mbfuncinput_m.c)
  endif()
endif()

target_sources(modbus PRIVATE ${SRCS})
