set(SRCS portother.c)

if (CONFIG_MODBUS_SLAVE)
  list(APPEND SRCS portevent.c portserial.c porttimer.c)
endif()

if (CONFIG_MB_RTU_MASTER)
  list(APPEND SRCS portother_m.c portserial_m.c porttimer_m.c portevent_m.c)
endif()
