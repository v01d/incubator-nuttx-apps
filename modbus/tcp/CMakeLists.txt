if (CONFIG_MB_TCP_ENABLED)
  target_sources(modbus PRIVATE mbtcp.c)
endif()
