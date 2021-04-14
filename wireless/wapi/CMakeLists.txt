if (CONFIG_WIRELESS_WAPI)
  nuttx_add_user_library(wapi
    src/network.c src/util.c src/wireless.c src/driver_wext.c)

  if (CONFIG_WIRELESS_WAPI_CMDTOOL)
    nuttx_add_application(
      NAME ${CONFIG_WIRELESS_WAPI_PROGNAME}
      PRIORITY ${CONFIG_WIRELESS_WAPI_PRIORITY}
      STACKSIZE ${CONFIG_WIRELESS_WAPI_STACKSIZE}
      SRCS src/wapi.c
      DEPENDS wapi)
  endif()
endif()
