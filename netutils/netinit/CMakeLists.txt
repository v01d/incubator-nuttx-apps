if (CONFIG_NETUTILS_NETINIT)
  set(SRCS netinit.c)

  if (CONFIG_WIRELESS_WAPI AND NOT CONFIG_NETINIT_NETLOCAL)
    list(APPEND SRCS netinit_associate.c)
  endif()

  nuttx_add_user_library(netinit ${SRCS})
endif()
