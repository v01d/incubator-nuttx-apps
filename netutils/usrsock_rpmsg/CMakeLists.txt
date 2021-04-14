if (CONFIG_NETUTILS_USRSOCK_RPMSG)
  if (CONFIG_NET_USRSOCK)
    set(SRCS usrsock_rpmsg_client.c)
  else()
    set(SRCS usrsock_rpmsg_server.c)
  endif()

  nuttx_add_application(
    NAME usrsock
    PRIORITY ${CONFIG_NETUTILS_USRSOCK_RPMSG_PRIORITY}
    STACKSIZE ${CONFIG_NETUTILS_USRSOCK_RPMSG_STACKSIZE}
    SRCS ${SRCS})
endif()
