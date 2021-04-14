if (CONFIG_EXAMPLES_RPMSGSOCKET)
  nuttx_add_application(
    NAME rpsock_client
    SRCS rpsock_client.c
  )

  nuttx_add_application(
    NAME rpsock_server
    SRCS rpsock_server.c
  )
endif()
