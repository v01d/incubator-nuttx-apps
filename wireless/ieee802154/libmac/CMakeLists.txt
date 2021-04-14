if (CONFIG_IEEE802154_LIBMAC)
  nuttx_add_user_library(ieee802154_libmac)

  # libmac source files
  set(SRCS
    ieee802154_assocreq.c ieee802154_assocresp.c ieee802154_disassocreq.c
    ieee802154_getreq.c ieee802154_gtsreq.c ieee802154_orphanresp.c
    ieee802154_resetreq.c ieee802154_rxenabreq.c  ieee802154_scanreq.c
    ieee802154_setreq.c ieee802154_startreq.c ieee802154_syncreq.c
    ieee802154_pollreq.c)

  if (CONFIG_NET_6LOWPAN)
    list(APPEND SRCS
      sixlowpan_assocreq.c sixlowpan_assocresp.c sixlowpan_disassocreq.c
      sixlowpan_getreq.c sixlowpan_gtsreq.c sixlowpan_orphanresp.c
      sixlowpan_resetreq.c sixlowpan_rxenabreq.c sixlowpan_scanreq.c
      sixlowpan_setreq.c sixlowpan_startreq.c sixlowpan_syncreq.c
      sixlowpan_pollreq.c)
  endif()

  target_sources(ieee802154_libmac PRIVATE ${SRCS})
endif()
