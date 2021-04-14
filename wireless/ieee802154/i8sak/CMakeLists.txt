if (CONFIG_IEEE802154_I8SAK)
  nuttx_add_application(
    NAME ${CONFIG_IEEE802154_I8SAK_PROGNAME}
    PRIORITY ${CONFIG_IEEE802154_I8SAK_PRIORITY}
    STACKSIZE ${CONFIG_IEEE802154_I8SAK_STACKSIZE}
    SRCS i8sak_main.c i8sak_acceptassoc.c i8sak_assoc.c
         i8sak_scan.c i8sak_blaster.c i8sak_poll.c
         i8sak_sniffer.c i8sak_startpan.c i8sak_tx.c i8sak_get.c i8sak_set.c
         i8sak_reset.c i8sak_regdump.c i8sak_events.c)
endif()
