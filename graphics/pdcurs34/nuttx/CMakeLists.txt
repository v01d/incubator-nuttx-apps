set(SRCS pdcclip.c pdcdisp.c pdcgetsc.c pdckbd.c pdcscrn.c pdcsetsc.c pdcutil.c)

if (NOT CONFIG_PDCURSES_MULTITHREAD)
  list(APPEND SRCS pdcthread.c)
endif

target_sources(pdcurs34 PRIVATE ${SRCS})
