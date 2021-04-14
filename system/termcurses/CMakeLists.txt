if (CONFIG_SYSTEM_TERMCURSES)
  nuttx_add_user_library(termcurses termcurses.c tcurses_vt100.c)
endif()
