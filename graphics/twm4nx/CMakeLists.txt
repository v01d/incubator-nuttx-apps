if (CONFIG_GRAPHICS_TWM4NX)
  # Twm4Nx

  set(SRCS twm4nx_main.cxx
    cbackground.cxx cfonts.cxx ciconmgr.cxx ciconwidget.cxx
    cmenus.cxx cmainmenu.cxx
    cwindow.cxx cwindowevent.cxx cresize.cxx cwindowfactory.cxx
    cinput.cxx ctwm4nx.cxx)

  if (CONFIG_TWM4NX_MOUSE)
    list(APPEND SRCS twm4nx_cursor.cxx)
  endif()

  list(TRANSFORM SRCS PREPEND src/)

  if (CONFIG_TWM4NX_CALIBRATION)
    list(APPEND SRCS apps/ccalibration.cxx)
  endif()

  if (CONFIG_TWM4NX_CLOCK)
    list(APPEND SRCS apps/cclock.cxx)
  endif()

  if (CONFIG_TWM4NX_NXTERM)
    list(APPEND SRCS apps/cnxterm.cxx)
  endif()

  nuttx_add_application(
    PROGNAME  ${CONFIG_TWM4NX_PROGNAME}
    PRIORITY  ${CONFIG_TWM4NX_PRIORITY}
    STACKSIZE ${CONFIG_TWM4NX_STACKSIZE}
    SRCS ${SRCS})

  target_link_libraries(apps_twm4nx PRIVATE nxglyphs)
endif()
