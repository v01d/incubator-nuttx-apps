if (CONFIG_NXWM)
  set(SRCS
    nxwm_main.cxx
    capplicationwindow.cxx cfullscreenwindow.cxx ctaskbar.cxx cwindowmessenger.cxx)

  # Device support
  if (CONFIG_NXWM_TOUCHSCREEN)
    list(APPEND SRCS ctouchscreen.cxx)
  endif()

  if (CONFIG_NXWM_KEYBOARD)
    list(APPEND SRCS ckeyboard.cxx)
  endif()

  # Applications

  list(APPEND SRCS cstartwindow.cxx chexcalculator.cxx)

  if (CONFIG_NXWM_TOUCHSCREEN)
    list(APPEND SRCS ccalibration.cxx)
  endif()

  if (CONFIG_NXWM_NXTERM)
    list(APPEND SRCS cnxterm.cxx)
  endif()

  if (CONFIG_NXWM_MEDIAPLAYER)
    list(APPEND SRCS cmediaplayer.cxx)
  endif()

  list(TRANSFORM SRCS PREPEND src/)
  nuttx_add_application(
    NAME nxwm
    SRCS ${SRCS})
endif()
