if (CONFIG_NXWIDGETS)
  nuttx_add_user_library(nxglyphs)

  set(SRCS
    glyph_nxlogo160x160.cxx glyph_nxlogo320x320.cxx
    glyph_arrowdown.cxx glyph_checkboxon.cxx glyph_screendepthup.cxx
    glyph_arrowleft.cxx glyph_control.cxx glyph_screenflipdown.cxx
    glyph_arrowright.cxx glyph_cycle.cxx glyph_screenflipup.cxx
    glyph_arrowup.cxx glyph_radiobuttonoff.cxx glyph_shift.cxx
    glyph_backspace.cxx glyph_radiobuttonmu.cxx glyph_windowclose.cxx
    glyph_capslock.cxx glyph_radiobuttonon.cxx glyph_windowdepthdown.cxx
    glyph_checkboxmu.cxx glyph_return.cxx glyph_windowdepthup.cxx
    glyph_checkboxoff.cxx glyph_screendepthdown.cxx)

  # Glyphs used by NxWM, Twm4Nx, and SLcd

  if (CONFIG_NXGLYPHS_LARGE_ICONS)
    # Large icons
    list(APPEND SRCS
      glyph_calculator47x49.cxx glyph_calibration48x42.cxx glyph_cmd49x43.cxx
      glyph_minimize42x42.cxx glyph_play48x48.cxx glyph_stop42x42.cxx
      glyph_menu42x42.cxx glyph_resize42x42.cxx glyph_nxicon42x42.cxx
      glpyh_menu2-42x42.cxx glpyh_minimize2-42x42.cxx glpyh_resize2-42x42.cxx
      glyph_stop2-42x42.cxx glyph_lcdclock102x48.cxx)
  else()
    # Small icons
    list(APPEND SRCS
      glyph_calculator24x25.cxx glyph_calibration24x21.cxx glyph_cmd25x22.cxx
      glyph_minimize21x21.cxx glyph_play24x24.cxx glyph_stop21x21.cxx
      glyph_menu21x21.cxx glyph_resize21x21.cxx glyph_nxicon21x21.cxx
      glpyh_menu2-21x21.cxx glpyh_minimize2-21x21.cxx glpyh_resize2-21x21.cxx
      glpyh_stop2-21x21.cxx glyph_lcdclock51x24.cxx)
  endif()

  if (CONFIG_NXGLYPHS_LARGE_ICONS)
    # Large icons
    list(APPEND SRCS
      glyph_mediaplayer44x50.cxx glyph_mplayer_controls43x41.cxx
      glyph_mediagrip60x30.cxx)
  else()
    # Small icons
    list(APPEND SRCS
      glyph_mediaplayer24x24.cxx glyph_mplayer_controls32x32.cxx)
  endif()

  list(TRANSFORM SRCS PREPEND src/)
  target_sources(nxglyphs PRIVATE ${SRCS})
endif()
