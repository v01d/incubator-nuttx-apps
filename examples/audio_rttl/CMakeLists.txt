if(CONFIG_EXAMPLES_AUDIO_SOUND)
  nuttx_add_application(
    NAME ${CONFIG_EXAMPLES_AUDIO_SOUND_PROGNAME}
    PRIORITY ${CONFIG_EXAMPLES_AUDIO_SOUND_PRIORITY}
    STACKSIZE ${CONFIG_EXAMPLES_AUDIO_SOUND_STACKSIZE}
    SRCS audio_rttl.cxx
    COMPILE_FLAGS -D_POSIX
  )

  target_compile_definitions(
    apps_${CONFIG_EXAMPLES_AUDIO_SOUND_PROGNAME} _POSIX
  )

  # TODO: the Make.defs points to SDKDIR which should really be provided
  # with Kconfig

  #target_include_directories(
  #  apps_${CONFIG_EXAMPLES_AUDIO_SOUND_PROGNAME}
  #  SDKDIR/modules/audio?
  #)
endif()
