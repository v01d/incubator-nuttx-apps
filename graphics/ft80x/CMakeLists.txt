if (CONFIG_GRAPHICS_FT80X)
  nuttx_add_user_library(ft80x)

  target_sources(ft80x PRIVATE
    ft80x_dl.c ft80x_ramg.c ft80x_ramdl.c ft80x_ramcmd.c
    ft80x_coproc.c ft80x_touch.c ft80x_audio.c ft80x_backlight.c
    ft80x_gpio.c ft80x_regs.c)
endif()
