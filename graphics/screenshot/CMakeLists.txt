if (CONFIG_GRAPHICS_SCREENSHOT)
  nuttx_add_application(
    NAME screenshot
    SRCS screenshot_main.c
    STACKSIZE 4096)
endif()
