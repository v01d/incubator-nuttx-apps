if (CONFIG_NETUTILS_LIBCURL4NX)
  nuttx_add_user_library(libcurl4nx
    curl4nx_easy_init.c curl4nx_easy_cleanup.c
    curl4nx_easy_reset.c curl4nx_easy_duphandle.c
    curl4nx_easy_escape.c curl4nx_easy_unescape.c
    curl4nx_easy_setopt.c curl4nx_easy_perform.c
    curl4nx_easy_getinfo.c)
endif()
