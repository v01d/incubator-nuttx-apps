nuttx_add_module(chardev chardev.c)

if (CONFIG_EXAMPLES_MODULE_LIBC)
  if (NOT CONFIG_BUILD_FLAT)
    target_link_libraries(chardev PRIVATE kc)
  else()
    target_link_libraries(chardev PRIVATE c)
  endif()
endif()

if (CONFIG_EXAMPLES_MODULE_LIBGCC)
#  set(LIBGCC "${shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name 2>/dev/null}")

#  if (NOT LIBGCC)
#    LDLIBPATH += -L "${shell dirname $(LIBGCC)}"
#    LDLIBS += -lgcc
#  endif()
  target_link_libraries(chardev PRIVATE gcc)
endif()

if (CONFIG_EXAMPLES_MODULE_LIBC)
  target_link_options(chardev -Bstatic)
  #LDLIBPATH +=  -L $(NUTTXLIB)
endif()

