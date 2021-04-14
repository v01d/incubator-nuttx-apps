nuttx_add_module(modprint modprint.c)

if (CONFIG_EXAMPLES_SOTEST_LIBGCC)
#  #LIBGCC = "${shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name 2>/dev/null}"
#  if (NOT LIBGCC)
#    LDLIBPATH += -L "${shell dirname $(LIBGCC)}"
#    LDLIBS += -lgcc
#  endif()
endif()

if (CONFIG_EXAMPLES_SOTEST_LIBC)
  target_link_libraries(modprint PRIVATE c)
  target_link_options(modprint PRIVATE -Bstatic)
  #LDLIBPATH +=  -L $(NUTTXLIB)
endif()

if (CONFIG_EXAMPLES_SOTEST_BUILTINFS)
  add_custom_command(
    TARGET modprint POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:modprint> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
    DEPENDS modprint)
endif()
