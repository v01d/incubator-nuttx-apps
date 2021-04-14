nuttx_add_module(sotest sotest.c)

if (CONFIG_EXAMPLES_SOTEST_LIBGCC)
#  #LIBGCC = "${shell $(CC) $(ARCHCPUFLAGS) -print-libgcc-file-name 2>/dev/null}"
#  if (NOT LIBGCC)
#    LDLIBPATH += -L "${shell dirname $(LIBGCC)}"
#    LDLIBS += -lgcc
#  endif()
endif()

if (CONFIG_EXAMPLES_SOTEST_LIBC)
  target_link_libraries(sotest PRIVATE c)
  target_link_options(sotest PRIVATE -Bstatic)
  #LDLIBPATH +=  -L $(NUTTXLIB)
endif()

if (CONFIG_EXAMPLES_SOTEST_BUILTINFS)
  add_custom_command(
    TARGET sotest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:sotest> ${CMAKE_CURRENT_BINARY_DIR}/../fsroot
    DEPENDS sotest)
endif()
