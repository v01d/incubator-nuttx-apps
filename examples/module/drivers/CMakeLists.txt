add_subdirectory(chardev)

if (CONFIG_EXAMPLES_MODULE_BUILTINFS)
  set(FSROOT ${CMAKE_CURRENT_BINARY_DIR}/fsroot)

  add_custom_command(
    OUTPUT ${FSROOT}/chardev
    COMMAND ${CMAKE_COMMAND} -E make_directory fsroot
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:chardev> fsroot
    DEPENDS chardev
  )
  add_custom_target(chardev_fsroot DEPENDS ${FSROOT}/chardev)

  if (CONFIG_EXAMPLES_MODULE_ROMFS)
    nuttx_add_romfs(
      NAME module
      PATH ${FSROOT})
    add_dependencies(romfs_module chardev_fsroot)
  elseif (CONFIG_EXAMPLES_MODULE_CROMFS)
    nuttx_add_romfs(
      NAME module
      PATH ${FSROOT})
    add_dependencies(cromfs_module chardev_fsroot)
  endif()
endif()

# Create the exported symbol table
nuttx_add_symtab(NAME mod BINARIES chardev PREFIX g_mod)
target_link_libraries(apps PRIVATE symtab_mod)

# TODO: update when two-pass build is implemented
# Copy the symbol table into the kernel pass1/ build directory
#if (NOT CONFIG_BUILD_FLAT)
#add_custom_command(
#  OUTPUT ${CMAKE_BINARY_DIR}/pass1/mod_symtab.c
#  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/mod_symtab.c ${CMAKE_BINARY_DIR}/pass1/mod_symtab.c)
#  #$(Q) install -m 0644 $(SYMTAB_SRC) $(PASS1_SYMTAB)
#endif()
