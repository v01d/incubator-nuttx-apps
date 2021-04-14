add_subdirectory(chardev)

set(FSROOT ${CMAKE_CURRENT_BINARY_DIR}/fsroot)

if (CONFIG_EXAMPLES_MODULE_BUILTINFS)
  if (CONFIG_EXAMPLES_MODULE_ROMFS)
    nuttx_add_romfs(
      NAME module
      PATH ${FSROOT})
  elseif (CONFIG_EXAMPLES_MODULE_CROMFS)
    nuttx_add_romfs(
      NAME module
      PATH ${FSROOT})
  endif()
endif()

# Create the exported symbol table

add_custom_command(
  OUTPUT mod_symtab.c
  COMMAND ${CMAKE_BINARY_DIR}/bin/mksymtab.sh ${FSROOT} g_mod > mod_symtab.c)

# Copy the symbol table into the kernel pass1/ build directory

if (NOT CONFIG_BUILD_FLAT)
add_custom_command(
  OUTPUT ${CMAKE_BINARY_DIR}/pass1/mod_symtab.c
  COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/mod_symtab.c ${CMAKE_BINARY_DIR}/pass1/mod_symtab.c)
  #$(Q) install -m 0644 $(SYMTAB_SRC) $(PASS1_SYMTAB)
endif()
