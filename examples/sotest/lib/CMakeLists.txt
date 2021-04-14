add_subdirectory(sotest)

if (NOT CONFIG_MODLIB_MAXDEPEND EQUAL 0)
  add_subdirectory(modprint)
  add_custom_target(modprint_fsroot ${CMAKE_CURRENT_BINARY_DIR}/fsroot/modprint)
endif()

if (CONFIG_EXAMPLES_SOTEST_BUILTINFS)
  set(ROMFS_DEPENDS sotest)
  if (NOT CONFIG_MODLIB_MAXDEPEND EQUAL 0)
    list(APPEND ROMFS_DEPENDS modprint)
  endif()

  nuttx_add_romfs(NAME sotest PATH ${CMAKE_CURRENT_BINARY_DIR}/fsroot DEPENDS ${ROMFS_DEPENDS})
endif()

nuttx_add_symtab(NAME sotest PREFIX g_sot BINARIES sotest modprint EXCLUDE modprint)
