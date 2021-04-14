set(TESTS_SUBDIRS errno hello struct signal)

if (CONFIG_HAVE_CXX)
  list(APPEND TESTS_SUBDIRS helloxx)
endif()

if (CONFIG_EXAMPLES_ELF_LONGJMP)
  list(APPEND TESTS_SUBDIRS longjmp)
endif()

if (NOT CONFIG_DISABLE_PTHREAD)
  list(APPEND TESTS_SUBDIRS mutex pthread)
endif()

if (NOT CONFIG_ARCH_ADDRENV)
  list(APPEND TESTS_SUBDIRS task)
endif()

set(ENABLED_BINARIES)
file(REMOVE ${CMAKE_CURRENT_BINARY_DIR}/dirlist.h)
foreach(subdir ${TESTS_SUBDIRS})
  add_subdirectory(${subdir})

  get_property(targets DIRECTORY ${subdir} PROPERTY BUILDSYSTEM_TARGETS)
  foreach(target ${targets})
    if (CONFIG_EXAMPLES_ELF_SYSCALL OR CONFIG_EXAMPLES_ELF_LIBC)
      target_link_options(${target} PRIVATE -Bstatic)
    endif()

    if (CONFIG_EXAMPLES_ELF_LIBC)
      target_link_libraries(${target} PRIVATE c)
    endif()

    if (CONFIG_EXAMPLES_ELF_SYSCALL)
      target_link_libraries(${target} PRIVATE proxies)
    endif()

    list(APPEND ENABLED_BINARIES ${target})
  endforeach()

  file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/dirlist.h "\"${subdir}\"")
endforeach()

set(ENABLED_BINARIES ${ENABLED_BINARIES} PARENT_SCOPE)
