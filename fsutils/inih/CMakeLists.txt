if (CONFIG_FSUTILS_INIH)
  set(INIH_VERSION r42)

  FetchContent_Declare(inih
    URL https://github.com/benhoyt/inih/archive/inih-${INIH_VERSION}.tar.gz)
  FetchContent_Populate(inih)
  FetchContent_GetProperties(inih SOURCE_DIR INIH_SOURCE_DIR)

  set(INIH_FLAGS -DINI_MAX_LINE=${CONFIG_INIH_MAX_LINE})

  if (CONFIG_INIH_MULTI_LINE_ENTRIES)
    list(APPEND INIH_FLAGS -DINI_ALLOW_MULTILINE=1)
  else()
    list(APPEND INIH_FLAGS -DINI_ALLOW_MULTILINE=0)
  endif()

  if (CONFIG_INIH_USE_MALLOC)
    list(APPEND INIH_FLAGS
      -DINI_USE_STACK=0
      -DINI_ALLOW_REALLOC=1
      -DINI_INITIAL_ALLOC=${CONFIG_INIH_INITIAL_ALLOC})
  else()
    list(APPEND INIH_FLAGS
      -DINI_USE_STACK=1
      -DINI_ALLOW_REALLOC=0)
  endif()

  # compile-time configuration of inih

  list(APPEND INIH_FLAGS
    -DINI_ALLOW_BOM=0
    -DINI_ALLOW_INLINE_COMMENTS=1
    -DINI_STOP_ON_FIRST_ERROR=0
    -DINI_HANDLER_LINENO=1)

  nuttx_add_user_library(inih ${INIH_SOURCE_DIR}/ini.c)
  target_compile_definitions(inih PRIVATE ${INIH_FLAGS})
  target_include_directories(inih INTERFACE ${INIH_SOURCE_DIR})
endif()
