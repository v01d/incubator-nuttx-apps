if (CONFIG_INTERPRETERS_WAMR)
  set(WAMR_VERSION ${CONFIG_INTERPRETERS_WAMR_VERSION})

  # Tarball of main branch and tagged release have little difference
  # in name after unpack and URL

  set(WAMR_VERSION_STUB)
  if (NOT "${WAMR_VERSION}" STREQUAL "main")
    set(WAMR_VERSION_STUB "WAMR-")
  endif()

  FetchContent_Declare(wamr
    URL https://github.com/bytecodealliance/wasm-micro-runtime/archive/${WAMR_VERSION_STUB}${WAMR_VERSION}.zip)
  FetchContent_Populate(wamr)
  FetchContent_GetProperties(wamr SOURCE_DIR WAMR_SOURCE_DIR)

  set(SRCS
    ${WAMR_SOURCE_DIR}/product-mini/platforms/nuttx/main.c
    # TODO: add sources as in $(WAMR_UNPACK)/product-mini/platforms/nuttx/wamr.mk
    )

  nuttx_add_application(
    NAME iwasm
    PRIORITY ${CONFIG_INTERPRETERS_WAMR_PRIORITY}
    STACKSIZE ${CONFIG_INTERPRETERS_WAMR_STACKSIZE}
    SRCS ${SRCS})
endif()
