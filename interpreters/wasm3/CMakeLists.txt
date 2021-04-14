if (CONFIG_INTERPRETERS_WASM3)
  set(WASM3_VERSION 0.4.7)
  FetchContent_Declare(wasm3
    URL https://github.com/wasm3/wasm3/archive/v${WASM3_VERSION}.tar.gz)
  FetchContent_Populate(wasm3)
  FetchContent_GetProperties(wasm3 SOURCE_DIR WASM3_SOURCE_DIR)

  set(SRCS
    m3_bind.c m3_code.c m3_compile.c m3_core.c
    m3_emit.c m3_env.c m3_exec.c m3_info.c
    m3_module.c m3_parse.c m3_api_libc.c)
  list(TRANSFORM SRCS PREPEND ${WASM3_SOURCE_DIR}/source/)

  nuttx_add_application(
    NAME wasm3
    PRIORITY ${CONFIG_INTERPRETERS_WASM3_PRIORITY}
    STACKSIZE ${CONFIG_INTERPRETERS_WASM3_STACKSIZE}
    SRCS ${WASM3_SOURCE_DIR}/platforms/app/main.c ${SRCS})

  target_compile_options(libapps_wasm3 PRIVATE
    -Wno-unused-function -Wno-unused-variable
    -Wno-strict-prototypes)

  if (CONFIG_INTERPRETERS_WASM3_FIXEDHEAP)
    target_compile_options(libapps_wasm3 PRIVATE -Dd_m3FixedHeap=${CONFIG_INTERPRETERS_WASM3_FIXEDHEAP})
  endif()

  if (CONFIG_INTERPRETERS_WASM3_CODEPAGE)
    target_compile_options(libapps_wasm3 PRIVATE -Dd_m3CodePageAlignSize=${CONFIG_INTERPRETERS_WASM3_CODEPAGE})
  endif()

  if (NOT CONFIG_INTERPRETERS_WASM3_LOG)
    target_compile_options(libapps_wasm3 PRIVATE -Dd_m3LogOutput=0)
  endif()

  if (NOT CONFIG_INTERPRETERS_WASM3_LOG_VERBOSE)
    target_compile_options(libapps_wasm3 PRIVATE -Dd_m3VerboseLogs=0)
  endif()
endif()
