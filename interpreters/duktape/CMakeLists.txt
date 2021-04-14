if (CONFIG_INTERPRETERS_DUKTAPE)
  set(DUKTAPE_VERSION 2.5.0)
  FetchContent_Declare(duktape
    URL https://github.com/svaarala/duktape/releases/download/v${DUKTAPE_VERSION}/duktape-${DUKTAPE_VERSION}.tar.xz
    PATCH_COMMAND patch -p0 < duk_cmdline.patch)

  FetchContent_Populate(duktape)
  FetchContent_GetProperties(duktape SOURCE_DIR DUKTAPE_SOURCE_DIR)

  set(SRCS
    ${DUKTAPE_SOURCE_DIR}/examples/cmdline/duk_cmdline.c
    ${DUKTAPE_SOURCE_DIR}/src-noline/duktape.c
    ${DUKTAPE_SOURCE_DIR}/extras/console/duk_console.c
    ${DUKTAPE_SOURCE_DIR}/extras/print-alert/duk_print_alert.c)

  nuttx_add_application(
    NAME duk
    PRIORITY ${CONFIG_INTERPRETERS_DUKTAPE_PRIORITY}
    STACKSIZE ${CONFIG_INTERPRETERS_DUKTAPE_STACKSIZE}
    SRCS ${SRCS})

  target_include_directories(libapps_duktape PUBLIC ${DUKTAPE_SOURCE_DIR}/src-noline)
  target_include_directories(libapps_duktape PRIVATE
    -I${DUKTAPE_SOURCE_DIR}/extras/console
    -I${DUKTAPE_SOURCE_DIR}/extras/print-alert)

  target_compile_options(libapps_duktape PRIVATE
    -I$(DUKTAPE_UNPACK)/src-noline
    -DDUK_CMDLINE_CONSOLE_SUPPORT
    -DDUK_CMDLINE_PRINTALERT_SUPPORT)
endif()
