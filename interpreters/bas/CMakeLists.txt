if (CONFIG_INTERPRETERS_BAS)
  # BAS Library
  set(SRCS
    bas_main.c
    bas.c bas_auto.c bas_fs.c bas_global.c bas_program.c
    bas_str.c bas_token.c bas_value.c bas_var.c)

  if (CONFIG_INTERPRETER_BAS_VT100)
    list(APPEND SRCS bas_vt100.c)
  endif()

  nuttx_add_application(
    NAME bas
    PRIORITY ${CONFIG_INTERPRETER_BAS_PRIORITY}
    STACKSIZE ${CONFIG_INTERPRETER_BAS_STACKSIZE}
    SRCS ${SRCS})
endif()
