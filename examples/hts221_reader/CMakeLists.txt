if(CONFIG_EXAMPLES_HTS221_READER)
  nuttx_add_application(
    NAME hts221_reader
    SRCS hts221_reader_main.c
  )
endif()
