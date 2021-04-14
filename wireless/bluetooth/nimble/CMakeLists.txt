if (CONFIG_NIMBLE)
  message(FATAL_ERROR "nimble not supported yet")
#  nuttx_add_user_library(nimble)

#  FetchContent_Declare(nimble
#    URL https://github.com/apache/mynewt-nimble/archive/${CONFIG_NIMBLE_REF}.tar.gz)
#  FetchContent_Populate(nimble)

#  # nimBLE assumes this flag since it expects undefined macros to be zero value
#  target_compile_options(libapps_nimble PRIVATE -Wno-undef)

#  NIMBLE_ROOT = $(APPDIR)/wireless/bluetooth/nimble/$(NIMBLE_UNPACKDIR)
endif()
