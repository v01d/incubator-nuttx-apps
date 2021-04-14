list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Declare apps library
nuttx_add_system_library(apps)

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/empty.c" "")
target_sources(apps PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/empty.c")

# Include all apps subdirectories

add_subdirectory(canutils)
add_subdirectory(examples)
add_subdirectory(fsutils)
add_subdirectory(gpsutils)
add_subdirectory(graphics)
#add_subdirectory(import)
add_subdirectory(industry)
add_subdirectory(interpreters)
add_subdirectory(modbus)
add_subdirectory(netutils)
add_subdirectory(nshlib)
#add_subdirectory(platform)
add_subdirectory(system)
add_subdirectory(testing)
#add_subdirectory(tools)
add_subdirectory(wireless)

if (EXISTS external/CMakeLists.txt)
  add_subdirectory(external)
endif()

# This needs to appear last, since it collects all
# builtin app declarations

add_subdirectory(builtin)

# loadable application support

get_property(loadable_apps TARGET nuttx PROPERTY NUTTX_LOADABLE_APPS)

if (loadable_apps)
  nuttx_add_symtab(NAME apps BINARIES ${loadable_apps})
  target_link_libraries(apps PRIVATE symtab_apps)
endif()
