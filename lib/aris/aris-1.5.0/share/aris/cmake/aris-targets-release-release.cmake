#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "aris::aris_lib" for configuration "Release"
set_property(TARGET aris::aris_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(aris::aris_lib PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/release/libaris_lib.so"
  IMPORTED_SONAME_RELEASE "libaris_lib.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS aris::aris_lib )
list(APPEND _IMPORT_CHECK_FILES_FOR_aris::aris_lib "${_IMPORT_PREFIX}/lib/release/libaris_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
