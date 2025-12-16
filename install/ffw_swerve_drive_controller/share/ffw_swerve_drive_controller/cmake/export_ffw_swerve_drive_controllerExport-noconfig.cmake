#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ffw_swerve_drive_controller::ffw_swerve_drive_controller" for configuration ""
set_property(TARGET ffw_swerve_drive_controller::ffw_swerve_drive_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ffw_swerve_drive_controller::ffw_swerve_drive_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libffw_swerve_drive_controller.so"
  IMPORTED_SONAME_NOCONFIG "libffw_swerve_drive_controller.so"
  )

list(APPEND _cmake_import_check_targets ffw_swerve_drive_controller::ffw_swerve_drive_controller )
list(APPEND _cmake_import_check_files_for_ffw_swerve_drive_controller::ffw_swerve_drive_controller "${_IMPORT_PREFIX}/lib/libffw_swerve_drive_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
