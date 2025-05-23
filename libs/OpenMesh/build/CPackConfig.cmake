# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


set(CPACK_BUILD_SOURCE_DIRS "F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh;F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh/build")
set(CPACK_CMAKE_GENERATOR "Visual Studio 17 2022")
set(CPACK_COMPONENTS_ALL "")
set(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
set(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_FILE "C:/Program Files/CMake/share/cmake-3.30/Templates/CPack.GenericDescription.txt")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_SUMMARY "OpenMesh built using CMake")
set(CPACK_GENERATOR "NSIS")
set(CPACK_INNOSETUP_ARCHITECTURE "x64")
set(CPACK_INSTALL_CMAKE_PROJECTS "F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh/build;OpenMesh;ALL;/")
set(CPACK_INSTALL_PREFIX "C:/Program Files/OpenMesh")
set(CPACK_MODULE_PATH "F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh/cmake;F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh/cmake")
set(CPACK_NSIS_DISPLAY_NAME "OpenMesh v8.1")
set(CPACK_NSIS_DISPLAY_NAME_SET "TRUE")
set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "CreateShortcut \"$SMPROGRAMS\\OpenMesh v8.1\\Documentation.lnk\" \"$INSTDIR\\Doc\\html\\index.html \" ")
set(CPACK_NSIS_HELP_LINK "http:\\www.openmesh.org")
set(CPACK_NSIS_INSTALLER_ICON_CODE "")
set(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
set(CPACK_NSIS_PACKAGE_NAME "OpenMesh v8.1")
set(CPACK_NSIS_UNINSTALL_NAME "Uninstall")
set(CPACK_NSIS_URL_INFO_ABOUT "http:\\www.openmesh.org")
set(CPACK_OUTPUT_CONFIG_FILE "F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh/build/CPackConfig.cmake")
set(CPACK_PACKAGE_DEFAULT_LOCATION "/")
set(CPACK_PACKAGE_DESCRIPTION_FILE "C:/Program Files/CMake/share/cmake-3.30/Templates/CPack.GenericDescription.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "OpenMesh")
set(CPACK_PACKAGE_FILE_NAME "OpenMesh-8.1")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "OpenMesh 8.1")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "OpenMesh")
set(CPACK_PACKAGE_NAME "OpenMesh")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_VENDOR "ACG")
set(CPACK_PACKAGE_VERSION "8.1")
set(CPACK_PACKAGE_VERSION_MAJOR "8")
set(CPACK_PACKAGE_VERSION_MINOR "1")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_RESOURCE_FILE_LICENSE "C:/Program Files/CMake/share/cmake-3.30/Templates/CPack.GenericLicense.txt")
set(CPACK_RESOURCE_FILE_README "C:/Program Files/CMake/share/cmake-3.30/Templates/CPack.GenericDescription.txt")
set(CPACK_RESOURCE_FILE_WELCOME "C:/Program Files/CMake/share/cmake-3.30/Templates/CPack.GenericWelcome.txt")
set(CPACK_SET_DESTDIR "OFF")
set(CPACK_SOURCE_GENERATOR "TGZ;TBZ2;ZIP")
set(CPACK_SOURCE_IGNORE_FILES "\\.#;/#;.*~;/\\.git;/\\.svn;F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh/build;Makefile;Makefile\\..*;\\.moc\\.cpp$;CMakeCache.txt;CMakeFiles;/.*_(32|64)_Debug/;/.*_(32|64)_Release/;/MacOS;/WIN;/tmp/;/.*\\.kdevelop;/.*\\.kdevses;/ACG/lib/;/ACG/include/")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh/build/CPackSourceConfig.cmake")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "OpenMesh-8.1")
set(CPACK_SOURCE_STRIP_FILES "")
set(CPACK_SYSTEM_NAME "win64")
set(CPACK_THREADS "1")
set(CPACK_TOPLEVEL_TAG "win64")
set(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "F:/document/University/CIS 660/Project/VoMoBo/libs/OpenMesh/build/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
