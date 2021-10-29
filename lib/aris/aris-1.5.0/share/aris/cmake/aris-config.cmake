
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was aris-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

#################################################################################### 

set(RELY_LINK_DIR "/opt/etherlab/lib;/usr/xenomai/lib")
link_directories(${RELY_LINK_DIR})

#set_and_check(aris_INCLUDE_DIRS "/usr/aris/aris-1.5.0/include")
set_and_check(aris_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include")
set(aris_LIBRARIES "optimized;aris::aris_lib;debug;aris::debug::aris_lib;ethercat_rtdm;-Wl,--no-as-needed -Wl,@/usr/xenomai/lib/modechk.wrappers -lalchemy -lcopperplate /usr/xenomai/lib/xenomai/bootstrap-pic.o;-L/usr/xenomai/lib;-lcobalt;-lmodechk;-lpthread;-lrt;atomic;pthread;stdc++fs")
if(UNIX)
	set(aris_LIBRARIES -Wl,--start-group ${aris_LIBRARIES} -Wl,--end-group)
endif(UNIX)


include("${CMAKE_CURRENT_LIST_DIR}/aris-targets-release.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/aris-targets-debug.cmake")
