# - Try to find btatofapi headers and binaries
# Once done this will define
#
option (BTA_ETH "Build using the BTA Ethernet library" OFF)
option (BTA_P100 "Build using the BTA P100(usb) library" OFF)

message( BTA_ETH:  ${BTA_ETH} )
message( BTA_P100:  ${BTA_P100} )

FIND_PATH(bta_INCLUDE_DIR NAMES bta.h
 	PATHS
		/usr/local/include/libbta/
        /usr/include/libbta/
        ../include/
        inc/
        #Bta/inc/
        #bta_sensor/Bta/inc
        ./
 	NO_DEFAULT_PATH
	DOC "Include directory of bta"
)

set(ARCH_DIR "x86")
if (WIN32)
  if (CMAKE_CL_64)
      set(ARCH_DIR "x64")
  endif()
elseif (UNIX)
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(ARCH_DIR "x64")
   endif()
endif()

message( ARCH_DIR:  ${ARCH_DIR} )

set (bta_name bta)
if (WIN32)
	set (bta_name BltTofApi)
endif()

if( BTA_ETH)
    if(MSVC)
      set (bta_name BtaEth)
    else()
        set (bta_name bta_eth)
    endif()
endif()

if( BTA_P100 )
    if(MSVC)
        set (bta_name BtaP100)
    else()
        set (bta_name bta_p100)
    endif()
endif()


find_library(bta_LIBRARY NAMES ${bta_name}
  PATHS
    lib/Lin_x64/
        /usr/lib/
        /usr/local/lib/
		../windows/lib/${ARCH_DIR}/
		../Bta/${ARCH_DIR}/
		Bta/lib/BtaEth/${ARCH_DIR}/
		Bta/lib/Bta/${ARCH_DIR}/
		Bta/lib/BtaP100/${ARCH_DIR}/
	NO_DEFAULT_PATH
	DOC "Library binary"
)
#set(bta_LIBRARIES ${bta_eth_LIBRARY} )

if (BTA_ETH)
	add_definitions(-DBTA_ETH)
elseif (BTA_P100)
	add_definitions(-DBTA_P100)
endif()

set(bta_LIBRARIES ${bta_LIBRARY})
message( ${bta_LIBRARIES} )
#Not needed by now as we have just 1 include and lib

set(bta_INCLUDE_DIRS ${bta_INCLUDE_DIR} )
message( ${bta_INCLUDE_DIRS} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set bta_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(bta  DEFAULT_MSG
                                  bta_LIBRARIES bta_INCLUDE_DIRS)

mark_as_advanced(bta_INCLUDE_DIRS bta_LIBRARIES)
