#############################################################
# Minimum required version                                  #
#############################################################
cmake_minimum_required(VERSION 3.1)
include(CMakeDependentOption)

#############################################################
# Platform option                                           #
#############################################################
set(NRMK_PLATFORM "STEP3" CACHE STRING "Supported Platform: STEP2 | STEP3")
set_property(CACHE NRMK_PLATFORM PROPERTY STRINGS "STEP2" "STEP3")

#############################################################
# Toolchains                                                #
#############################################################
if (${NRMK_PLATFORM} STREQUAL "STEP3")
	set(ENV{CC} "$ENV{NRMK_TOOLCHAIN_PC3}/x86_64-unknown-linux-gnu-gcc")
	set(ENV{CXX} "$ENV{NRMK_TOOLCHAIN_PC3}/x86_64-unknown-linux-gnu-g++")
else ()
	set(ENV{CC} "$ENV{NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-gcc")
	set(ENV{CXX} "$ENV{NRMK_TOOLCHAIN_PC2}/i686-unknown-linux-gnu-g++")
endif ()


#############################################################
# Cross-compile Toolchain                                   #
#############################################################
set(CMAKE_CROSSCOMPILING true)
set(CMAKE_HOST_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_NAME Linux)
if (${NRMK_PLATFORM} STREQUAL "STEP3")
	set(CMAKE_SYSTEM_PROCESSOR x86_64)
else ()
	set(CMAKE_SYSTEM_PROCESSOR i686)
endif ()
#set(CMAKE_SYSTEM_VERSION 1)

#############################################################
# Project name                                              #
#############################################################
project(SimpleCPPTest)


##############################################################
## CPP Compiler & Linker flags                               #
##############################################################
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_COMPILER_VERSION  6.5)
if (${NRMK_PLATFORM} STREQUAL "STEP3")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -m64 -lrt -fmessage-length=0 -ffast-math -fPIC")
else ()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -m32 -lrt -fmessage-length=0 -ffast-math")
	#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0 -ffast-math -fPIC")
endif ()

# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

# Debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0")



##############################################################
## Includes                                                  #
##############################################################
if (${NRMK_PLATFORM} STREQUAL "STEP3")
	include_directories(
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/json/x86_64/include"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/modbus/x86_64/include"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/grpc/x86_64/include"
			"$ENV{NRMK_BOOST_X64_INC_PATH}"
			"."
	)
else ()
	include_directories(
			"$ENV{NRMK_XENO_PATH}/include"
			"$ENV{NRMK_ECAT_PATH}/include"
			"$ENV{NRMK_CORE_INC_PATH}"
			"$ENV{NRMK_HELPER_INC_PATH}"
			"$ENV{NRMK_HELPER_INC_PATH}/hw"
			"$ENV{NRMK_HELPER_INC_PATH}/hw/nrmk_ecat"
			"$ENV{NRMK_EIGEN_INC_PATH}"
			"$ENV{NRMK_POCO_INC_PATH}"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/json/include"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/modbus/include"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/qpOASES/include"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/boost/include"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/grpc/include"
			"."
	)
endif ()

##############################################################
## Libraries                                                 #
##############################################################
# Define LIBS
if (${NRMK_PLATFORM} STREQUAL "STEP3")
	set( LIBS
			pthread
			z
			jsoncpp
			hiredis
			microhttpd
			curl
			ssl
			crypto
			argtable2
			jsonrpccpp-common
			jsonrpccpp-stub
			jsonrpccpp-server
			jsonrpccpp-client
			modbus
			grpc++
			grpc
			grpc++_reflection
			protobuf
			dl
			)
else ()
	set( LIBS
			xenomai
			native
			rt
			rtdm
			ethercat_rtdm
			pthread
			NRMKFramework
			NRMKCore
			NRMKHelperi686
			NRMKhw_tp
			PocoUtil
			PocoXML
			PocoJSON
			PocoNet
			PocoFoundation
			z
			jsoncpp
			hiredis
			microhttpd
			curl
			ssl
			crypto
			argtable2
			jsonrpccpp-common
			jsonrpccpp-stub
			jsonrpccpp-server
			jsonrpccpp-client
			modbus
			qpOASES
			grpc++
			grpc
			grpc++_reflection
			protobuf
			dl
			)
endif ()

if (${NRMK_PLATFORM} STREQUAL "STEP3")
	link_directories(
			"$ENV{NRMK_LIB_X64_PATH}"
			"$ENV{NRMK_POCO_X64_LIB_PATH}"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/json/x86_64/lib"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/modbus/x86_64/lib"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/boost/x86_64/lib"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/grpc/x86_64/lib"
	)
else ()
	link_directories(
			"$ENV{NRMK_XENO_PATH}/lib"
			"$ENV{NRMK_ECAT_PATH}/lib"
			"$ENV{NRMK_LIB_PATH}"
			"$ENV{NRMK_POCO_LIB_PATH}/i686"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/json/lib"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/modbus/lib"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/qpOASES/lib"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/boost/lib"
			"$ENV{NRMK_INSTALL_PATH}/NRMKFoundation/core/3rdparty/grpc/lib"
	)
endif ()

#############################################################
# Configure Files                                           #
#############################################################


#############################################################
# Build & Install                                           #
#############################################################

# Additional Apps
add_subdirectory(src/MobyClientCpp)

# Installation

