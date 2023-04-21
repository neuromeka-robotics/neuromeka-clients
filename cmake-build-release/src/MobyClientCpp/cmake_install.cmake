# Install script for directory: /cygdrive/c/NRMK-Local-Repository/ethercat_moby_framework/src/neuromeka-clients/src/MobyClientCpp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "true")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./MobyClient" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./MobyClient")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./MobyClient"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE EXECUTABLE FILES "/cygdrive/c/NRMK-Local-Repository/ethercat_moby_framework/src/neuromeka-clients/cmake-build-release/src/MobyClientCpp/MobyClient")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./MobyClient" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./MobyClient")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./MobyClient"
         OLD_RPATH "C:/Program Files (x86)/neuromeka/NRMKFoundation/lib64:C:/Program Files (x86)/neuromeka/NRMKFoundation/core/3rdparty/Poco/x86_64/lib:C:/Program Files (x86)/neuromeka/NRMKFoundation/core/3rdparty/json/x86_64/lib:C:/Program Files (x86)/neuromeka/NRMKFoundation/core/3rdparty/modbus/x86_64/lib:C:/Program Files (x86)/neuromeka/NRMKFoundation/core/3rdparty/boost/x86_64/lib:C:/Program Files (x86)/neuromeka/NRMKFoundation/core/3rdparty/grpc/x86_64/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "C:/Program Files (x86)/neuromeka/NRMKPlatformPC3/Drivers/x86_64-unknown-linux-gnu/bin/x86_64-unknown-linux-gnu-strip.exe" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/./MobyClient")
    endif()
  endif()
endif()

