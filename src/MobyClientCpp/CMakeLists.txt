#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE MOBYCLIENT_SRC "*.cpp" "*.cc")

#############################################################
# Executable                                                #
#############################################################
set(MOBYCLIENT_EXE MobyClient)
add_executable(${MOBYCLIENT_EXE} ${MOBYCLIENT_SRC} proto/MobyClient.cpp proto/MobyClient.h proto/EcatClient.cpp proto/EcatClient.h)

set_target_properties(${MOBYCLIENT_EXE} PROPERTIES SUFFIX "")

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${MOBYCLIENT_EXE} ${LIBS})

install (TARGETS ${MOBYCLIENT_EXE} DESTINATION .)