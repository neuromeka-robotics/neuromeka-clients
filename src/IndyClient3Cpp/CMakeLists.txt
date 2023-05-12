#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE INDYCLIENT3_SRC "*.cpp" "*.cc")

#############################################################
# Executable                                                #
#############################################################
set(INDYCLIENT3_EXE IndyClient3)
add_executable(${INDYCLIENT3_EXE} ${INDYCLIENT3_SRC} proto/IndyClient3.cpp proto/IndyClient3.h)

set_target_properties(${INDYCLIENT3_EXE} PROPERTIES SUFFIX "")

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${INDYCLIENT3_EXE} ${LIBS})

install (TARGETS ${INDYCLIENT3_EXE} DESTINATION .)