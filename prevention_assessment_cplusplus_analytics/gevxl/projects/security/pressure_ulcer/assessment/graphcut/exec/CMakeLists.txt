#/gevxl/projects/security/pressure_ulcer/assessment/graphcut/exec/CMakeLists.txt

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/share )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/projects/security )
INCLUDE_DIRECTORIES( ${gevxlnew_BINARY_DIR}/share )

#####################################################
ADD_EXECUTABLE( pu_graphcut_example_exec example.cpp )
TARGET_LINK_LIBRARIES( pu_graphcut_example_exec pu_graphcut )



