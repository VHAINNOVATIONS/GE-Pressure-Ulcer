#/gevxl/projects/security/pressure_ulcer/assessment/graphcut/CMakeLists.txt

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/share )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/projects/security )
INCLUDE_DIRECTORIES( ${gevxlnew_BINARY_DIR}/share )

#####################################################
SET( pu_graphcut_sources
	block.h				energy.h
	LinkedBlockList.h		LinkedBlockList.cpp
	graph.h				graph.cpp	
	GCoptimization.h		GCoptimization.cpp
	maxflow.cpp
)
ADD_LIBRARY( pu_graphcut ${pu_graphcut_sources} )

# example executables
SUBDIRS( exec )
