#/gevxl/projects/security/pressure_ulcer/assessment/biochemical/CMakeLists.txt

find_path(LABVIEW_INCLUDE_DIR extcode.h PATHS ENV LABVIEW_INCLUDE)
IF (LABVIEW_INCLUDE_DIR)
	MESSAGE("Found Labview include library at ${LABVIEW_INCLUDE_DIR}")
ELSE (LABVIEW_INCLUDE_DIR)
	MESSAGE("Labview include library not found")
ENDIF (LABVIEW_INCLUDE_DIR)
INCLUDE_DIRECTORIES( ${LABVIEW_INCLUDE_DIR} )

#####################################################
SET( pu_assessment_biochemical_sources
   AllDLL.h
   biochemical_sensor.h
   biochemical_sensor.cxx
)

ADD_LIBRARY( pu_assessment_biochemical ${pu_assessment_biochemical_sources} )
#TARGET_LINK_LIBRARIES( pu_assessment_biochemical )

# executables
SUBDIRS( exec )
