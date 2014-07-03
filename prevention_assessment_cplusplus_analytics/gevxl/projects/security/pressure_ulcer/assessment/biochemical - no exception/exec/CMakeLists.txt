#/gevxl/projects/security/pressure_ulcer/assessment/biochemical/exec/CMakeLists.txt

find_library(BIOCHEMICAL_SENSOR_LIB AllDLL.lib PATHS C:/tmp/AllDLL ENV BIOCHEMICAL_SENSOR_LIB)
IF (BIOCHEMICAL_SENSOR_LIB)
	MESSAGE("Found biochemical sensor lib at ${BIOCHEMICAL_SENSOR_LIB}")
ELSE (BIOCHEMICAL_SENSOR_LIB)
	MESSAGE("Biochemical sensor lib not found")
ENDIF (BIOCHEMICAL_SENSOR_LIB)

#####################################################
ADD_EXECUTABLE( pu_biochemical_example_exec biochemical_example.cxx )
TARGET_LINK_LIBRARIES( pu_biochemical_example_exec pu_assessment_biochemical ${BIOCHEMICAL_SENSOR_LIB} )
