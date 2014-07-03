#/gevxl/projects/security/pressure_ulcer/assessment/biochemical/exec/CMakeLists.txt

#####################################################
ADD_EXECUTABLE( pu_biochemical_example_exec biochemical_example.cxx )
TARGET_LINK_LIBRARIES( pu_biochemical_example_exec pu_assessment_biochemical ${VAI2_CHEMICAL_SENSING_LIBRARY} )
