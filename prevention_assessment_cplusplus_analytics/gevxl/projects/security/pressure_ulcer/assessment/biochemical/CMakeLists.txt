#/gevxl/projects/security/pressure_ulcer/assessment/biochemical/CMakeLists.txt

FIND_PACKAGE( VAi2ChemicalSensing )

IF (VAI2_CHEMICAL_SENSING_FOUND)
  
  ADD_DEFINITIONS( -DGEVXL_HAS_VAI2_CHEMICAL_SENSING_SDK )

  INCLUDE_DIRECTORIES( ${VAI2_LABVIEW_RUNTIME_INCLUDE} )
  INCLUDE_DIRECTORIES( ${VAI2_CHEMICAL_SENSING_INCLUDE} )

  #####################################################
  SET( pu_assessment_biochemical_sources
     biochemical_sensor.h
     biochemical_sensor.cxx
  )

  ADD_LIBRARY( pu_assessment_biochemical ${pu_assessment_biochemical_sources} )
  TARGET_LINK_LIBRARIES( pu_assessment_biochemical ${VAI2_CHEMICAL_SENSING_LIBRARY} )

  # executables
  SUBDIRS( exec )

ENDIF (VAI2_CHEMICAL_SENSING_FOUND)
