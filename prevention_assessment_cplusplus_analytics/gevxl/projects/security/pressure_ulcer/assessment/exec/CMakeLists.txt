#/gevxl/projects/security/pressure_ulcer/assessment/exec/CMakeLists.txt

INCLUDE( ${GEVXL_CONFIG_DIR}/VIDConfig.cmake )
SET( CONFIG_PATH ${gevxlnew_SOURCE_DIR}/projects/security/pressure_ulcer/config )

IF( WXWIN_FOUND )
  
  #####################################################
  SET( pu_assessment_gui_app_sources pu_assessment_gui_app.cxx )

  IF( USING_IDE )
    SET( pu_assessment_gui_app_sources pu_assessment_gui_app.cxx ${CONFIG_PATH}/pu_assessment.tconf
    )
  ENDIF( USING_IDE )
  
  ADD_EXECUTABLE( pu_assessment_gui_app ${pu_assessment_gui_app_sources} )
  
  TARGET_LINK_LIBRARIES( pu_assessment_gui_app pu_assessment_proc pu_assessment_gui gui gui_wx )

ENDIF( WXWIN_FOUND )
