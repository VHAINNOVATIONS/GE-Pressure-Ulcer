#/gevxl/projects/security/pressure_ulcer/exec/CMakeLists.txt

INCLUDE( ${GEVXL_CONFIG_DIR}/VIDConfig.cmake )
SET( CONFIG_PATH ${gevxlnew_SOURCE_DIR}/projects/security/pressure_ulcer/config )

IF( WXWIN_FOUND )
  
  #####################################################
  SET( pu_motion_feature_analysis_gui_app_sources pu_motion_feature_analysis_gui_app.cxx )

  IF( USING_IDE )
    SET( pu_motion_feature_analysis_gui_app_sources pu_motion_feature_analysis_gui_app.cxx ${CONFIG_PATH}/pu_motion_feature_analysis.tconf
    )
  ENDIF( USING_IDE )
  
  ADD_EXECUTABLE( pu_motion_feature_analysis_gui_app ${pu_motion_feature_analysis_gui_app_sources} )
  
  TARGET_LINK_LIBRARIES( pu_motion_feature_analysis_gui_app pu_motion_feature_analysis gui gui_wx )

  #####################################################
  SET( pu_camera_source_gui_app_sources pu_camera_source_gui_app.cxx )

  IF( USING_IDE )
    SET( pu_camera_source_gui_app_sources pu_camera_source_gui_app.cxx ${CONFIG_PATH}/pu_camera_source.tconf
    )
  ENDIF( USING_IDE )
  
  ADD_EXECUTABLE( pu_camera_source_gui_app ${pu_camera_source_gui_app_sources} )
  
  TARGET_LINK_LIBRARIES( pu_camera_source_gui_app pu_camera_source pu_gui gui gui_wx )

ENDIF( WXWIN_FOUND )
