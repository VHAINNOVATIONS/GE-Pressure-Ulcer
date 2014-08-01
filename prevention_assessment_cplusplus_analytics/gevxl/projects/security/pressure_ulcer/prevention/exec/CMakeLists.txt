#/gevxl/projects/security/pressure_ulcer/prevention/exec/CMakeLists.txt

INCLUDE( ${GEVXL_CONFIG_DIR}/VIDConfig.cmake )
SET( CONFIG_PATH ${gevxlnew_SOURCE_DIR}/projects/security/pressure_ulcer/config )

IF( WXWIN_FOUND )
  
  #####################################################
  SET( pu_prevention_gui_app_sources pu_prevention_gui_app.cxx )

  IF( USING_IDE )
    SET( pu_prevention_gui_app_sources pu_prevention_gui_app.cxx ${CONFIG_PATH}/pu_prevention.tconf
    )
  ENDIF( USING_IDE )
  
  ADD_EXECUTABLE( pu_prevention_gui_app ${pu_prevention_gui_app_sources} )
  
  TARGET_LINK_LIBRARIES( pu_prevention_gui_app pu_prevention_proc pu_prevention_gui gui gui_wx )

  #####################################################
  SET( pu_prevention_pose_estimate_gui_app_sources pu_prevention_pose_estimate_gui_app.cxx )

  IF( USING_IDE )
    SET( pu_prevention_pose_estimate_gui_app_sources pu_prevention_pose_estimate_gui_app.cxx ${CONFIG_PATH}/pu_prevention_pose_estimate.tconf
    )
  ENDIF( USING_IDE )
  
  ADD_EXECUTABLE( pu_prevention_pose_estimate_gui_app ${pu_prevention_pose_estimate_gui_app_sources} )
  
  TARGET_LINK_LIBRARIES( pu_prevention_pose_estimate_gui_app pu_prevention_pose_estimate_proc gui gui_wx )

ENDIF( WXWIN_FOUND )
