#/gevxl/projects/security/pressure_ulcer/poc/wxwidget_vtk/CMakeLists.txt

INCLUDE( ${GEVXL_CONFIG_DIR}/VIDConfig.cmake )
SET( CONFIG_PATH ${gevxlnew_SOURCE_DIR}/projects/security/pressure_ulcer/config )

IF( WXWIN_FOUND )
  IF (VTK_FOUND) 
 
    SET( pu_rgbd_camera_vtk_gui_app_sources pu_rgbd_camera_vtk_gui_app.cxx )

    IF( USING_IDE )
      SET( pu_rgbd_camera_vtk_gui_app_sources pu_rgbd_camera_vtk_gui_app.cxx ${CONFIG_PATH}/pu_rgbd_camera_vtk.tconf
      )
    ENDIF( USING_IDE )
  
    ADD_EXECUTABLE( pu_rgbd_camera_vtk_gui_app ${pu_rgbd_camera_vtk_gui_app_sources} )
  
    TARGET_LINK_LIBRARIES( pu_rgbd_camera_vtk_gui_app gui gui_wx vtkRendering )

  ENDIF(VTK_FOUND)
ENDIF( WXWIN_FOUND )
