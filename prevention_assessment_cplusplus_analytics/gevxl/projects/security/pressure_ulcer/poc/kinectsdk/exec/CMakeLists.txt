#projects/security/pressure_ulcer/kinectsdk/exec/CMakeLists.txt 

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )

IF( WXWIN_FOUND )

  IF (KinectSDK_FOUND)
    ADD_EXECUTABLE( pu_kinectsdk_wx_exec pu_kinectsdk_wx_exec.cxx )
    TARGET_LINK_LIBRARIES( pu_kinectsdk_wx_exec pu_kinectsdk pu_kinectsdk_gui gui)
  ENDIF (KinectSDK_FOUND)

ENDIF(WXWIN_FOUND)


