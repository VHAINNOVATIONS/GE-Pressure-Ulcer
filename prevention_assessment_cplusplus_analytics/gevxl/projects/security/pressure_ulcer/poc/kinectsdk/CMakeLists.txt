#projects/security/pressure_ulcer/kinectsdk/CMakeLists.txt 

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )

FIND_PACKAGE( KinectSDK )

IF (KinectSDK_FOUND)
  
  ADD_DEFINITIONS( -DGEVXL_DEPTH_HAS_KinectSDK )

  INCLUDE_DIRECTORIES( "C:/Program Files/Microsoft SDKs/Windows/v6.0A/Include" )
  INCLUDE_DIRECTORIES( ${KinectSDK_INCLUDE_DIR} )
  INCLUDE_DIRECTORIES( ${KinectDeveloperToolkitSDK_INCLUDE_DIR} )
        
  SET( pu_kinectsdk_source_files
      
      pu_kinectsdk_sensor.h           		 pu_kinectsdk_sensor.cxx
      pu_kinectsdk_control_process_chain.h       pu_kinectsdk_control_process_chain.cxx
      
      #pu_kinectsdk_skeleton_process.h            pu_kinectsdk_skeleton_process.cxx
      #pu_kinectsdk_face_process.h     		 pu_kinectsdk_face_process.cxx
      pu_kinectsdk_kinfu_process.h     		 pu_kinectsdk_kinfu_process.cxx
     )
 
    
    ADD_LIBRARY( pu_kinectsdk ${pu_kinectsdk_source_files} )
    TARGET_LINK_LIBRARIES( pu_kinectsdk vcl vbl vnl img util util_time ${KinectSDK_LIBS} ${KinectDeveloperToolkitSDK_LIBS} )

ENDIF (KinectSDK_FOUND)

#sub-folders
SUBDIRS( gui )
SUBDIRS( exec )
