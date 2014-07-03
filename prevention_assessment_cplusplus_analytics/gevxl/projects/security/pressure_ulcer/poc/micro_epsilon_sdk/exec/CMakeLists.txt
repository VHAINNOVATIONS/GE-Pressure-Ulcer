#projects/security/pressure_ulcer/micro_epsilon_sdk/exec/CMakeLists.txt 

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )

IF (MICRO_EPSILON_IMAGERIPC_FOUND)
    
    ADD_DEFINITIONS( -DGEVXL_HAS_MICRO_EPSILON_IMAGERIPC_SDK )
    
    INCLUDE_DIRECTORIES( ${MICRO_EPSILON_IMAGERIPC_INCLUDE_DIR} )

    SET( pu_micro_epsilon_imageripc_exp01_sources
	   pu_micro_epsilon_imageripc_setup_thread.h	pu_micro_epsilon_imageripc_setup_thread.cxx
           pu_micro_epsilon_imageripc_camera.h		pu_micro_epsilon_imageripc_camera.cxx
           pu_micro_epsilon_imageripc_exp01.cxx
       )
    ADD_EXECUTABLE( pu_micro_epsilon_imageripc_exp01 ${pu_micro_epsilon_imageripc_exp01_sources} )
    
    TARGET_LINK_LIBRARIES( pu_micro_epsilon_imageripc_exp01 util util_time vil threading ${MICRO_EPSILON_IMAGERIPC_LIBRARIES} )
   
    ADD_EXECUTABLE( pu_micro_epsilon_socket_camera_exp01 pu_micro_epsilon_socket_camera.cxx )
    
    TARGET_LINK_LIBRARIES( pu_micro_epsilon_socket_camera_exp01 util util_time vil threading gevxl_boost_system gevxl_boost_thread )
 
ENDIF (MICRO_EPSILON_IMAGERIPC_FOUND)
