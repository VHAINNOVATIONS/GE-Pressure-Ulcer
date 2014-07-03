#projects/security/pressure_ulcer/poc/bayspec_sdk/exec/CMakeLists.txt 

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )

SET( pu_bayspec_socket_client_exp01_sources
	pu_bayspec_socket_client_exp01.cxx
   )
ADD_EXECUTABLE( pu_bayspec_socket_client_exp01 ${pu_bayspec_socket_client_exp01_sources} )
    
TARGET_LINK_LIBRARIES( pu_bayspec_socket_client_exp01 util util_time vil threading gevxl_boost_system gevxl_boost_thread )

#####################################################333
#FIND_PACKAGE( VAi2Bayspec )
#
#IF (VAI2_BAYSPEC_FOUND)
#  
#  ADD_DEFINITIONS( -DGEVXL_HAS_VAI2_BAYSPEC_SDK )
#
#  INCLUDE_DIRECTORIES( ${VAI2_BAYSPEC_INCLUDE} )
#
#  SET( pu_bayspec_64bit_dll_exp02_sources
#	pu_bayspec_64bit_dll_exp02.cxx
#   )
#  ADD_EXECUTABLE( pu_bayspec_64bit_dll_exp02 ${pu_bayspec_64bit_dll_exp02_sources} )
#
#  TARGET_LINK_LIBRARIES( pu_bayspec_64bit_dll_exp02 vil vid img ${VAI2_BAYSPEC_GIGE_LIBRARY} ${VAI2_BAYSPEC_PROCESSOR_LIBRARY} )
#
#ENDIF (VAI2_BAYSPEC_FOUND)
