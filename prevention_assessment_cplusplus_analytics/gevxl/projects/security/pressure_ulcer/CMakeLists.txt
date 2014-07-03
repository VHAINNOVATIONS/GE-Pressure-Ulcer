#/gevxl/projects/security/pressure_ulcer/CMakeLists.txt

PROJECT (pressure_ulcer)

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/share )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/projects/security )
INCLUDE_DIRECTORIES( ${gevxlnew_BINARY_DIR}/share )

# find openni2 sdk
FIND_PACKAGE( OpenNI2 )
IF(OPENNI2_FOUND)
  SET( GEVXL_VID_HAS_OPENNI2 1 )
  ADD_DEFINITIONS(-DGEVXL_VID_HAS_OPENNI2)

  INCLUDE_DIRECTORIES(${OPENNI2_INCLUDE_DIR})
ENDIF(OPENNI2_FOUND)

# find intel pcsdk
FIND_PACKAGE( PCSDK )
IF(PCSDK_FOUND)
  SET( GEVXL_VID_HAS_PCSDK 1 )
  ADD_DEFINITIONS(-DGEVXL_VID_HAS_PCSDK)

  MESSAGE( STATUS PCSDK_INCLUDE_DIR = ${PCSDK_INCLUDE_DIR} ) 
  MESSAGE( STATUS PCSDK_COMMON_INCLUDE_DIR = ${PCSDK_COMMON_INCLUDE_DIR} ) 
  INCLUDE_DIRECTORIES(${PCSDK_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${PCSDK_COMMON_INCLUDE_DIR})
ENDIF(PCSDK_FOUND)

# find opencv2 sdk
FIND_PACKAGE( OpenCV2 )

# find pcl sdk
FIND_PACKAGE( PCL )
IF(PCL_FOUND)
  SET( GEVXL_HAS_PCL 1 )
  ADD_DEFINITIONS( -DGEVXL_HAS_PCL )
  
  MESSAGE( STATUS PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS} ) 
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
ENDIF(PCL_FOUND)  	

#####################################################
SET( pu_motion_feature_analysis_sources
   # the range imagery based motion feature analysis   
   pu_motion_feature_analysis_proc.h	pu_motion_feature_analysis_proc.cxx
)
ADD_LIBRARY( pu_motion_feature_analysis ${pu_motion_feature_analysis_sources} )
TARGET_LINK_LIBRARIES( pu_motion_feature_analysis vil vnl vbl framework img img_descriptor vid util fgbg )


#####################################################
SET( pu_camera_source_sources
   # the camera source live visualization process
   pu_camera_source_proc.h	       pu_camera_source_proc.cxx   
)
ADD_LIBRARY( pu_camera_source ${pu_camera_source_sources} )
TARGET_LINK_LIBRARIES( pu_camera_source vil vnl vbl framework img vid util )


#####################################################
# sub directories

IF(OPENNI2_FOUND)
  # prevention process directory
  SUBDIRS( prevention )
ENDIF(OPENNI2_FOUND)

IF(PCSDK_FOUND)
  IF(OPENCV2_FOUND)
	IF(PCL_FOUND)
	  # assessment process directory
	  SUBDIRS( assessment )
	ENDIF(PCL_FOUND)
  ENDIF(OPENCV2_FOUND)
ENDIF(PCSDK_FOUND)

# GUI frames and clients
SUBDIRS( gui )

# executables
SUBDIRS( exec )

# proof of concept directory for experimenting with new SDKs
SUBDIRS( poc )

# SWIG for generating Python interface
SUBDIRS( swig )
