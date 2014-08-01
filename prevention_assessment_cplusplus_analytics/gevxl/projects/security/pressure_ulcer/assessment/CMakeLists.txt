#/gevxl/projects/security/pressure_ulcer/assessment/CMakeLists.txt

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/share )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/projects/security )
INCLUDE_DIRECTORIES( ${gevxlnew_BINARY_DIR}/share )

#####################################################
SET( pu_assessment_proc_sources

   # the 3D reconstruction process from point cloud data
   pu_3d_recon_proc.h		   	pu_3d_recon_proc.cxx
   
   # the thermal imagery analysis process
   pu_thermal_analysis_proc.h          	pu_thermal_analysis_proc.cxx
   
   # the color image segmentation process based on color and texture (from the camera's RGB channels)
   # as well as 3D curvature and surface normal (from the RGB to depth sensor calibration)
   # need to add the tissue type labeling process based on the color image segmentation results
   pu_tissue_analysis_proc.h	       	pu_tissue_analysis_proc.cxx

   # the pressure ulcer assessment system's main process
   pu_assessment_system_proc.h		pu_assessment_system_proc.cxx   
)

ADD_LIBRARY( pu_assessment_proc ${pu_assessment_proc_sources} )
TARGET_LINK_LIBRARIES( pu_assessment_proc vil vnl vbl vgl framework img vid util shape opencv_bridge ${PCL_LIBRARIES} )

# database writer and reader
SUBDIRS( database )

# GUI frames and clients
SUBDIRS( gui )

# executables
SUBDIRS( exec )

# graphcut library
SUBDIRS( graphcut )

# biochemical sensor
SUBDIRS( biochemical )
