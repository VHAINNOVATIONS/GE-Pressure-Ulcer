# gevxl/projects/security/pressure_ulcer/poc/pose_estimate/CMakeLists.txt

INCLUDE_DIRECTORIES( ${VXL_MUL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR})

#set the configure directory
SET( pu_poc_prevention_pose_estimate_source_files 
     patient_pose.h patient_pose.cxx
     saliency.h saliency.cxx
     nearest_neighbor_classifier.h nearest_neighbor_classifier.cxx
     pose_estimate_proc.h pose_estimate_proc.cxx
   ) 

ADD_LIBRARY( pu_poc_prevention_pose_estimate ${pu_poc_prevention_pose_estimate_source_files} )

TARGET_LINK_LIBRARIES(pu_poc_prevention_pose_estimate mvl vgl vgl_algo vcl vbl vnl threading img util mathematics vul)
