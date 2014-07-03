#projects/security/pressure_ulcer/pcl_sdk/exec/CMakeLists.txt 

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )

#set(PCL_DIR "E:/pcl_msvc2008_64bit_build_with_3rdparty_installer/pcl-master_install")
#find_package(PCL 1.7 REQUIRED COMPONENTS common io)
#find_package(PCL COMPONENTS common io)

#message("cmake gets here now.")
#message(PCL_INCLUDE_DIRS = ${PCL_INCLUDE_DIRS})
#include_directories(${PCL_INCLUDE_DIRS})

#message(PCL_LIBRARY_DIRS = ${PCL_LIBRARY_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})

#add_definitions(${PCL_DEFINITIONS})

#add_executable(pu_pcl_sdk_exp01 pu_pcl_sdk_exp01.cxx)

#target_link_libraries(pu_pcl_sdk_exp01 ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

#####################################################################################
IF (PCL_FOUND)
    
    ADD_DEFINITIONS( -DGEVXL_DEPTH_HAS_PCL_SDK )
        
    INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
    
    #MESSAGE(STATUS BOOST_LIBRARIES = ${BOOST_LIBRARIES})
    
    ADD_EXECUTABLE( pu_pcl_sdk_exp01 pu_pcl_sdk_exp01.cxx )
    #working	TARGET_LINK_LIBRARIES( pu_pcl_sdk_exp01 ${PCL_LIBRARIES} ${BOOST_LIBRARIES} )
    #working	TARGET_LINK_LIBRARIES( pu_pcl_sdk_exp01 ${PCL_LIBRARIES} )
    TARGET_LINK_LIBRARIES( pu_pcl_sdk_exp01 ${PCL_LIBRARIES} vid )
    
    ADD_EXECUTABLE( pu_pcl_sdk_exp02 pu_pcl_sdk_exp02.cxx )
    TARGET_LINK_LIBRARIES( pu_pcl_sdk_exp02 ${PCL_LIBRARIES}  vid )
    
    ADD_EXECUTABLE( pu_pcl_sdk_exp_recon pu_pcl_sdk_exp_recon.cxx )
    TARGET_LINK_LIBRARIES( pu_pcl_sdk_exp_recon ${PCL_LIBRARIES}  vid )

    ADD_EXECUTABLE( pu_pcl_sdk_exp_plane pu_pcl_sdk_exp_plane.cxx )
    TARGET_LINK_LIBRARIES( pu_pcl_sdk_exp_plane ${PCL_LIBRARIES}  vid )

    ADD_EXECUTABLE( pu_pcl_sdk_exp_icp pu_pcl_sdk_exp_icp.cxx )
    TARGET_LINK_LIBRARIES( pu_pcl_sdk_exp_icp ${PCL_LIBRARIES}  vid )

    
ENDIF (PCL_FOUND)



