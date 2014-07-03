# gevxl/share/vid/CMakeLists.txt

# Try to locate the various external things that could be used by this
# library
INCLUDE( ${GEVXL_CMAKE_DIR}/FindFFMPEG.cmake )
INCLUDE( ${GEVXL_CMAKE_DIR}/FindVideoSearchCommonLib.cmake )
INCLUDE( ${GEVXL_CMAKE_DIR}/FindThermovision.cmake )

IF( THERMOVISION_FOUND )
  ADD_DEFINITIONS( -DGEVXL_HAS_THERMOVISION )
  INCLUDE_DIRECTORIES( ${THERMOVISION_OCX_LIBRARY} )
ENDIF( THERMOVISION_FOUND )

IF(NOT MINGW)
    # MFC is not available in MinGW.
    # This should be fixed in FindMFC.cmake and then this IF can be removed
    INCLUDE(${CMAKE_ROOT}/Modules/FindMFC.cmake)
ENDIF(NOT MINGW)

IF( FFMPEG_FOUND )
  SET( GEVXL_VID_HAS_FFMPEG 1 )
  OPTION( BUILD_VID_NET_WRITER "Include ffmpeg-based network writer into vid?" "NO" )
  IF( BUILD_VID_NET_WRITER )
    SET( GEVXL_VID_HAS_NET_WRITER 1 )
  ENDIF( BUILD_VID_NET_WRITER )
ELSE( FFMPEG_FOUND )
  # Can't build the net writer without ffmpeg
  SET( GEVXL_VID_HAS_NET_WRITER )
ENDIF( FFMPEG_FOUND )

IF( EXISTS ${VXL_CMAKE_DIR}/NewCMake/FindDirectShow.cmake)
  SET( DIRECTSHOW_FIND_QUIETLY TRUE ) # need this until FindDirectShow.cmake uses FindPackageHandleStandardArgs
  SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${VXL_CMAKE_DIR}/NewCMake )
  FIND_PACKAGE( DirectShow QUIET )
ENDIF()

IF( DIRECTSHOW_FOUND )
  OPTION( USE_DIRECTSHOW_IN_VID "Use DirectShow in vid? Requires vidl2, which is sometimes not available." "YES" )
  IF( USE_DIRECTSHOW_IN_VID )
  # if we have vidl in core (new location), or vidl2 in brl (old location)
  IF( VXL_CORE_VIDEO_FOUND OR VXL_BRL_FOUND )
    IF( DIRECTSHOW_INCLUDE_DIRS )
      # Use this newer include dir variable name from VXL's
      # FindDirectShow.cmake if it is set
      INCLUDE_DIRECTORIES( ${DIRECTSHOW_INCLUDE_DIRS} ) # DIRS - with an S
    ELSE( DIRECTSHOW_INCLUDE_DIRS )
      # Otherwise, use the older variable name
      INCLUDE_DIRECTORIES( ${DIRECTSHOW_INCLUDE_DIR} ) # DIR - without an S
    ENDIF( DIRECTSHOW_INCLUDE_DIRS )
    IF( NOT VXL_CORE_VIDEO_FOUND )
      # must be using vidl2 from brl, so add to include path
      INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR}/bbas )
    ENDIF( NOT VXL_CORE_VIDEO_FOUND )
    SET( GEVXL_VID_HAS_DIRECTSHOW 1 )
  ENDIF( VXL_CORE_VIDEO_FOUND OR VXL_BRL_FOUND )
  ENDIF( USE_DIRECTSHOW_IN_VID )
ENDIF( DIRECTSHOW_FOUND )

FIND_PACKAGE( BaslerPylon )
IF( BASLER_PYLON_FOUND )
  SET( GEVXL_VID_HAS_BASLER_PYLON 1 )
ENDIF( BASLER_PYLON_FOUND )

FIND_PACKAGE( OpenNI )
IF(OpenNI_FOUND)
  SET( GEVXL_VID_HAS_OPENNI 1 )
  ADD_DEFINITIONS(-DGEVXL_VID_HAS_OPENNI)
ENDIF(OpenNI_FOUND)

FIND_PACKAGE( OpenNI2 )
IF(OPENNI2_FOUND)
  SET( GEVXL_VID_HAS_OPENNI2 1 )
  ADD_DEFINITIONS(-DGEVXL_VID_HAS_OPENNI2)

  INCLUDE_DIRECTORIES(${OPENNI2_INCLUDE_DIR})
ENDIF(OPENNI2_FOUND)

FIND_PACKAGE( PCSDK )
IF(PCSDK_FOUND)
  SET( GEVXL_VID_HAS_PCSDK 1 )
  ADD_DEFINITIONS(-DGEVXL_VID_HAS_PCSDK)

  MESSAGE( STATUS PCSDK_INCLUDE_DIR = ${PCSDK_INCLUDE_DIR} ) 
  MESSAGE( STATUS PCSDK_COMMON_INCLUDE_DIR = ${PCSDK_COMMON_INCLUDE_DIR} ) 
  INCLUDE_DIRECTORIES(${PCSDK_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${PCSDK_COMMON_INCLUDE_DIR})
ENDIF(PCSDK_FOUND)

# Save the configuration details for this library
#
CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
                ${CMAKE_CURRENT_BINARY_DIR}/config.h IMMEDIATE )

CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
                ${GEVXL_CONFIG_DIR}/VIDConfig.cmake IMMEDIATE )

SET( vid_framework_source_files

  frame_tag.cxx             frame_tag.h 

  tag_source.cxx            tag_source.h

                            frame_process.h
                            frame_process_sptr.h
                            tagged_frame_process.h
                                        ) 
                                        
ADD_LIBRARY( vid_framework ${vid_framework_source_files} )

TARGET_LINK_LIBRARIES( vid_framework util vil img_util )
                                        
IF( WIN32 )
 IF(MFC_FOUND)
  IF( VIDEOSEARCH_COMMONLIB_FOUND )
   OPTION( JPG_VIDEO_CTC_SUPPORT "Include support for reading jpgvideoctc file formats into vid (creates MFC dependency)?" "NO" )
   IF( JPG_VIDEO_CTC_SUPPORT )
    OPTION( JPG_VIDEO_CTC_USE_VXL_JPG "Use the vxl jpg library for jpeg capabilities?" "NO" )
    IF( JPG_VIDEO_CTC_USE_VXL_JPG )
      ADD_DEFINITIONS( -DGEVXL_USE_VXL_IN_JPGVIDEOCTC )
    ENDIF( JPG_VIDEO_CTC_USE_VXL_JPG )
    ADD_DEFINITIONS(-D_AFXDLL)
    SET(CMAKE_MFC_FLAG 2) 
    INCLUDE_DIRECTORIES(${VIDEOSEARCH_COMMONLIB_INCLUDE_DIR})
    ADD_DEFINITIONS( -DGEVXL_HAS_JPGVIDEOCTC )
   ENDIF( JPG_VIDEO_CTC_SUPPORT )
  ENDIF( VIDEOSEARCH_COMMONLIB_FOUND )
 ENDIF(MFC_FOUND)
ENDIF(WIN32)

INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} ) # probably not required here?
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )

IF( WIN32 )
  IF( NOT CYGWIN )
    ADD_DEFINITIONS(-DHAS_WINMM)
  ENDIF( NOT CYGWIN )
ENDIF( WIN32 )

SET( vid_source_files

  generic_frame_process.cxx             generic_frame_process.h
                                        source_process.h
  image_list_frame_process.txx          image_list_frame_process.h
  external_frame_process.txx            external_frame_process.h
  shared_memory_frame_process.txx       shared_memory_frame_process.h
                                        shared_memory_frame_process_data.h
  matrox_frame_process.cxx              matrox_frame_process.h

  pixelfly_frame_process.txx
  pixelfly_frame_process.cxx            pixelfly_frame_process.h

  ffmpeg_frame_process.cxx              ffmpeg_frame_process.h

  videoarchive_frame_process.cxx        videoarchive_frame_process.h
  videoarchive_writer_process.cxx       videoarchive_writer_process.h

  # see below about directshow_frame_process
  # directshow_frame_process.cxx          directshow_frame_process.h
  blank_frame_process.txx               blank_frame_process.h

  convert_image_frame_process.txx       convert_image_frame_process.h
  buffered_frame_process.txx            buffered_frame_process.h
  delayed_frame_process.txx             delayed_frame_process.h
  smoothed_frame_process.txx            smoothed_frame_process.h
  saturation_mask_process.txx           saturation_mask_process.h
  integral_image_proc.cxx               integral_image_proc.h 
  smoothed_from_integral_image_proc.h

  generic_process_chain.cxx             generic_process_chain.h

  generic_writer_process.cxx            generic_writer_process.h
  #image_list_writer_process.txx         image_list_writer_process.h
  
  threshold_proc.cxx                    threshold_proc.h 
  
  gray_frame_process.cxx                gray_frame_process.h
  colorspace_conversion_frame_process.cxx  colorspace_conversion_frame_process.h
  
  vertical_flip_frame_process.cxx       vertical_flip_frame_process.h      
  horizontal_flip_frame_process.cxx     horizontal_flip_frame_process.h      
  translate_frame_process.cxx           translate_frame_process.h    
  code_stamp_frame_tag_process.cxx      code_stamp_frame_tag_process.h 
  grid_combine_frame_process.cxx        grid_combine_frame_process.h
  scaled_frame_process.txx              scaled_frame_process.h 
  
  distance_transform_frame_process.txx     distance_transform_frame_process.h 

  optical_flow_process.txx              optical_flow_process.h
  spatio_temporal_sep_filter_process.cxx   spatio_temporal_sep_filter_process.h

  splitter_frame_process.txx            splitter_frame_process.h

  io/reader.cxx                         io/reader.h
                                        io/writer.h
  io/ffmpeg_writer.cxx                  io/ffmpeg_writer.h
  io/ffmpeg_writer_stub.txx
  #io/ffmpeg_writer_v2.txx
  #io/ffmpeg_writer_v3.txx
  #io/ffmpeg_writer_v4.txx
  #io/ffmpeg_writer_v5.txx
  
  io/ffmpeg_reader.cxx                  io/ffmpeg_reader.h
  io/ffmpeg_reader_stub.txx
  #io/ffmpeg_reader_v2.txx
  #io/ffmpeg_reader_v3.txx
  #io/ffmpeg_reader_v4.txx
  #io/ffmpeg_reader_v5.txx
  
  io/async_writer_wrapper.txx           io/async_writer_wrapper.h
  #io/image_list_writer.txx              io/image_list_writer.h
  #io/image_list_reader.txx              io/image_list_reader.h
  
  ffmpeg_writer_process.cxx             ffmpeg_writer_process.h 
  io/ffmpeg_initialize.cxx              io/ffmpeg_initialize.h
  io/ffmpeg_asynch_writer.cxx           io/ffmpeg_asynch_writer.h
  io/ffmpeg_asynch_writer_process.cxx   io/ffmpeg_asynch_writer_process.h

  upsample_process.cxx upsample_process.h 
  downsample_process.cxx downsample_process.h 

  basler_frame_process.cxx              basler_frame_process.h
  basler_gige_frame_process.cxx         basler_gige_frame_process.h
  basler_genicam.cxx                    basler_genicam.h

  axis_ip_frame_process.cxx             axis_ip_frame_process.h
  
  kinect_frame_process.txx              kinect_frame_process.h
  rgbd_ffmpeg_frame_process.cxx         rgbd_ffmpeg_frame_process.h
  rgbd_ffmpeg_writer_process.cxx        rgbd_ffmpeg_writer_process.h
  rgbd_frame_process.txx                rgbd_frame_process.h
  rgbd_writer_process.txx               rgbd_writer_process.h

  jpgvideoctc_frame_process.cxx         jpgvideoctc_frame_process.h
  disguise_frame_process.cxx            disguise_frame_process.h
  bluescreen_detection_frame_process.cxx bluescreen_detection_frame_process.h

  avi_frame_process.cxx avi_frame_process.h
  # avi_writer_process uses vidl version 1, now named vidl1 instead of vidl
  # avi_writer_process.cxx avi_writer_process.h
  
  shared_memory_boost_frame_process.txx shared_memory_boost_frame_process.h
  shared_memory_boost_writer_process.txx shared_memory_boost_writer_process.h
  shared_memory_boost_frame_data.txx shared_memory_boost_frame_data.h   

)

# directshow_frame_process uses "vidl2" from VXL.  In VXL, on 3/24/09,
# "vidl2" was moved to core and renamed to "vidl" (and "vidl" was
# renamed to "vidl1").  If VXL_CORE_VIDEO_FOUND is set, that indicates
# we are using a version of VXL from after the renaming.
# directshow_frame_process_deprecated.cxx is our original file and
# uses vidl2, while directshow_frame_process.cxx is updated to use
# vidl.  After some period of time to allow everyone to update VXL, we
# can remove the file directshow_frame_process_deprecated.cxx, and
# go back to directly using directshow_frame_process.cxx above.
# When this happens, vid will no longer depend on brl, so this file
# can be simplified whever brl is mentioned.  -Fred
#
IF( VXL_CORE_VIDEO_FOUND )
  SET( vid_source_files ${vid_source_files}
    directshow_frame_process.cxx directshow_frame_process.h
    )
ELSE( VXL_CORE_VIDEO_FOUND )
  SET( vid_source_files ${vid_source_files}
    directshow_frame_process_deprecated.cxx directshow_frame_process.h
    )
ENDIF( VXL_CORE_VIDEO_FOUND )

IF( BUILD_GEVXL_PYTHON_LIBRARY )
  SET ( vid_source_files ${vid_source_files} vid_python_module.h 
        python_script_frame_process.cxx python_script_frame_process.h
        python_script_generic_process_chain.cxx python_script_generic_process_chain.h
  ) 
ENDIF( BUILD_GEVXL_PYTHON_LIBRARY )

IF (BASLER_PYLON_FOUND)
   INCLUDE_DIRECTORIES(${BASLER_PYLON_INCLUDE_DIRS})   
ENDIF(BASLER_PYLON_FOUND)

IF(OpenNI_FOUND)
   SET( vid_source_files ${vid_source_files}
     openni_camera.cxx                     openni_camera.h
   )
   INCLUDE_DIRECTORIES(${OpenNI_INCLUDE_DIR})        
ENDIF(OpenNI_FOUND)

IF(OPENNI2_FOUND)
   SET( vid_source_files ${vid_source_files}
     openni2_camera.cxx                     openni2_camera.h
     openni2_frame_process.cxx		    openni2_frame_process.h
     openni2_writer_process.cxx		    openni2_writer_process.h
   )   
ENDIF(OPENNI2_FOUND)

IF(PCSDK_FOUND)
   SET( vid_source_files ${vid_source_files}
     pxc_camera.cxx                 pxc_camera.h
     pxc_frame_process.cxx		    pxc_frame_process.h
   )   
ENDIF(PCSDK_FOUND)

OPTION( BUILD_MICRO_EPSILON "Build the Micro-Epsilon Camera Wrapper?" NO )
IF(BUILD_MICRO_EPSILON)
   SET( GEVXL_VID_HAS_MICRO_EPSILON_SOCKET 1 )
   ADD_DEFINITIONS(-DGEVXL_VID_HAS_MICRO_EPSILON_SOCKET)
   
   SET( vid_source_files ${vid_source_files}
     micro_epsilon_socket_camera.cxx                     micro_epsilon_socket_camera.h
     micro_epsilon_socket_frame_process.cxx		 micro_epsilon_socket_frame_process.h
   )   
ENDIF(BUILD_MICRO_EPSILON)

OPTION( BUILD_BAYSPEC "Build the Bayspec Camera Wrapper?" NO )
IF(BUILD_BAYSPEC)
   SET( GEVXL_VID_HAS_BAYSPEC_SOCKET 1 )
   ADD_DEFINITIONS(-DGEVXL_VID_HAS_BAYSPEC_SOCKET)
   
   SET( vid_source_files ${vid_source_files}
     bayspec_socket_camera.cxx                     	bayspec_socket_camera.h
     bayspec_socket_frame_process.cxx		 	bayspec_socket_frame_process.h
   )   
ENDIF(BUILD_BAYSPEC)


IF( GEVXL_VID_HAS_FFMPEG )
  #SET( vid_source_files ${vid_source_files}
  #)
  INCLUDE_DIRECTORIES( ${FFMPEG_INCLUDE_DIR} )

  IF( GEVXL_VID_HAS_NET_WRITER )
    SET( vid_source_files ${vid_source_files}
      io/ffmpeg_net_protocol.cxx
    )
  ENDIF( GEVXL_VID_HAS_NET_WRITER )
ENDIF( GEVXL_VID_HAS_FFMPEG )

IF( THERMOVISION_FOUND )
  SET(vid_source_files ${vid_source_files}
    thermovision_frame_process.cxx        thermovision_frame_process.h )
ENDIF( THERMOVISION_FOUND )

# Template instantiations
AUX_SOURCE_DIRECTORY( Templates vid_source_files )
AUX_SOURCE_DIRECTORY( io/Templates vid_source_files )

ADD_LIBRARY( vid ${vid_source_files} )

IF (BASLER_PYLON_FOUND)      
  TARGET_LINK_LIBRARIES( vid ${BASLER_PYLON_LIBRARIES})  
ENDIF(BASLER_PYLON_FOUND)

###############################################
IF( BUILD_OPENCV_BRIDGE AND OPENCV2_FOUND )
   ADD_DEFINITIONS(-DGEVXL_USE_OPENCV)
   MESSAGE( STATUS "vid uses opencv_bridge for video capture" )
   TARGET_LINK_LIBRARIES( vid opencv_bridge ${OPENCV2_LIBS} )	
ENDIF( BUILD_OPENCV_BRIDGE AND OPENCV2_FOUND )

IF(OpenNI_FOUND)      
  TARGET_LINK_LIBRARIES(vid ${OpenNI_LIBS})  
ENDIF(OpenNI_FOUND)

IF(OPENNI2_FOUND)      
  TARGET_LINK_LIBRARIES(vid ${OPENNI2_LIBRARY})  
ENDIF(OPENNI2_FOUND)

IF(PCSDK_FOUND)      
  TARGET_LINK_LIBRARIES(vid ${PCSDK_LIBS})  
ENDIF(PCSDK_FOUND)

IF(BUILD_MICRO_EPSILON)
  TARGET_LINK_LIBRARIES(vid gevxl_boost_system gevxl_boost_thread)
ENDIF(BUILD_MICRO_EPSILON)

IF(BUILD_BAYSPEC)
  TARGET_LINK_LIBRARIES(vid gevxl_boost_system gevxl_boost_thread)
ENDIF(BUILD_BAYSPEC)


IF( WIN32 )
  IF( NOT CYGWIN )
    TARGET_LINK_LIBRARIES( vid winmm Vfw32)
  ENDIF( NOT CYGWIN )
ENDIF( WIN32 )

# Linking for the shm_open() function 
IF( ${CMAKE_SYSTEM_NAME} MATCHES "Linux" )
    TARGET_LINK_LIBRARIES( vid rt )
ENDIF( ${CMAKE_SYSTEM_NAME} MATCHES "Linux") 


###############################################
IF( GEVXL_VID_HAS_FFMPEG )
  TARGET_LINK_LIBRARIES( vid ${FFMPEG_LIBRARIES} )  
  
  IF( GEVXL_VID_HAS_NET_WRITER )
    TARGET_LINK_LIBRARIES( vid netcom )
  ENDIF( GEVXL_VID_HAS_NET_WRITER )

  ADD_LIBRARY( vid_io_ffmpeg_support
    io/ffmpeg_support.c
  )
  TARGET_LINK_LIBRARIES( vid vid_io_ffmpeg_support )  

ENDIF( GEVXL_VID_HAS_FFMPEG )


###############################################
IF( GEVXL_VID_HAS_DIRECTSHOW )
  IF( VXL_CORE_VIDEO_FOUND )
    # using the newer vidl2 which is in core and called vidl
    TARGET_LINK_LIBRARIES( vid vidl )
  ELSE( VXL_CORE_VIDEO_FOUND )
    IF( VXL_BRL_FOUND )
      # using the older vidl2 which is in brl
      TARGET_LINK_LIBRARIES( vid vidl2 )
    ENDIF( VXL_BRL_FOUND )
  ENDIF( VXL_CORE_VIDEO_FOUND )
ENDIF( GEVXL_VID_HAS_DIRECTSHOW )


###############################################
IF( JPG_VIDEO_CTC_SUPPORT )
  IF( JPG_VIDEO_CTC_USE_VXL_JPG )
    TARGET_LINK_LIBRARIES(vid ${VIDEOSEARCH_COMMONLIB_LIBRARIES} jpeg psapi threading)    
  ELSE( JPG_VIDEO_CTC_USE_VXL_JPG )
    TARGET_LINK_LIBRARIES(vid ${VIDEOSEARCH_COMMONLIB_LIBRARIES} ${VIDEOSEARCH_COMMONLIB_JPG_LIBRARIES} psapi threading)    
  ENDIF( JPG_VIDEO_CTC_USE_VXL_JPG )
ENDIF( JPG_VIDEO_CTC_SUPPORT )


###############################################
IF( THERMOVISION_FOUND )
   TARGET_LINK_LIBRARIES( vid thermovision )
ENDIF( THERMOVISION_FOUND )

###############################################
# Note: we use stub to interface with Matrox MIL so we must include matrox.lib in all cases.
TARGET_LINK_LIBRARIES( vid vid_framework vidarchive cmu_1394 util util_time vil vil_algo matrox pixelfly img img_util img_spatio_temporal_filter) # opencv_bridge)

IF( WIN32 )
IF( NOT CYGWIN )

#SET( vid_clr_source_files
#     AssemblyInfo.cpp
#     ffmpeg_reader_clr.cpp
#     ffmpeg_reader_clr.h
#)
#
#SET_SOURCE_FILES_PROPERTIES( ${vid_clr_source_files} PROPERTIES COMPILE_FLAGS "/clr" )

#ADD_LIBRARY( vid_clr SHARED ${vid_clr_source_files} )
#
#TARGET_LINK_LIBRARIES( vid_clr vid )

ENDIF( NOT CYGWIN )
ENDIF( WIN32 )
