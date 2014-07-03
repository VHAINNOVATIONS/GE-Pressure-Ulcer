# gevxl/geom/gui/CMakeLists.txt
#INCLUDE( ${GEVXL_CMAKE_DIR}/FindWxWin.cmake )
INCLUDE_DIRECTORIES( ${VXL_MUL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR})
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR})
          
IF( WXWIN_FOUND )

    #INCLUDE_DIRECTORIES( ${WXWIN_INCLUDE_DIR} )
    #LINK_DIRECTORIES( ${WXWIN_LINK_DIRECTORIES} )
    #ADD_DEFINITIONS( ${WXWIN_DEFINITIONS} )
  ##################
  # VIDEOIQ
  ##################

  FIND_PACKAGE(GEVIDIQ)
  IF(NOT GEVIDIQ_FOUND)
    FIND_PACKAGE(GEVIDIQ QUIET)
  ENDIF(NOT GEVIDIQ_FOUND)
  IF(GEVIDIQ_FOUND)
    INCLUDE(${GEVIDIQ_DIR}/UseGEVIDIQ.cmake) 
  ELSE(GEVIDIQ_FOUND)
  #  MESSAGE("GEVIDIQ_DIR should be set to the GEVIDIQ build directory.")
  ENDIF(GEVIDIQ_FOUND)


  SET( sec_va_gui_sources
  	 va_image_frame.h va_image_frame.cxx	
         va_image_client.h va_image_client.cxx
     )

  ADD_LIBRARY( sec_va_gui ${sec_va_gui_sources} )

  TARGET_LINK_LIBRARIES( sec_va_gui framework gui gui_wx vid
                vil util_time framework mvl vgl vnl vcl sec_va
                vul img img_gui)



IF(BUILD_DETECTOR_EXECS )

   


                          

ENDIF( BUILD_DETECTOR_EXECS )
                                           

ENDIF( WXWIN_FOUND )


