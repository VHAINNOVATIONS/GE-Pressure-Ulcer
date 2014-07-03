# gevxl/geom/gui/CMakeLists.txt
#INCLUDE( ${GEVXL_CMAKE_DIR}/FindWxWin.cmake )

IF( WXWIN_FOUND )

    #INCLUDE_DIRECTORIES( ${WXWIN_INCLUDE_DIR} )
    #LINK_DIRECTORIES( ${WXWIN_LINK_DIRECTORIES} )
    #ADD_DEFINITIONS( ${WXWIN_DEFINITIONS} )
    INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )


    # a basic exectuable the executes a process on a video in batch mode 
    ADD_EXECUTABLE( sec_va_image_exec 
                    sec_va_image_exec.cxx )

    TARGET_LINK_LIBRARIES( sec_va_image_exec sec_va sec_va_gui 
                           vid vul ) 

    # a basic exectuable the executes a process on a video in batch mode 
    ADD_EXECUTABLE( sec_va_video_exec 
                    sec_va_video_exec.cxx )

    TARGET_LINK_LIBRARIES( sec_va_video_exec sec_va 
                           vid vul ) 
			   

ENDIF( WXWIN_FOUND )

