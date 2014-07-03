#projects/security/pressure_ulcer/kinectsdk/gui/CMakeLists.txt 

IF( WXWIN_FOUND )
	
    IF (KinectSDK_FOUND)
	  SET( pu_kinectsdk_gui_source_files
	    pu_kinectsdk_frame.h          pu_kinectsdk_frame.cxx
	    )
	  ADD_LIBRARY( pu_kinectsdk_gui ${pu_kinectsdk_gui_source_files} )
	  TARGET_LINK_LIBRARIES( pu_kinectsdk_gui shape_gui gui_wx framework pu_kinectsdk )
    ENDIF (KinectSDK_FOUND)

ENDIF( WXWIN_FOUND )
