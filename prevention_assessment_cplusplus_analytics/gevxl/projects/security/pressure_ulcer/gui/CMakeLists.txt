#/gevxl/projects/security/pressure_ulcer/gui/CMakeLists.txt

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )


IF( WXWIN_FOUND )

SET( pu_gui_sources
      pu_camera_source_frame.h           	 pu_camera_source_frame.cxx
      pu_pixel_picking_client.h       		 pu_pixel_picking_client.cxx      
    )
ADD_LIBRARY( pu_gui ${pu_gui_sources} )

FIND_PACKAGE( PCL )
IF (PCL_FOUND)
    ADD_DEFINITIONS( -DGEVXL_PUGUI_HAS_PCL_SDK )
    INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
	TARGET_LINK_LIBRARIES( pu_gui framework gui gui_wx vid vil util_time framework vcl vul img img_gui ${PCL_LIBRARIES})
ELSEIF (PCL_FOUND)
	TARGET_LINK_LIBRARIES( pu_gui framework gui gui_wx vid vil util_time framework vcl vul img img_gui )
ENDIF(PCL_FOUND)


ENDIF( WXWIN_FOUND )