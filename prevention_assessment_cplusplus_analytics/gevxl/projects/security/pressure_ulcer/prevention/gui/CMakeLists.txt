#/gevxl/projects/security/pressure_ulcer/prevention/gui/CMakeLists.txt

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )


IF( WXWIN_FOUND )

SET( pu_prevention_gui_sources
      
      # GUI frame for the prevention system
      pu_prevention_frame.h      pu_prevention_frame.cxx

      # GUI client for pose estimate
      pu_pose_estimate_client.h     pu_pose_estimate_client.cxx    
    )
ADD_LIBRARY( pu_prevention_gui ${pu_prevention_gui_sources} )

TARGET_LINK_LIBRARIES( pu_prevention_gui framework gui gui_wx vid vil util_time framework vcl vul img img_gui shape shape_gui )

ENDIF( WXWIN_FOUND )