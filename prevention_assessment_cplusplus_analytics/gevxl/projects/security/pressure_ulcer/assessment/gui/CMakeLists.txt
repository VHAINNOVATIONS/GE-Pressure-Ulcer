#/gevxl/projects/security/pressure_ulcer/assessment/gui/CMakeLists.txt

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )


IF( WXWIN_FOUND )

SET( pu_assessment_gui_sources
      
      # GUI frame for the assessment system
      pu_assessment_frame.h      pu_assessment_frame.cxx

      # GUI client for tissue analysis.      
      pu_tissue_analysis_client.h     pu_tissue_analysis_client.cxx
      
      # GUI client for thermal analysis.
      pu_thermal_analysis_client.h    pu_thermal_analysis_client.cxx

      # GUI client for 3d analysis.

      # GUI client for taking the image snapshots
      pu_snapshot_taking_client.h     pu_snapshot_taking_client.cxx
    )
ADD_LIBRARY( pu_assessment_gui ${pu_assessment_gui_sources} )

TARGET_LINK_LIBRARIES( pu_assessment_gui framework gui gui_wx vid vil util_time framework vcl vul img img_gui shape shape_gui pu_assessment_database )

ENDIF( WXWIN_FOUND )