#/gevxl/projects/security/pressure_ulcer/assessment/database/CMakeLists.txt

INCLUDE_DIRECTORIES( ${VXL_OXL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_GEL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${VXL_RPL_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/share )
INCLUDE_DIRECTORIES( ${gevxlnew_SOURCE_DIR}/projects/security )
INCLUDE_DIRECTORIES( ${gevxlnew_BINARY_DIR}/share )

#####################################################
SET( pu_assessment_database_sources

   # database entries
   measurement3d_database_entry.h		   	measurement3d_database_entry.cxx
   segment_database_entry.h		   			segment_database_entry.cxx
   temperature_database_entry.h		   		temperature_database_entry.cxx
   
   # database tables
   measurement3d_database_table.h		   	measurement3d_database_table.cxx
   segment_database_table.h		   			segment_database_table.cxx
   temperature_database_table.h				temperature_database_table.cxx
   
   # database writer and reader
   assessment_database_reader.h				assessment_database_reader.cxx
   assessment_database_writer.h				assessment_database_writer.cxx
)

ADD_LIBRARY( pu_assessment_database ${pu_assessment_database_sources} )
TARGET_LINK_LIBRARIES( pu_assessment_database vil vnl vbl vgl framework database )

# executables
SUBDIRS( exec )
