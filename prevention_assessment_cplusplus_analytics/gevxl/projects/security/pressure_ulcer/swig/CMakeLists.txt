PROJECT(pressure_ulcer_python)

cmake_minimum_required(VERSION 2.6)

IF( GEVXL_USE_PYTHON )
  #FILE(APPEND ${PYTHON_PATH_FILE} "${gevxl_security_SOURCE_DIR}/pressure_ulcer/python\n")
  
  INCLUDE_DIRECTORIES( ${PYTHON_INCLUDE_PATH} )
  
  IF( BUILD_GEVXL_PYTHON_LIBRARY )
  FIND_PACKAGE(SWIG)
  IF(SWIG_FOUND)
  INCLUDE(${GEVXL_CMAKE_DIR}/UseGEVXLSWIG.cmake)
  
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
  INCLUDE_DIRECTORIES(${gevxl_security_SOURCE_DIR}/pressure_ulcer/python)

  SET(CMAKE_SWIG_FLAGS "-I${gevxl_security_SOURCE_DIR}/pressure_ulcer/swig"
                       "-I${gevxl_share_SOURCE_DIR}/framework/swig"
                       "-I${gevxl_share_SOURCE_DIR}/img/swig"
                       "-I${gevxl_share_SOURCE_DIR}/util/swig"
                       "-I${gevxl_share_SOURCE_DIR}/vid/swig"
                       "-I${gevxl_share_SOURCE_DIR}/shape/swig"
                       "-I${gevxl_share_SOURCE_DIR}/geom/swig"
                       "-I${gevxl_security_SOURCE_DIR}/scripting"
                       "-I${gevxl_share_SOURCE_DIR}/scripting")

  SET(CMAKE_GEVXL_SWIG_OUTDIR ${PYTHON_GEVXL_FOLDER}/gevxl/security/pressure_ulcer)
  
  GEVXL_SWIG_ADD_MODULE(pressure_ulcer python pressure_ulcer.i )
                                           
  GEVXL_SWIG_LINK_LIBRARIES(pressure_ulcer ${PYTHON_LIBRARIES} pu_camera_source pu_prevention_proc pu_assessment_proc pu_assessment_biochemical _framework _img _util _vid _shape _geom )
  
  ENDIF(SWIG_FOUND)
  ENDIF( BUILD_GEVXL_PYTHON_LIBRARY )
ENDIF( GEVXL_USE_PYTHON )
