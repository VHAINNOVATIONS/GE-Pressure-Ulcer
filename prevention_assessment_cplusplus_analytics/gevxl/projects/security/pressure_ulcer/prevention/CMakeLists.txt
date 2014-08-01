#/gevxl/projects/security/pressure_ulcer/prevention/CMakeLists.txt

SET( CONFIG_PATH ${gevxlnew_SOURCE_DIR}/projects/security/pressure_ulcer/prevention )

# For naming covention, use pu_prv, where "prv" is short form for prevention
# to avoid confusion with filename with the rest of the VA_Pressure_Ulcer
# project

IF( WXWIN_FOUND )
    
    # proof of concept directory for experimenting with new SDKs
    SUBDIRS( pose_estimate )

    SET ( pu_prevention_proc_sources 

	# the database writer abstract class
	pu_prevention_database_writer.h

	# videoarchive writer and reader
    pu_prevention_videoarchive_frame_process.h     	pu_prevention_videoarchive_frame_process.cxx
	pu_prevention_videoarchive_writer_process.h     pu_prevention_videoarchive_writer_process.cxx
	
	# body motion estimate
    pu_prv_rectify_kinect_process.h    		pu_prv_rectify_kinect_process.cxx
    pu_prv_motion_estimate_process.h   		pu_prv_motion_estimate_process.cxx
	
	# person detection
	pu_prevention_person_detect_process.h     	pu_prevention_person_detect_process.cxx
	
	# body pose estimate
	pu_prevention_pose_estimate_process.h        	pu_prevention_pose_estimate_process.cxx

	# turning protocol monitoring
	pu_prevention_turning_protocol_process.h        pu_prevention_turning_protocol_process.cxx

	# the process chaining
	pu_prevention_chaining_process.h        	pu_prevention_chaining_process.cxx
)

    ADD_LIBRARY( pu_prevention_proc ${pu_prevention_proc_sources} )
    TARGET_LINK_LIBRARIES( pu_prevention_proc vil vnl vgl vbl mvl rrel geom framework img img_descriptor vid util pu_prevention_pose_estimate )

    SET ( pu_prevention_pose_estimate_proc_sources 

	# kinect rectification process
    	pu_prv_rectify_kinect_process.h    		pu_prv_rectify_kinect_process.cxx
	
	# the process chaining
	pu_prevention_pose_estimate_chaining_process.h  pu_prevention_pose_estimate_chaining_process.cxx
)

    ADD_LIBRARY( pu_prevention_pose_estimate_proc ${pu_prevention_pose_estimate_proc_sources} )
    TARGET_LINK_LIBRARIES( pu_prevention_pose_estimate_proc vil vnl vgl vbl mvl rrel geom framework img img_descriptor vid util pu_prevention_pose_estimate )
    
    SET ( pu_prv_motion_demo_gui_app_sources 
        pu_prv_motion_demo_chain_process.h pu_prv_motion_demo_chain_process.cxx
        pu_prv_motion_demo_gui_app.cxx    
        )

    IF( USING_IDE )
        ADD_EXECUTABLE( pu_prv_motion_demo_gui_app 
            ${pu_prv_motion_demo_gui_app_sources} 
            ${CONFIG_PATH}/pu_prv_motion_demo.tconf
            )
    ELSE( USING_IDE )
        ADD_EXECUTABLE( pu_prv_motion_demo_gui_app ${pu_prv_motion_demo_gui_app_sources} )
    ENDIF( USING_IDE )

    TARGET_LINK_LIBRARIES( pu_prv_motion_demo_gui_app gui gui_wx pu_prevention_proc )


    ## pu_prevention_gui_app
    #SET ( pu_prevention_gui_app_sources 
    #    pu_prevention_gui_app.cxx    
    #    )
    #
    #IF( USING_IDE )
    #    ADD_EXECUTABLE( pu_prevention_gui_app 
    #        ${pu_prevention_gui_app_sources} 
    #        ${gevxlnew_SOURCE_DIR}/projects/security/pressure_ulcer/config/pu_prevention.tconf
    #        )
    #ELSE( USING_IDE )
    #    ADD_EXECUTABLE( pu_prevention_gui_app ${pu_prevention_gui_app_sources} )
    #ENDIF( USING_IDE )
    #
    #TARGET_LINK_LIBRARIES( pu_prevention_gui_app gui gui_wx pu_prevention_proc )

    # GUI frames and clients
    SUBDIRS( gui )

    # executables
    SUBDIRS( exec )

ENDIF( WXWIN_FOUND )
