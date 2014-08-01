from pyramid.response import Response
from pyramid.view import view_config
from pyramid.renderers import get_renderer

from sqlalchemy import exc
from sqlalchemy.exc import DBAPIError
from sqlalchemy import desc
from sqlalchemy import func
from sqlalchemy import and_

import inspect
from datetime import datetime

from mmpspupc.models.dbsession import DBSession

from mmpspupc.models.patient_identification import PatientIdentification
from mmpspupc.models.braden_scores import BradenScores
from mmpspupc.models.nutritional_status import NutritionalStatus
from mmpspupc.models.patient_admission import PatientAdmission
from mmpspupc.models.patient_assessment import PatientAssessment
from mmpspupc.models.treatment_plan import TreatmentPlan
from mmpspupc.models.algorithm import Algorithm
from mmpspupc.models.experiment import Experiment
from mmpspupc.models.patient_turning import PatientTurning
from mmpspupc.models.wound_assessment import WoundAssessment
from mmpspupc.models.assessment_session import AssessmentSession
from mmpspupc.models.assessment_measurements import AssessmentMeasurements
from mmpspupc.models.assessment_segmentation import AssessmentSegmentation
from mmpspupc.models.assessment_temperature import AssessmentTemperature

from mmpspupc.scripts.prevention_summary_queries import PreventionSummaryQueries

@view_config(route_name='home', renderer='../templates/va.pt')
def my_view(request):
    # get other templates that will be included via macros
    footer = get_renderer('../templates/va-footer.pt').implementation()
    logo = get_renderer('../templates/va-logo.pt').implementation()
    nav = get_renderer('../templates/va-navigation.pt').implementation()
    try:
#         one = DBSession.query(MyModel).filter(MyModel.name == 'one').first()
        one = None
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'one': one, 'project': 'mmpspupc', 'footer': footer, 'logo': logo, 'nav': nav}

@view_config(route_name='patient-list-view', renderer='../templates/patientlist.pt')
def patient_list_view(request):
    # get other templates that will be included via macros
    footer = get_renderer('../templates/va-footer.pt').implementation()
    logo = get_renderer('../templates/va-logo.pt').implementation()
    nav = get_renderer('../templates/va-navigation.pt').implementation()
    patientidentification = get_renderer('../templates/patient-identification.pt').implementation()
    return {'project': 'mmpspupc', 'footer': footer, 'logo': logo, 'nav': nav, 'patientidentification': patientidentification}

@view_config(route_name='patient-view', renderer='../templates/patient.pt')
def patient_view(request):
    # get other templates that will be included via macros
    footer = get_renderer('../templates/va-footer.pt').implementation()
    logo = get_renderer('../templates/va-logo.pt').implementation()
    nav = get_renderer('../templates/va-navigation.pt').implementation()
    patientshort = get_renderer('../templates/patient-short.pt').implementation()
    admission = get_renderer('../templates/patient-admission.pt').implementation()
    assessment = get_renderer('../templates/patient-assessment.pt').implementation()
    braden = get_renderer('../templates/braden-scores.pt').implementation()
    nutritional = get_renderer('../templates/nutritional-status.pt').implementation()
    treatment = get_renderer('../templates/treatment-plan.pt').implementation()
    try:
        patients = DBSession.query(PatientIdentification).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patients': patients, 'project': 'mmpspupc', 'footer': footer, 'logo': logo, 'nav': nav,
            'patientshort': patientshort, 'admission': admission, 'assessment': assessment, 'braden': braden, 
            'nutritional': nutritional, 'treatment': treatment}

@view_config(route_name='patients', renderer='json')
def patient_list(request):
    try:
        patients = DBSession.query(PatientIdentification).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patients': patients}

@view_config(route_name='patientidentification', renderer='json')
def patientidentification(request):
    try:
        patient = DBSession.query(PatientIdentification).get(request.GET['patient_id'])
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patient': patient}

@view_config(route_name='patient-identification-post', renderer='json')
def patient_identification_post(request):
    try:
        msg = ''
        data = {}
        patient_id = int(request.POST['patient_id'])
        if patient_id > 0:
            patient = DBSession.query(PatientIdentification).get(patient_id)
        else:
            patient = PatientIdentification()
        data['va_patient_id'] = request.POST['va_patient_id']
        data['patient_name'] = request.POST['patient_name']
        data['age'] = int(request.POST['age'])
        data['medical_history'] = request.POST['medical_history']
        data['camera_id'] = int(request.POST['camera_id'])
        patient.setFromData(data)
        print "Updating PatientIdentification with patient_id = %d" % (patient_id)
        o = DBSession.merge(patient)
        DBSession.flush()
        DBSession.refresh(o)
        patient_id = o.patient_id
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Update successful"
    except exc.SQLAlchemyError as e:
        print "In patient_identification_post caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In patient_identification_post caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'patient_id': str(patient_id)}
    return {'msg': msg, 'patient_id': str(patient_id)}

@view_config(route_name='patient-identification-delete', renderer='json')
def patient_identification_delete(request):
    try:
        msg = ''
        data = {}
        patient_id = int(request.POST['patient_id'])
        if id <= 0:
            msg = 'Illegal row patient_id (' + str(patient_id) + ') passed to delete routine'
            return
        print "Deleting PatientIdentification with id = %d" % (patient_id)
        patient = DBSession.query(PatientIdentification).get(patient_id)
        DBSession.delete(patient)
        DBSession.flush()
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Delete successful"
    except exc.SQLAlchemyError as e:
        print "In patient_identification_delete caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In patient_identification_delete caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'patient_id': str(patient_id)}
    return {'msg': msg, 'patient_id': str(patient_id)}

@view_config(route_name='patient-short-single', renderer='json')
def patient_short_single(request):
    try:
        patientshort = DBSession.query(PatientIdentification).get(request.GET['patient_id'])
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patientshort': patientshort}

@view_config(route_name='patient-short-post', renderer='json')
def patient_short_post(request):
    try:
        msg = ''
        data = {}
        patient_id = int(request.POST['patient_id'])
        if patient_id > 0:
            patientshort = DBSession.query(PatientIdentification).get(patient_id)
        else:
            raise "KeyError"
        patientshort.age = int(request.POST['age'])
        patientshort.medical_history = request.POST['medical_history']
        print "Updating PatientIdentification with patient_id = %d" % (patient_id)
        o = DBSession.merge(patientshort)
        DBSession.flush()
        DBSession.refresh(o)
        patient_id = o.patient_id
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Update successful"
    except exc.SQLAlchemyError as e:
        print "In patient_short_post caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In patient_short_post caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'patient_id': str(patient_id)}
    return {'msg': msg, 'patient_id': str(patient_id)}

@view_config(route_name='bradenscores', renderer='json')
def braden_scores_list(request):
    try:
        bradenScores = DBSession.query(BradenScores).filter_by(patient_id = request.GET['patient_id']).order_by(desc(BradenScores.braden_scoring_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'bradenScores': bradenScores}

@view_config(route_name='braden-scores-single', renderer='json')
def braden_scores_single(request):
    try:
        bradenScores = DBSession.query(BradenScores).get(request.GET['id'])
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'bradenScores': bradenScores}

@view_config(route_name='braden-scores-post', renderer='json')
def braden_scores_post(request):
    try:
        # bradenScores = DBSession.query(BradenScores).get(request.GET['id'])
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id > 0:
            bradenScores = DBSession.query(BradenScores).get(id)
        else:
            bradenScores = BradenScores()
        data['patient_id'] = int(request.POST['patient_id'])
        data['braden_scoring_date'] = datetime.strptime(request.POST['braden_scoring_date'],"%Y-%m-%d %H:%M:%S")
        data['sensory_perception_score'] = int(request.POST['sensory_perception_score'])
        data['moisture_score'] = int(request.POST['moisture_score'])
        data['activity_score'] = int(request.POST['activity_score'])
        data['mobility_score'] = int(request.POST['mobility_score'])
        data['nutrition_score'] = int(request.POST['nutrition_score'])
        data['friction_shear_score'] = int(request.POST['friction_shear_score'])
        bradenScores.setFromData(data)
        print "Updating BradenScores with id = %d" % (id)
        o = DBSession.merge(bradenScores)
        DBSession.flush()
        DBSession.refresh(o)
        id = o.id
        # msg = "Merged"
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Update successful"
    except exc.SQLAlchemyError as e:
        print "In braden_scores_post caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In braden_scores_post caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='braden-scores-delete', renderer='json')
def braden_scores_delete(request):
    try:
        # bradenScores = DBSession.query(BradenScores).get(request.GET['id'])
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id <= 0:
            msg = 'Illegal row id (' + str(id) + ') passed to delete routine'
            return
        print "Deleting BradenScores with id = %d" % (id)
        bradenScores = DBSession.query(BradenScores).get(id)
        DBSession.delete(bradenScores)
        DBSession.flush()
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Delete successful"
    except exc.SQLAlchemyError as e:
        print "In braden_scores_delete caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In braden_scores_post caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='nutritionalstatus', renderer='json')
def nutritional_status_list(request):
    try:
        nutritionalStatus = DBSession.query(NutritionalStatus).filter_by(patient_id = request.GET['patient_id']).order_by(desc(NutritionalStatus.assessment_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'nutritionalStatus': nutritionalStatus}

@view_config(route_name='nutritional-status-single', renderer='json')
def nutritional_status_single(request):
    try:
        nutritionalStatus = DBSession.query(NutritionalStatus).get(request.GET['id'])
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'nutritionalStatus': nutritionalStatus}

@view_config(route_name='nutritional-status-post', renderer='json')
def nutritional_status_post(request):
    try:
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id > 0:
            nutritionalStatus = DBSession.query(NutritionalStatus).get(id)
        else:
            nutritionalStatus = NutritionalStatus()
        data['patient_id'] = int(request.POST['patient_id'])
        data['assessment_date'] = datetime.strptime(request.POST['assessment_date'],"%Y-%m-%d %H:%M:%S")
        data['nutritional_notes'] = request.POST['nutritional_notes']
        nutritionalStatus.setFromData(data)
        print "Updating NutritionalStatus with id = %d" % (id)
        o = DBSession.merge(nutritionalStatus)
        DBSession.flush()
        DBSession.refresh(o)
        id = o.id
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Update successful"
    except exc.SQLAlchemyError as e:
        print "In nutritional_status_post caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In nutritional_status_post caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='nutritional-status-delete', renderer='json')
def nutritional_status_delete(request):
    try:
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id <= 0:
            msg = 'Illegal row id (' + str(id) + ') passed to delete routine'
            return
        print "Deleting NutritionalStatus with id = %d" % (id)
        nutritionalStatus = DBSession.query(NutritionalStatus).get(id)
        DBSession.delete(nutritionalStatus)
        DBSession.flush()
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Delete successful"
    except exc.SQLAlchemyError as e:
        print "In nutritional_status_delete caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In nutritional_status_delete caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='patientadmission', renderer='json')
def patient_admission_list(request):
    try:
        patientAdmission = DBSession.query(PatientAdmission).filter_by(patient_id = request.GET['patient_id']).order_by(desc(PatientAdmission.admission_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patientAdmission': patientAdmission}

@view_config(route_name='patient-admission-single', renderer='json')
def patient_admission_single(request):
    try:
        patientAdmission = DBSession.query(PatientAdmission).get(request.GET['id'])
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patientAdmission': patientAdmission}

@view_config(route_name='patient-admission-post', renderer='json')
def patient_admission_post(request):
    try:
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id > 0:
            patientAdmission = DBSession.query(PatientAdmission).get(id)
        else:
            patientAdmission = PatientAdmission()
        data['patient_id'] = int(request.POST['patient_id'])
        data['admission_date'] = datetime.strptime(request.POST['admission_date'],"%Y-%m-%d %H:%M:%S")
        data['admission_note'] = request.POST['admission_note']
        data['factors_impairing_healing'] = request.POST['factors_impairing_healing']
        data['patient_group'] = request.POST['patient_group']
        patientAdmission.setFromData(data)
        print "Updating PatientAdmission with id = %d" % (id)
        o = DBSession.merge(patientAdmission)
        DBSession.flush()
        DBSession.refresh(o)
        id = o.id
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Update successful"
    except exc.SQLAlchemyError as e:
        print "In patient_admission_post caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In patient_admission_post caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='patient-admission-delete', renderer='json')
def patient_admission_delete(request):
    try:
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id <= 0:
            msg = 'Illegal row id (' + str(id) + ') passed to delete routine'
            return
        print "Deleting PatientAdmission with id = %d" % (id)
        patientAdmission = DBSession.query(PatientAdmission).get(id)
        DBSession.delete(patientAdmission)
        DBSession.flush()
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Delete successful"
    except exc.SQLAlchemyError as e:
        print "In patient_admission_delete caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In patient_admission_delete caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='patientassessment', renderer='json')
def patient_assessment_list(request):
    try:
        patientAssessment = DBSession.query(PatientAssessment).filter_by(patient_id = request.GET['patient_id']).order_by(desc(PatientAssessment.assessment_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patientAssessment': patientAssessment}

@view_config(route_name='patient-assessment-single', renderer='json')
def patient_assessment_single(request):
    try:
        patientAssessment = DBSession.query(PatientAssessment).get(request.GET['id'])
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patientAssessment': patientAssessment}

@view_config(route_name='patient-assessment-post', renderer='json')
def patient_assessment_post(request):
    try:
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id > 0:
            patientAssessment = DBSession.query(PatientAssessment).get(id)
        else:
            patientAssessment = PatientAssessment()
        data['patient_id'] = int(request.POST['patient_id'])
        data['assessment_date'] = datetime.strptime(request.POST['assessment_date'],"%Y-%m-%d %H:%M:%S")
        data['assessment_note'] = request.POST['assessment_note']
        data['education_notes'] = request.POST['education_notes']
        data['education_understanding'] = request.POST['education_understanding']
        data['education_evidenced_by'] = request.POST['education_evidenced_by']
        patientAssessment.setFromData(data)
        print "Updating PatientAssessment with id = %d" % (id)
        o = DBSession.merge(patientAssessment)
        DBSession.flush()
        DBSession.refresh(o)
        id = o.id
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Update successful"
    except exc.SQLAlchemyError as e:
        print "In patient_assessment_post caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In patient_assessment_post caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='patient-assessment-delete', renderer='json')
def patient_assessment_delete(request):
    try:
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id <= 0:
            msg = 'Illegal row id (' + str(id) + ') passed to delete routine'
            return
        print "Deleting PatientAssessment with id = %d" % (id)
        patientAssessment = DBSession.query(PatientAssessment).get(id)
        DBSession.delete(patientAssessment)
        DBSession.flush()
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Delete successful"
    except exc.SQLAlchemyError as e:
        print "In patient_assessment_delete caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In patient_assessment_delete caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='treatmentplan', renderer='json')
def treatment_plan_list(request):
    try:
        treatmentPlan = DBSession.query(TreatmentPlan).filter_by(patient_id = request.GET['patient_id']).order_by(desc(TreatmentPlan.plan_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'treatmentPlan': treatmentPlan}

@view_config(route_name='treatment-plan-single', renderer='json')
def treatment_plan_single(request):
    try:
        treatmentPlan = DBSession.query(TreatmentPlan).get(request.GET['id'])
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'treatmentPlan': treatmentPlan}

@view_config(route_name='treatment-plan-post', renderer='json')
def treatment_plan_post(request):
    try:
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id > 0:
            treatmentPlan = DBSession.query(TreatmentPlan).get(id)
        else:
            treatmentPlan = TreatmentPlan()
        data['patient_id'] = int(request.POST['patient_id'])
        data['plan_date'] = datetime.strptime(request.POST['plan_date'],"%Y-%m-%d %H:%M:%S")
        data['plan_notes'] = request.POST['plan_notes']
        treatmentPlan.setFromData(data)
        print "Updating TreatmentPlan with id = %d" % (id)
        o = DBSession.merge(treatmentPlan)
        DBSession.flush()
        DBSession.refresh(o)
        id = o.id
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Update successful"
    except exc.SQLAlchemyError as e:
        print "In treatment_plan_post caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In treatment_plan_post caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='treatment-plan-delete', renderer='json')
def treatment_plan_delete(request):
    try:
        msg = ''
        data = {}
        id = int(request.POST['id'])
        if id <= 0:
            msg = 'Illegal row id (' + str(id) + ') passed to delete routine'
            return
        print "Deleting TreatmentPlan with id = %d" % (id)
        treatmentPlan = DBSession.query(TreatmentPlan).get(id)
        DBSession.delete(treatmentPlan)
        DBSession.flush()
        # DBSession.commit() # Not needed since Pyramid uses the Zope transaction manager
        print "Delete successful"
    except exc.SQLAlchemyError as e:
        print "In treatment_plan_delete caught exception of type: "
        print type(e)
        msg = str(e)
        print msg
        DBSession.rollback()
    except Exception as e:
        print "In treatment_plan_delete caught exception of type: "
        print type(e)
        msg = str(e)
        DBSession.rollback()
    finally:
        return {'msg': msg, 'id': str(id)}
    return {'msg': msg, 'id': str(id)}

@view_config(route_name='prevention-event-view', renderer='../templates/prevention-event.pt')
def prevention_event_view(request):
    # get other templates that will be included via macros
    footer = get_renderer('../templates/va-footer.pt').implementation()
    logo = get_renderer('../templates/va-logo.pt').implementation()
    nav = get_renderer('../templates/va-navigation.pt').implementation()
    try:
        algorithm_id = DBSession.query(Algorithm.id).filter(func.lower(Algorithm.algorithm_name)=='prevention pose').scalar()
        experiment_id = DBSession.query(Experiment.id).filter(and_(Experiment.algorithm_id==algorithm_id, Experiment.default_flag==True)).scalar()
        patients = DBSession.query(PatientIdentification).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patients': patients, 'algorithm_id':algorithm_id, 'experiment_id':experiment_id, 'project': 'mmpspupc', 'footer': footer, 'logo': logo, 'nav': nav}

@view_config(route_name='prevention-event-table', renderer='json')
def prevention_event_table(request):
    try:
        turnings = DBSession.query(PatientTurning).filter(and_(PatientTurning.patient_id == request.GET['patient_id'], PatientTurning.experiment_id == request.GET['experiment_id'])).order_by(desc(PatientTurning.turn_time)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    events = []
    for turn in turnings:
        event = turn.__json__(None)
        events.append(event)
    prevTime = None
    for event in reversed(events):
        deltaStr = 'N/A'
        if prevTime:
            delta = datetime.strptime(event['turn_time'],'%Y-%m-%d %H:%M:%S') - datetime.strptime(prevTime,'%Y-%m-%d %H:%M:%S')
            deltaStr = str(delta)
        prevTime = event['turn_time']
        event['interval'] = deltaStr
    return {'events': events}

@view_config(route_name='prevention-summary-view', renderer='../templates/prevention-summary.pt')
def prevention_summary_view(request):
    # get other templates that will be included via macros
    footer = get_renderer('../templates/va-footer.pt').implementation()
    logo = get_renderer('../templates/va-logo.pt').implementation()
    nav = get_renderer('../templates/va-navigation.pt').implementation()
    try:
        algorithm_id = DBSession.query(Algorithm.id).filter(func.lower(Algorithm.algorithm_name)=='prevention pose').scalar()
        experiment_id = DBSession.query(Experiment.id).filter(and_(Experiment.algorithm_id==algorithm_id, Experiment.default_flag==True)).scalar()
        patients = DBSession.query(PatientIdentification).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patients': patients, 'algorithm_id':algorithm_id, 'experiment_id':experiment_id, 'project': 'mmpspupc', 'footer': footer, 'logo': logo, 'nav': nav}

@view_config(route_name='prevention-summary-table', renderer='json')
def prevention_summary_table(request):
    results = PreventionSummaryQueries (request.GET['patient_id'], request.GET['experiment_id'])
    return results

@view_config(route_name='assessment-wound-view', renderer='../templates/assessment-wound.pt')
def assessment_wound_view(request):
    # get other templates that will be included via macros
    footer = get_renderer('../templates/va-footer.pt').implementation()
    logo = get_renderer('../templates/va-logo.pt').implementation()
    nav = get_renderer('../templates/va-navigation.pt').implementation()
    try:
        algorithm_id = DBSession.query(Algorithm.id).filter(func.lower(Algorithm.algorithm_name)=='assessment measure').scalar()
        experiment_id = DBSession.query(Experiment.id).filter(and_(Experiment.algorithm_id==algorithm_id, Experiment.default_flag==True)).scalar()
        patients = DBSession.query(PatientIdentification).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patients': patients, 'algorithm_id':algorithm_id, 'experiment_id':experiment_id, 'project': 'mmpspupc', 'footer': footer, 'logo': logo, 'nav': nav}

@view_config(route_name='assessment-measurements', renderer='json')
def assessment_measurements(request):
    try:
        measurements = DBSession.query(AssessmentMeasurements).join(AssessmentSession, AssessmentMeasurements.session_id == AssessmentSession.id).filter(and_(AssessmentSession.patient_id == request.GET['patient_id'], AssessmentSession.wound_id == request.GET['wound_id'], AssessmentMeasurements.experiment_id == request.GET['experiment_id'])).order_by(desc(AssessmentMeasurements.start_time)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return measurements

@view_config(route_name='assessment-segmentation', renderer='json')
def assessment_segmentation(request):
    try:
        segmentation = DBSession.query(AssessmentSegmentation).join(AssessmentSession, AssessmentSegmentation.session_id == AssessmentSession.id).filter(and_(AssessmentSession.patient_id == request.GET['patient_id'], AssessmentSession.wound_id == request.GET['wound_id'], AssessmentSegmentation.experiment_id == request.GET['experiment_id'])).order_by(desc(AssessmentSegmentation.start_time)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return segmentation

@view_config(route_name='assessment-temperatures', renderer='json')
def assessment_temperature(request):
    try:
        temperatures = DBSession.query(AssessmentTemperature).join(AssessmentSession, AssessmentTemperature.session_id == AssessmentSession.id).filter(and_(AssessmentSession.patient_id == request.GET['patient_id'], AssessmentSession.wound_id == request.GET['wound_id'], AssessmentTemperature.experiment_id == request.GET['experiment_id'])).order_by(desc(AssessmentTemperature.start_time)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return temperatures

@view_config(route_name='wound-dropdown', renderer='json')
def populate_wound_dropdown(request):
    try:
        wounds = DBSession.query(WoundAssessment).filter(WoundAssessment.patient_id == request.GET['patient_id']).order_by(WoundAssessment.wound_location_description).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return wounds


conn_err_msg = """\
Pyramid is having a problem using your SQL database.  The problem
might be caused by one of the following things:

1.  You may need to run the "initialize_mmpspupc_db" script
    to initialize your database tables.  Check your virtual 
    environment's "bin" directory for this script and try to run it.

2.  Your database server may not be running.  Check that the
    database server referred to by the "sqlalchemy.url" setting in
    your "development.ini" file is running.

After you fix the problem, please restart the Pyramid application to
try it again.
"""

