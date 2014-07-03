from pyramid.response import Response
from pyramid.view import view_config
from pyramid.renderers import get_renderer

from sqlalchemy.exc import DBAPIError
from sqlalchemy import desc
from sqlalchemy import func
from sqlalchemy import and_

import inspect
from datetime import datetime

from ..models.dbsession import DBSession

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
    return {'project': 'mmpspupc', 'footer': footer, 'logo': logo, 'nav': nav}

@view_config(route_name='patient-view', renderer='../templates/patient.pt')
def patient_view(request):
    # get other templates that will be included via macros
    footer = get_renderer('../templates/va-footer.pt').implementation()
    logo = get_renderer('../templates/va-logo.pt').implementation()
    nav = get_renderer('../templates/va-navigation.pt').implementation()
    try:
        patients = DBSession.query(PatientIdentification).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patients': patients, 'project': 'mmpspupc', 'footer': footer, 'logo': logo, 'nav': nav}

@view_config(route_name='patients', renderer='json')
def patient_list(request):
    try:
        patients = DBSession.query(PatientIdentification).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patients': patients}

@view_config(route_name='bradenscores', renderer='json')
def braden_scores_list(request):
    try:
        bradenScores = DBSession.query(BradenScores).filter_by(patient_id = request.GET['patient_id']).order_by(desc(BradenScores.braden_scoring_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'bradenScores': bradenScores}

@view_config(route_name='nutritionalstatus', renderer='json')
def nutritional_status_list(request):
    try:
        nutritionalStatus = DBSession.query(NutritionalStatus).filter_by(patient_id = request.GET['patient_id']).order_by(desc(NutritionalStatus.assessment_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'nutritionalStatus': nutritionalStatus}

@view_config(route_name='patientadmission', renderer='json')
def patient_admission_list(request):
    try:
        patientAdmission = DBSession.query(PatientAdmission).filter_by(patient_id = request.GET['patient_id']).order_by(desc(PatientAdmission.admission_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patientAdmission': patientAdmission}

@view_config(route_name='patientassessment', renderer='json')
def patient_assessment_list(request):
    try:
        patientAssessment = DBSession.query(PatientAssessment).filter_by(patient_id = request.GET['patient_id']).order_by(desc(PatientAssessment.assessment_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'patientAssessment': patientAssessment}

@view_config(route_name='treatmentplan', renderer='json')
def treatment_plan_list(request):
    try:
        treatmentPlan = DBSession.query(TreatmentPlan).filter_by(patient_id = request.GET['patient_id']).order_by(desc(TreatmentPlan.plan_date)).all()
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
    return {'treatmentPlan': treatmentPlan}

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

