from sqlalchemy.exc import DBAPIError
from sqlalchemy import desc
from sqlalchemy import func
from sqlalchemy import and_

import inspect
from datetime import datetime
from datetime import timedelta

from ..models.dbsession import DBSession

from mmpspupc.models.patient_turning import PatientTurning

def PreventionSummaryQueries (patientId, experimentId):
    results = {}
    try:
        begin_all = DBSession.query(func.min(PatientTurning.turn_time)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId)).scalar()
        results['begin_all'] = str(begin_all)
        end_all = DBSession.query(func.max(PatientTurning.turn_time)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId)).scalar()
        results['end_all'] = str(end_all)
        duration_all = end_all - begin_all
        results['duration_all'] = '{:9.1f}'.format(duration_all.total_seconds() / 86400.0)
        count = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId)).scalar()
        count_right = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, func.lower(PatientTurning.final_position)=='right side')).scalar()
        count_left = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, func.lower(PatientTurning.final_position)=='left side')).scalar()
        count_back = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, func.lower(PatientTurning.final_position)=='back')).scalar()
        count_other = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, func.lower(PatientTurning.final_position)=='unknown')).scalar()
        results['right_pc_all'] = '{:.1%}'.format(count_right/float(count))
        results['left_pc_all'] = '{:.1%}'.format(count_left/float(count))
        results['back_pc_all'] = '{:.1%}'.format(count_back/float(count))
        results['other_pc_all'] = '{:.1%}'.format(count_other/float(count))
        results['total_all'] = count
        turnings = DBSession.query(PatientTurning).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId)).order_by(desc(PatientTurning.turn_time)).all()
        events = []
        for turn in turnings:
            event = {'turn_time' : turn.turn_time, 'final_position': turn.final_position}
            events.append(event)
        duration_right = timedelta()
        duration_left = timedelta()
        duration_back = timedelta()
        duration_other = timedelta()
        count_gt2h = 0
        max_delta = timedelta()
        max_delta_time = None
        prevTime = None
        for event in reversed(events):
            delta = timedelta()
            if prevTime:
                delta = event['turn_time'] - prevTime
            prevTime = event['turn_time']
            event['interval'] = delta
            if event['final_position'].lower() == 'right side':
                duration_right = duration_right + delta
            elif event['final_position'].lower() == 'left side':
                duration_left = duration_left + delta
            elif event['final_position'].lower() == 'back':
                duration_back = duration_back + delta
            else:
                duration_other = duration_other + delta
            if delta > timedelta(0,0,0,0,0,2):
                count_gt2h = count_gt2h + 1
            if delta > max_delta:
                max_delta = delta
                max_delta_time = event['turn_time']
        results['right_all'] = '{:.2f}'.format(duration_right.total_seconds()/3600.0)
        results['left_all'] = '{:.2f}'.format(duration_left.total_seconds()/3600.0)
        results['back_all'] = '{:.2f}'.format(duration_back.total_seconds()/3600.0)
        results['other_all'] = '{:.2f}'.format(duration_other.total_seconds()/3600.0)
        results['avgi_all'] = '{:.2f}'.format((duration_right.total_seconds()+duration_left.total_seconds()+duration_back.total_seconds()+duration_other.total_seconds())/3600.0/(count-1))
        results['gt2h_all'] = '{:.1%}'.format(count_gt2h/float(count-1))  
        count_provider = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.provider_present_flag==1)).scalar()
        results['cgp_all'] = '{:.1%}'.format(count_provider/float(count))
        results['longi_all'] = '{:.2f}'.format(max_delta.total_seconds()/3600.0)
        results['longd_all'] = str(max_delta_time)
        results['alertpc_all'] = '{:.1%}'.format(count_gt2h/float(count-1))
        results['alert_all'] = count_gt2h
        results['alertavg_all'] = '{:.2f}'.format(float(count_gt2h)/(duration_all.total_seconds() / 86400.0))
        # Queries for latest 24-hour period
        begin_l24 = end_all - timedelta(1)
         # begin_l24 = DBSession.query(func.min(PatientTurning.turn_time)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.turn_time >= begin_l24)).scalar()
        results['begin_l24'] = str(begin_l24)
        end_l24 = DBSession.query(func.max(PatientTurning.turn_time)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId)).scalar()
        results['end_l24'] = str(end_l24)
        duration_l24 = end_l24 - begin_l24
        results['duration_l24'] = '{:.1f}'.format(duration_l24.total_seconds() / 86400.0)
        count = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.turn_time >= begin_l24)).scalar()
        count_right = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.turn_time >= begin_l24, func.lower(PatientTurning.final_position)=='right side')).scalar()
        count_left = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.turn_time >= begin_l24, func.lower(PatientTurning.final_position)=='left side')).scalar()
        count_back = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.turn_time >= begin_l24, func.lower(PatientTurning.final_position)=='back')).scalar()
        count_other = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.turn_time >= begin_l24, func.lower(PatientTurning.final_position)=='unknown')).scalar()
        results['right_pc_l24'] = '{:.1%}'.format(count_right/float(count))
        results['left_pc_l24'] = '{:.1%}'.format(count_left/float(count))
        results['back_pc_l24'] = '{:.1%}'.format(count_back/float(count))
        results['other_pc_l24'] = '{:.1%}'.format(count_other/float(count))
        results['total_l24'] = count
        turnings = DBSession.query(PatientTurning).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.turn_time >= begin_l24)).order_by(desc(PatientTurning.turn_time)).all()
        events = []
        for turn in turnings:
            event = {'turn_time' : turn.turn_time, 'final_position': turn.final_position}
            events.append(event)
        duration_right = timedelta()
        duration_left = timedelta()
        duration_back = timedelta()
        duration_other = timedelta()
        count_gt2h = 0
        max_delta = timedelta()
        max_delta_time = None
        prevTime = None
        for event in reversed(events):
            delta = timedelta()
            if prevTime:
                delta = event['turn_time'] - prevTime
            prevTime = event['turn_time']
            event['interval'] = delta
            if event['final_position'].lower() == 'right side':
                duration_right = duration_right + delta
            elif event['final_position'].lower() == 'left side':
                duration_left = duration_left + delta
            elif event['final_position'].lower() == 'back':
                duration_back = duration_back + delta
            else:
                duration_other = duration_other + delta
            if delta > timedelta(0,0,0,0,0,2):
                count_gt2h = count_gt2h + 1
            if delta > max_delta:
                max_delta = delta
                max_delta_time = event['turn_time']
        results['right_l24'] = '{:.2f}'.format(duration_right.total_seconds()/3600.0)
        results['left_l24'] = '{:.2f}'.format(duration_left.total_seconds()/3600.0)
        results['back_l24'] = '{:.2f}'.format(duration_back.total_seconds()/3600.0)
        results['other_l24'] = '{:.2f}'.format(duration_other.total_seconds()/3600.0)
        if count > 1:
            results['avgi_l24'] = '{:.2f}'.format((duration_right.total_seconds()+duration_left.total_seconds()+duration_back.total_seconds()+duration_other.total_seconds())/3600.0/(count-1))
        else:
            results['avgi_l24'] = '0.0%'
        if count > 1:
            results['gt2h_l24'] = '{:.1%}'.format(count_gt2h/float(count-1))  
        else:
            results['gt2h_l24'] = '0.0%'
        count_provider = DBSession.query(func.count(PatientTurning.final_position)).filter(and_(PatientTurning.patient_id == patientId, PatientTurning.experiment_id == experimentId, PatientTurning.turn_time >= begin_l24, PatientTurning.provider_present_flag==1)).scalar()
        results['cgp_l24'] = '{:.1%}'.format(count_provider/float(count))
        results['longi_l24'] = '{:.2f}'.format(max_delta.total_seconds()/3600.0)
        results['longd_l24'] = str(max_delta_time)
        if count > 1:
            results['alertpc_l24'] = '{:.1%}'.format(count_gt2h/float(count-1))
        else:
            results['alertpc_l24'] = '0.0%'
        results['alert_l24'] = count_gt2h
        results['alertavg_l24'] = '{:.2f}'.format(float(count_gt2h)/(duration_all.total_seconds() / 86400.0))
        return results
    except DBAPIError:
        return Response(conn_err_msg, content_type='text/plain', status_int=500)
