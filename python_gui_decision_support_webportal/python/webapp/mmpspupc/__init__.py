from pyramid.config import Configurator
from pyramid.renderers import get_renderer
from pyramid.renderers import JSON
from sqlalchemy import engine_from_config
import datetime

from models.dbsession import (
    DBSession,
    Base,
    )


def main(global_config, **settings):
    """ This function returns a Pyramid WSGI application.
    """
    engine = engine_from_config(settings, 'sqlalchemy.')
    DBSession.configure(bind=engine)
    Base.metadata.bind = engine
    config = Configurator(settings=settings)
    # jsonRenderer = JSON()
    # def datetime_adapter(obj, request):
    #     return obj.isoformat()
    # jsonRenderer.add_adapter(datetime.datetime, datetime_adapter)
    # config.add_renderer('json', jsonRenderer)
    config.include('pyramid_chameleon')
    config.add_static_view('static', 'static', cache_max_age=3600)
    config.add_route('home', '/')
    config.add_route('patient-list-view', '/patient-list-view')
    config.add_route('patient-view', '/patient-view')
    config.add_route('patients', '/patients')
    config.add_route('bradenscores', '/bradenscores')
    config.add_route('nutritionalstatus', '/nutritionalstatus')
    config.add_route('patientadmission', '/patientadmission')
    config.add_route('patientassessment', '/patientassessment')
    config.add_route('treatmentplan', '/treatmentplan')
    config.add_route('prevention-event-view', '/prevention-event-view')
    config.add_route('prevention-event-table', '/prevention-event-table')
    config.add_route('prevention-summary-view', '/prevention-summary-view')
    config.add_route('prevention-summary-table', '/prevention-summary-table')
    config.add_route('assessment-wound-view', '/assessment-wound-view')
    config.add_route('wound-dropdown', '/wound-dropdown')
    config.add_route('assessment-measurements', '/assessment-measurements')
    config.add_route('assessment-segmentation', '/assessment-segmentation')
    config.add_route('assessment-temperatures', '/assessment-temperatures')
    config.scan()
    return config.make_wsgi_app()
