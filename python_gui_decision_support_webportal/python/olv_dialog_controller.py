from sqlalchemy import exc
import re
from pprint import pprint

class OlvDialogController(object):
    """
    Controller class for OLV Dialog which add, updates, and deletes records from a database
    table. It also has a method getting all the records from a table. 
    The database entry to be updated is represented as an object instance.
    The controller uses ObjectListView objects to display results in a panel.
    Methods:
        __init__(db, obj, objOlv, objOlvCols) - Constructor
        addRecord(data) - Adds a record to the database
        convertResults(results) - Convert results to olvResults objects
        deleteRecord(idNum) - Deletes a record from the database
        editRecord(idNum, row) - Edits a record
        getAllRecords() - Gets all records and return them
        getRecordByKey(key) - Get a record and return it
    """
    
    def __init__(self, db, obj, objOlv, objOlvCols):
        """
        Constructor which initializes the class
        Arguments:
            db - Database object
            obj - Database object (model)
            objOlv - ObjectListView object representing the displayed list of objects
            objOlvCols - Column names displayed on the ObjectListView widget
        """
        self.db = db
        self.obj = obj
        self.objOlv = objOlv
        self.objOlvCols = objOlvCols
        
        
    def addRecord(self, data):
        """
        Adds a record to the database. 
        Argument:
            data - Dictionary containing the object attributes
        """
        rc = 0
        msg = ""
        try:
            # print "OlvDialogController.addRecord....data = "
            # pprint(data)
            o = self.obj()
            o.setFromData(data)
            self.db.session.add(o)
            self.db.session.commit()
        except exc.SQLAlchemyError as e:
            print "caught exception of type: "
            print type(e)
            print e
            errorNumSearch = re.search('\(.*\) (\d+) (.*)',str(e))
            rc = int(errorNumSearch.group(1))
            print "error # "+str(rc)
            msg = str(e)
            self.db.session.rollback()
        except Exception as e:
            rc = -1
            msg = str(e)
            self.db.session.rollback()
        finally:
            return (rc, msg)


    def convertResults(self, results):
        """
        Convert results to olvResults objects.
        Arguments:
            results - Result set object from the query
        """
        olvResults = []
        print "# results = %d" % len(results)
        for r in results:
            olvr = self.objOlv(r)
            # print olvr
            olvResults.append(olvr)
        return olvResults

  
    def deleteRecord(self, idNum):
        """
        Deletes a record from the database.
        Arguments:
            idNum - Id of the record to be deleted
        """
        rc = 0
        msg = ""
        try:
            record = self.db.session.query(self.obj).get(idNum)
            self.db.session.delete(record)
            self.db.session.commit()
        except exc.SQLAlchemyError as e:
            print "caught exception of type: "
            print type(e)
            print e
            errorNumSearch = re.search('\(.*\) (\d+) (.*)',str(e))
            rc = int(errorNumSearch.group(1))
            print "error # "+str(rc)
            msg = str(e)
            self.db.session.rollback()
        except Exception as e:
            rc = -1
            msg = str(e)
            self.db.session.rollback()
        finally:
            return (rc, msg)

    
    def editRecord(self, idNum, row):
        """
        Edits a record.
        Arguments:
            idNum - Id of the record to be deleted
            row - Object to be modified
        """
        rc = 0
        msg = ""
        try:
            o = self.db.session.query(self.obj).get(idNum)
            o.setFromData(row)
            self.db.session.add(o)
            self.db.session.commit()
        except exc.SQLAlchemyError as e:
            print "caught exception of type: "
            print type(e)
            print e
            errorNumSearch = re.search('\(.*\) (\d+) (.*)',str(e))
            rc = int(errorNumSearch.group(1))
            print "error # "+str(rc)
            msg = str(e)
            self.db.session.rollback()
        except Exception as e:
            rc = -1
            msg = str(e)
            self.db.session.rollback()
        finally:
            return (rc, msg)

    
    def getAllRecords(self):
        """
        Gets all records and return them
        """
        result = self.db.session.query(self.obj).all()
        olvResults = self.convertResults(result)
        return olvResults

    def getAllFilteredRecords(self,use_filter):
        """
        Gets all filtered records and return them
        """
        result = self.db.session.query(self.obj).filter(use_filter).all()
        olvResults = self.convertResults(result)
        return olvResults

    def getRecordByKey(self, key):
        """
        Get a record and return it
        Arguments:
            key - Id of the record to retrieve
        """
        o = self.db.session.query(self.obj).get(key)
        return o
