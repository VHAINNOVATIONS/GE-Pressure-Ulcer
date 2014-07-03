import mysql.connector
from mysql.connector import errorcode
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

class Database:
    """
    This class implements a database connection object. It holds information about the connection
    to a MySQL database.
    Methods:
        __init__(dbHost,dbSchema) - Initializes the object with the host URL and the schema name
        Logon(userid, password) - Logs onto the database and sets the connection
    """

    def __init__(self,dbHost,dbSchema):
        """
        Initializes the object with the host URL and the schema name.
        Arguments:
            dbHost - URL of the database server
            dbSchema - Database schema name
        """
        self.dbHost = dbHost
        self.dbSchema = dbSchema
        self.cnx = None

    def Logon(self, userid, password):
        """
        Logs onto the database and sets the connection.
        Arguments:
            userid - User Id
            password - Password
        """
        rc = 0
        msg = ""
        try:
            self.cnx = mysql.connector.connect(user=userid, password=password,
                    host=self.dbHost, database=self.dbSchema)
            self.engine = create_engine('mysql+mysqlconnector://'+userid+':'+password+'@'+self.dbHost+'/'+self.dbSchema)
            Session = sessionmaker(bind=self.engine)
            self.session = Session()
        except mysql.connector.Error as err:
            rc = err.errno
            if err.errno == errorcode.ER_ACCESS_DENIED_ERROR:
                msg = "Something is wrong with your user name or password"
            elif err.errno == errorcode.ER_BAD_DB_ERROR:
                msg = "Database does not exists"
            else:
                msg = err
        else:
            self.cnx.close()
        return (rc, msg)

    def ChangePassword(self, oldPassword, newPassword):
        """
        Changes a user's password
        Arguments:
            oldPassword - Old password
            newPassword - New password
        """
        rc = 0
        msg = ""
        try:
            # Get current user
            userHost = self.session.execute("SELECT CURRENT_USER()").scalar()
            userHostList = userHost.split("@")
            # Verify old password
            cnt = self.session.execute("SELECT COUNT(1) FROM mysql.user WHERE user = :u AND password = PASSWORD(:p)", {'u':userHostList[0], 'p':oldPassword}).scalar()
            if cnt <= 0:
                rc = 4
                msg = "Old password is not correct"
                return (rc, msg)
            resultProxy = self.session.execute("SET PASSWORD = PASSWORD(:p)", {'p':newPassword})
        except Exception as err:
            rc = 8
            msg = err
        return (rc, msg)
