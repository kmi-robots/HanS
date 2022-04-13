import psycopg2
import keyring # used for more secure pw storage

def connect_db(user, dbname):

    try:
        conn = psycopg2.connect(dbname=dbname, user=user, password=keyring.get_password(dbname, user))
        cur = conn.cursor()
        print(conn.get_dsn_parameters(), "\n")
        cur.execute("SELECT version();")
        record = cur.fetchone()
        print('You are connected to' + str(record))

    except (Exception, psycopg2.Error) as error:
        print('Error while connecting to PostgreSQL ' +str(error))
        conn = None
        cur = None

    return conn, cur


def disconnect_DB(connection,cursor):

    cursor.close()
    connection.close()
    print("Closed PostgreSQL connection")