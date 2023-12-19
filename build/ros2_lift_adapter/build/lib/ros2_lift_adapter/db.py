import sqlite3
import time

# Commands are stored in constants at the end of the module

class DB():
    def __init__(self, DB_PATH: str):
        self.DB_PATH = DB_PATH

        # Set up connection with sqlite3
        self.CONNECTION = sqlite3.connect(self.DB_PATH)
        self.CURSOR = self.CONNECTION.cursor()

    # Get the request_ids from the last x number of lift requests
    def get_latest_request_ids(self, num: int) -> list:
        res = self.CURSOR.execute(GET_LATEST_REQUEST_IDS.format(num=num))
        return res.fetchall()   
    
    # Get the value of any column in the DB by supplying the request_id and the column name
    def get_lift_request_data(self, request_id: str, data_type: str) -> list:
        res = self.CURSOR.execute(GET_LIFT_REQUEST_DATA.format(data_type=data_type, request_id=request_id))
        return res.fetchall()
    
    # Add a new lift request to the DB by supplying the request
    def add_new_lift_request(self, request: dict, time_stamp: int):
        # Define the request dictionary's keys
        KEYS = ["request_id", "request_level", "destination_level", "service_state", "publish_state"]

        # Create the data tuple
        data = []
        for key in KEYS:
            data.append(request[key])
        data.append(time_stamp)
        data = tuple(data)        

        # Run the SQL command
        self.CURSOR.execute(ADD_NEW_LIFT_REQUEST, data)
        self.CONNECTION.commit()

        print("Request written to database")
    
    # Write an update to the timestamp of request, given the request_id
    def update_time_stamp(self, request_id: str):
        # Generate a new timestamp
        time_stamp = round(time.time())

        res = self.CURSOR.execute(UPDATE_TIME_STAMP.format(time_stamp=time_stamp, request_id=request_id))
        self.CONNECTION.commit()

        print(f"Timestamp for Request {request_id} has been updated.")
    
    def update_publish_state(self, request_id: str, publish_state: str):
        res = self.CURSOR.execute(UPDATE_PUBLISH_STATE.format(publish_state=publish_state, request_id=request_id))
        self.CONNECTION.commit()

        print(f"Publish state for Request {request_id} has been updated.")
    

class FleetDB(DB):
    def __init__(self, DB_PATH):
        super().__init__(DB_PATH)

class LiftDB(DB):
    def __init__(self, DB_PATH):
        super().__init__(DB_PATH)

GET_LATEST_REQUEST_IDS = """
SELECT request_id 
FROM Requests
ORDER BY time_stamp DESC
LIMIT {num}
"""

GET_LIFT_REQUEST_DATA = """
SELECT {data_type}
FROM Requests
WHERE request_id = "{request_id}"
"""

ADD_NEW_LIFT_REQUEST = """
INSERT INTO Requests (request_id, request_level, destination_level, service_state, publish_state, time_stamp)
VALUES (?, ?, ?, ?, ?, ?)
"""

UPDATE_TIME_STAMP  = """
UPDATE Requests
SET time_stamp = {time_stamp}
WHERE request_id = "{request_id}"
"""

UPDATE_PUBLISH_STATE = """
UPDATE Requests
SET publish_state = {publish_state}
WHERE request_id = "{request_id}"
"""
# Testing script
if __name__ == "__main__":

    DB = DB(r"C:\Users\userAdmin\Documents\Dev\python_lift_adapter\data\lift_example.db")
    id = DB.get_latest_request_ids(1)[0][0]
    print(f"ID 1 obtained: {id}")

    data = DB.get_lift_request_data(id, "destination_level")
    print(f"Data obtained: {data}")

    request = {
        "request_id": "AAB",
        "request_level": "01", 
        "destination_level": "09", 
        "service_state" : "0",
        "publish_state" : "0",
        "time_stamp": "1234567890"
    }
    try:
        DB.add_new_lift_request(request, 1234567890)
    except Exception as e:
        print("Error: you probably have the change request_id to smth new if u already committedd this request id previously")
        print(f"heres the actual error :P {e}")

    DB.update_time_stamp("AAB")