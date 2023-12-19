# Import Libraries
import sqlite3
import paho.mqtt.client as mqtt
import time
import os

# ROS2 Stuff
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Import modules
print("test")
import ros2_lift_adapter.mqtt_client as mqtt_client
import ros2_lift_adapter.lift_request_handler as RequestHandler
import ros2_lift_adapter.db as db
import ros2_lift_adapter.ros2_fleet as ros2_fleet
# Define topics

# publish to fleet_control/qr_display/qrcode_content/lift_request

TOPICS = {
    "Debug Requests": "fleet_adapter/debug_requests",
    "Lift Request": "fleet_control/qr_display/qrcode_content/lift_request",
    "Fleet Lift State": "fleet_adapter/lift_state"  
}   

REQ_HANDLER = RequestHandler.RequestHandler()

CURRENT_DIRECTORY = os.getcwd()
DB_PATH = os.path.join(CURRENT_DIRECTORY, "src", "ros2_lift_adapter", "ros2_lift_adapter", "data", "lift_requests.db")

class FleetAdapter(mqtt_client.MQTTClient):
    def __init__(self, IP, port):
        super().__init__(IP, port)

        #Create DB object
        self.DB = db.DB(DB_PATH)

        # Create ROS2 Subprocess object
        rclpy.init(args=None) 
        self.ROS2FleetHandler = ros2_fleet.FleetROS2Handler()

        self.lift_state = {
            "curr_level": "00",
            "door_state": "X" # X represents closed, O represents open
        }

    def get_lift_state(self):
        return self.lift_state

    def subscribeToTopics(self, mqttClient):
        mqttClient.subscribe(TOPICS["Fleet Lift State"])
        # mqttClient.subscribe(TOPICS["Debug Requests"])
    
    def update_lift_state(self, client, userdata, msg):
        msg = msg.payload.decode("utf-8")

        self.lift_state["curr_level"] = msg["curr_level"]
        self.lift_state["door_state"] = msg["door_state"]

    def publish_lift_request(self, mqttClient, request):
        # ok this needs to take a message from ROS2 DDS
        # Generate the request string
        request_string = ""
        for value in request.values():
            request_string += value
            request_string += ";"
        try:
            pubMsg = mqttClient.publish(topic=TOPICS["Lift Request"], payload=request_string.encode('utf-8'), qos=0)
            #Library methods, idk what they do
            pubMsg.wait_for_publish()
            print(f"Lift Request {request_string[:3]} successfully published")

        except Exception as e:
            print(f"Error in publishing lift request to 'button_pressed': {e}")
    
    # Generate a new request_id for lift requests
    def generate_new_request_id(self) -> str:
        # Get the most recent ID generated | format: ["request_id_here"]
        hexID = self.DB.get_latest_request_ids(1)[0]

        # Convert to dec, add one then convert back to hex
        decID = int(hexID, base=16)
        newDecID = decID + 1
        newHexID = str(hex(newDecID))

        return newHexID
    
    def generate_lift_request(self, request_level: str, destination_level: str) -> str:
        request_id = self.generate_new_request_id()

        request = REQ_HANDLER.createAndQueueNewLiftRequest(request_id, request_level, destination_level)
        # Add request details to db
        self.DB.add_new_lift_request(request)

    
    def attachCallbackFunctions(self, mqttClient):
        mqttClient.message_callback_add(TOPICS["Fleet Lift State"], self.update_lift_state)

def main():
    IP_ADDRESS = "10.168.2.219"
    mqttClient = mqtt.Client("fleet_adapter") # Create client object
    sim = FleetAdapter(IP_ADDRESS, 1883)

    # Attach on_connect and on_disconnect functionsde
    mqttClient.on_connect = sim.on_connected
    mqttClient.on_disconnect = sim.on_disconnected
    sim.attachCallbackFunctions(mqttClient)

    # Connect and start loop + subscribe to topic
    mqttClient.connect(sim.IP_ADDRESS, sim.PORT)
    mqttClient.loop_start()
    sim.subscribeToTopics(mqttClient)

    while True:
        # Delay for 2 seconds per loop
        time.sleep(2)
        print("Awaiting command...")

        CURRENT_LIFT_STATE = sim.get_lift_state()

        if REQ_HANDLER.lift_queue_is_empty() == False:
            CURRENT_REQUEST_ID = REQ_HANDLER.get_lift_requests_queue()[0]
            CURRENT_REQUEST_DATA = REQ_HANDLER.get_lift_requests_list()[0]

            # Get new request from the ros2 subprocess
            requestData = sim.ROS2FleetHandler.get_request_data()
            if requestData != None: # If the list of request data isnt empty,
                for data in requestData:
                    sim.generate_lift_request(data["request_level"], data["destination_level"]) # create new lift request with given data

            # Publish to fleet manager if lift has reached requested level and doors are opne
            if (CURRENT_REQUEST_DATA["request_level"] == CURRENT_LIFT_STATE["level"]) and (CURRENT_LIFT_STATE == "O"):
                sim.ROS2FleetHandler.publish_lift_state_to_fleet_manager(sim.lift_state)
            
            sim.publish_lift_request(mqttClient, CURRENT_REQUEST_DATA)

        if sim.check_connection() == False:
            print("Connection lost. Attempting to reconnect")
if __name__ == "__main__":
    main()