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
import ros2_lift_adapter.mqtt_client as mqtt_client
import ros2_lift_adapter.lift_request_handler as RequestHandler
import ros2_lift_adapter.db as db
import ros2_lift_adapter.ros2_fleet as ros2_fleet
# Define topics

# publish to fleet_control/qr_display/qrcode_content/lift_request

TOPICS = {
    "To Lift": "fleet_adapter_transceiver/next_message_to_lift/",
    "From Lift": "fleet_adapter_transceiver/message_from_lift/*",
    "Lift Sim State": "fleet_adapter_transceiver/door_state/*",
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
            "door_state": "Closed", # X represents closed, O represents open
            "door_state_changes" : "False",
            "curr_level_changes" : "False"
        }

    def get_lift_state(self):
        return self.lift_state

    def subscribeToTopics(self, mqttClient):
        mqttClient.subscribe(TOPICS["From Lift"])
        mqttClient.subscribe(TOPICS["Lift Sim State"])

        # mqttClient.subscribe(TOPICS["Debug Requests"])
    
    def update_lift_state(self, client, userdata, msg):
        msg = msg.payload.decode("utf-8")

        data = msg.split(";")
        # Check if there are changes
        NEW_CURR_LEVEL = data[0]
        NEW_DOOR_STATE = data[1]
        if self.lift_state["curr_level"] != NEW_CURR_LEVEL:
            self.lift_state["curr_level"] = NEW_CURR_LEVEL
            self.lift_state["curr_level_changes"] = "True"
        
        if self.lift_state["door_state"] != NEW_DOOR_STATE:
            self.lift_state["door_state"] = NEW_DOOR_STATE
            self.lift_state["door_state_changes"] = "True"
        
    def publish_lift_request(self, mqttClient, request):
        # ok this needs to take a message from ROS2 DDS
        # Generate the request string
        # Change service_state to 2, indicaing request s being served
        request["service_state"] = "2"
        request_string = ""
        for value in request.values():
            request_string += value
            request_string += ";"
        # Customise topic to publish to
        CUSTOMISED_TOPIC = TOPICS["To Lift"] + self.lift_state["curr_level"]
        try:
            pubMsg = mqttClient.publish(topic=CUSTOMISED_TOPIC, payload=request_string.encode('utf-8'), qos=0)
            #Library methods, idk what they do
            pubMsg.wait_for_publish()
            print(f"Lift Request {request_string[:3]} successfully published")

        except Exception as e:
            print(f"Error in publishing lift request to 'button_pressed': {e}")

    def publish_blank_request_and_curr_level(self, mqttClient):
        CURR_LEVEL = self.lift_state["curr_level"]
        msg = "<blank>;" + CURR_LEVEL
        try:
            pubMsg = mqttClient.publish(topic=TOPICS["Lift Request"], payload=CURR_LEVEL.encode('utf-8'), qos=0)
            #Library methods, idk what they do
            pubMsg.wait_for_publish()
            print(f"Blank Lift Request for Level {CURR_LEVEL} successfully published")

        except Exception as e:
            print(f"Error in publishing lift request to 'button_pressed': {e}")

    # Generate a new request_id for lift requests
    def generate_new_request_id(self) -> str:
        # Get the most recent ID generated | format: ["request_id_here"]
        hexID = self.DB.get_latest_request_ids(1)[0][0]
        # Convert to dec, add one then convert back to hex
        decID = int(hexID, base=16)
        newDecID = decID + 1
        newHexID = f"{newDecID:x}"

        return newHexID
    
    def generate_lift_request(self, request_level: str, destination_level: str) -> str:
        request_id = self.generate_new_request_id()

        request = REQ_HANDLER.createAndQueueNewLiftRequest(request_id, request_level, destination_level)
        # Add request details to db
        self.DB.add_new_lift_request(request, time_stamp=round(time.time()))
    
    def complete_lift_request(self, request_id):
        REQ_HANDLER.resolve_lift_request(request_id)
        self.DB.update_service_state(request_id, "3")
        print(f"Resolved lift request ID {request_id}")
    
    def update_current_request(self, client, userdata, msg):
        msg = msg.payload.decode("utf-8")
        data = msg.split(";")
        # Check if service state is 3
        SERVICE_STATE = data[3]
        if SERVICE_STATE == "3":
            REQ_HANDLER.set_service_state("3")
        

    
    def attachCallbackFunctions(self, mqttClient):
        mqttClient.message_callback_add(TOPICS["Lift Sim State"], self.update_lift_state)
        mqttClient.message_callback_add(TOPICS["From Lift"], self.update_current_request)
        

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
        # Check for lift state updates
        CURRENT_LIFT_STATE = sim.get_lift_state()
        if CURRENT_LIFT_STATE["door_state_changes"] == "True":
            sim.publish_door_state(mqttClient, sim.get_lift_state())
        if CURRENT_LIFT_STATE["curr_level_changes"] == "True":
            sim.publish_curr_level(mqttClient, sim.get_lift_state())
            
        # Get new request from the ros2 subprocess
        print("Checking for requests from ROS2 DDS")
        requestData = ros2_fleet.check_for_subscriptions(sim.ROS2FleetHandler)
        if requestData != None: # If the list of request data isnt empty,
            for data in requestData:
                sim.generate_lift_request(data["request_level"], data["destination_level"]) # create new lift request with given data
        
        if REQ_HANDLER.lift_queue_is_empty() == False:
            CURRENT_REQUEST_ID = REQ_HANDLER.get_lift_requests_queue()[0]
            CURRENT_REQUEST_DATA = REQ_HANDLER.get_lift_requests_list()[0]
            if CURRENT_REQUEST_DATA["service_state"] == "1": # If the current lift request service state has nto been served
                # Publish to fleet manager if lift has reached requested level and doors are opne
                if (CURRENT_REQUEST_DATA["request_level"] == CURRENT_LIFT_STATE["curr_level"]) and (CURRENT_LIFT_STATE == "O"):
                    sim.ROS2FleetHandler.publish_lift_state_to_fleet_manager(sim.lift_state)
                
                sim.publish_lift_request(mqttClient, CURRENT_REQUEST_DATA) # Publish lift request to optical transceiver
            else:
                # if request has been served, send blank message
                sim.publish_blank_request_and_curr_level(mqttClient) 

            # Check if current lift request has been served
            if CURRENT_REQUEST_DATA["service_state"] == "3":
                sim.complete_lift_request(CURRENT_REQUEST_ID)
        else:
            # publish blank message
            sim.publish_blank_request_and_curr_level(mqttClient) 

        if sim.check_connection() == False:
            print("Connection lost. Attempting to reconnect")
if __name__ == "__main__":
    main()
