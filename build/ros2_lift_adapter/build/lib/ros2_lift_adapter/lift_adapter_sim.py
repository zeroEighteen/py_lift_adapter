# Import libraries
import paho.mqtt.client as mqtt # Using Paho MQTT Python client
import time
import os

# Import modules
import ros2_lift_adapter.mqtt_client as mqtt_client
import ros2_lift_adapter.lift_request_handler as RequestHandler
import ros2_lift_adapter.db as db

# Import ROS2 Modules
import ros2_lift_adapter.ros2_lift as ros2_lift
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPICS = {
        "Door State" : "lift_adapter_transceiver/door_state",
        "Lift Request" : "lift_adapter_transceiver/message_from_fleet",
        "To Fleet": "lift_adapter_transceiver/next_message_to_fleet"
}

class LiftSim(mqtt_client.MQTTClient):
    def __init__(self, IP, port):  
        super().__init__(IP, port)
        # Creates RequestHandler object
        self.reqHandler = RequestHandler.RequestHandler()

        CURRENT_DIRECTORY = os.getcwd()
        self.DB_PATH = os.path.join(CURRENT_DIRECTORY, "src", "ros2_lift_adapter", "ros2_lift_adapter", "data", "lift_requests_copy.db")

        # Create DB obeject
        self.DB = db.DB(self.DB_PATH)

        # Create and initate ROS2 object
        rclpy.init(args=None) 
        self.LiftROS2Handler = ros2_lift.LiftROS2Handler()

        # Dictionary containing lift information from lift_sim
        # └── lift_sim/
        #     ├── current_level
        #     └── door_state
        self._lift_state = {
            "current_level": "1",
            "door_state": "X" # X indicates closed, O indicates open
        }

        self.to_fleet = ""

    # Consolidated function to subscribe to all required topics    
    def subscribe_and_attach_callbacks_to_topics(self, mqttClient):
        mqttClient.subscribe(TOPICS["Door State"]) 
        mqttClient.subscribe(TOPICS["Lift Request"]) 

        mqttClient.message_callback_add(TOPICS["Door State"], self.update_lift_state_door_state)
        mqttClient.message_callback_add(TOPICS["Lift Request"], self.update_lift_request_state)

        print(f"Subscribed to topics.")

    # Getter method for getting the lift's current level
    def get_current_level(self) -> str:
        return self._lift_state["current_level"]

    # Getter method for getting the lift's door state
    def get_door_state(self) -> str:
        return self._lift_state["door_state"]

    # Check if the lift has reached the destination floor. If so, return True
    def destination_level_reached_check(self) -> bool:
        print("Checking Destination")
        LIFT_REQUEST_QUEUE_IS_NOT_EMPTY = len(self.reqHandler.get_lift_requests_queue()) > 0
        DOOR_STATE_IS_OPENED = self.get_door_state() == "Opened" 
        CURRENT_LEVEL_IS_EQUAL_TO_DESTINATION_LEVEL = (self.get_current_level() == self.reqHandler.get_lift_requests_list()[0]["destination_level"])
        print("Finished checking Destination")
        if LIFT_REQUEST_QUEUE_IS_NOT_EMPTY and DOOR_STATE_IS_OPENED and CURRENT_LEVEL_IS_EQUAL_TO_DESTINATION_LEVEL:
            print("reached destination level")
            return True
        
    
    # Callback function: Update current level of _lift_state upon receiving request
    # def update_lift_state_current_level(self, client ,userdata, msg):
    #     try: 
    #         # Decode value from mqtt message
    #         level = msg.payload.decode("utf-8")
    #         # Add leading zero if need be
    #         if len(level) == 1:
    #            level = "0" + level
    #         # Update lift sim state with current level
    #         self._lift_state["current_level"] = level
    #         print(f"Updated Lift State: Floor: {self._lift_state['current_level']}")
    #     except Exception as e:
    #         self._lift_state["current_level"] = None
    #         print(f"Error in updating 'current_level' of _lift_state: {e}\n 'current_level' set to None")
        
    def update_lift_state_current_level(self, curr_level):
        try: 
            level = curr_level
            # Add leading zero if need be
            if len(level) == 1:
               level = "0" + level
            # Update lift sim state with current level
            self._lift_state["current_level"] = level
            print(f"Updated Lift State: Floor: {self._lift_state['current_level']}")
        except Exception as e:
            self._lift_state["current_level"] = None
            print(f"Error in updating 'current_level' of _lift_state: {e}\n 'current_level' set to None")

    # Callback function: Update door state of  _lift_state upon receiving request
    def update_lift_state_door_state(self, client, userdata, msg):
        try: 
            self._lift_state["door_state"] = msg.payload.decode("utf-8")
            print(f"Updated Lift State: Door State: {self._lift_state['door_state']}")

        except Exception as e:
            self._lift_state["door_state"] = None
            print(f"Error in updating 'door_state' of _lift_state: {e}\n 'door_state' set to None")
    
    # Callback function: decode lift requests and create new lift request
    def update_lift_request_state(self, client, userdata, msg):
        try:
            # Parse decoded message
            allInfo =  msg.payload.decode("utf-8")
            IS_BLANK_MESSAGE = "<" in allInfo
            if not IS_BLANK_MESSAGE:
                print("not blank")
                allInfo = allInfo.split(";")
                request_id = allInfo[0]
                request_level = allInfo[1]
                destination_level = allInfo[2]

                # update curr_level
                curr_level = allInfo[5]
                self.update_lift_state_current_level(curr_level)  

                # If request is not already queued, create a new request
                queue = self.reqHandler.get_lift_requests_queue()
                print(f"queue: {queue}")
                if len(queue) > 0:
                    CURR_REQUEST_ID_SMALLER_THAN_NEW_REQUEST_ID = request_id < queue[0]
                else:
                    CURR_REQUEST_ID_SMALLER_THAN_NEW_REQUEST_ID = True
                # for topic cleaning purposes
                print("pre-topic cleaning")
                if (request_id in queue) == False and CURR_REQUEST_ID_SMALLER_THAN_NEW_REQUEST_ID:
                    newRequest = self.reqHandler.createAndQueueNewLiftRequest(request_id, request_level, destination_level)
                    newDB = db.DB(self.DB_PATH)
                    newDB.add_new_lift_request(newRequest, time_stamp=round(time.time()))

            else:
                print("blank")
                curr_level = allInfo.strip(">")
                print(f"post strip: {curr_level}")
                curr_level = curr_level[11:]
                print(f"post strip 2: {curr_level}")
                if len(curr_level) == 1:
                    curr_level = "0" + curr_level
                self.update_lift_state_current_level(curr_level) 
            if not self.reqHandler.lift_queue_is_empty():
                print("queue empty")
                CURRENT_LIFT_REQUEST_ID = self.reqHandler.get_lift_requests_queue()[0]
                CURRENT_LIFT_REQUEST_DATA = self.reqHandler.get_lift_requests_list()[0]
                print("checkpoint")

                REACHED_DESTINATION_LEVEL = self.destination_level_reached_check()
                if REACHED_DESTINATION_LEVEL:
                    #Publish back to fleet
                    CURRENT_LIFT_REQUEST_DATA["service_state"] = "3"
                    print("Update service state")
                    request_string = ""
                    for value in CURRENT_LIFT_REQUEST_DATA.values():
                        request_string += value
                        request_string += ";"
                    print("Finished constructing message")
                    self.to_fleet = request_string
                    

        except Exception as e:
            print(f"Error in updating lift_request_state: {e}")

    # Publish lift_request to lift sim
    def publish_lift_requests_to_lift_sim(self, level, mqttClient):
        try:
            pubMsg = mqttClient.publish(topic="lift_sim/button_pressed", payload=level.encode('utf-8'), qos=0)
            #Library methods, idk what they do
            pubMsg.wait_for_publish()
            print("Message successfully published")

        except Exception as e:
            print(f"Error in publishing lift request to 'button_pressed': {e}")

    def publish_lift_request_to_fleet_adapter(self, data, mqttClient):
        try:
            CURRENT_LIFT_REQUEST_ID = self.reqHandler.get_lift_requests_queue()[0]
            CURRENT_LIFT_REQUEST_DATA = self.reqHandler.get_lift_requests_list()[0]
            pubMsg = mqttClient.publish(topic=TOPICS["To Fleet"], payload=data.encode('utf-8'), qos=0)
            print(f"Waintin for publish: {pubMsg.rc}")
            #Library methods, idk what they do
            pubMsg.wait_for_publish()
            print(f"is published? : {pubMsg.is_published}")
            self.reqHandler.resolve_lift_request(CURRENT_LIFT_REQUEST_ID)
            print("Resolve Lift Request")
            newDB = db.DB(self.DB_PATH)
            newDB.update_service_state(CURRENT_LIFT_REQUEST_ID, "3")
            print("Uodate DB's service state")
            # self.LiftROS2Handler.publish_robot_exit_status_to_robot("leave")
            # Reset to_fleet
            self.to_fleet = ""
        except Exception as e:
            print(f"Error in publishing lift requests to fleet manager: {e}")

def publish_lift_state_update(sim, mqttClient):

    CURRENT_REQUEST_ID = sim.reqHandler.get_lift_requests_queue()[0]
    CURRENT_REQUEST_DATA = sim.reqHandler.get_lift_requests_list()[0]
    print(f"CUrret sim level: {sim.get_current_level()}")
    print(f"CUrret sim level: {CURRENT_REQUEST_DATA['request_level']}")
    print(f"Is current level = request level?; {sim.get_current_level() == CURRENT_REQUEST_DATA['request_level']}")
    print(f"CUrrent publish state: {CURRENT_REQUEST_DATA['publish_state']}")
    if CURRENT_REQUEST_DATA["publish_state"] == "0":
        sim.publish_lift_requests_to_lift_sim(CURRENT_REQUEST_DATA["request_level"], mqttClient)
        sim.reqHandler.set_lift_request_publish_state("1")
        sim.DB.update_publish_state(CURRENT_REQUEST_ID, "1")
        print("Request level successfully published.")
    elif CURRENT_REQUEST_DATA["publish_state"] == "1" and sim.get_current_level() == CURRENT_REQUEST_DATA["request_level"]:
        sim.publish_lift_requests_to_lift_sim(CURRENT_REQUEST_DATA["destination_level"], mqttClient)
        sim.reqHandler.set_lift_request_publish_state("2")
        sim.DB.update_publish_state(CURRENT_REQUEST_ID, "2")
        print("Destination level successfully published.")
    
# Start of script logic ----------------------------
def main():
    mqttClient = mqtt.Client("lift_adapter") # Create client object
    sim = LiftSim("192.168.18.12", 1883)

    # Attach on_connect and on_disconnect functions
    mqttClient.on_connect = sim.on_connected
    mqttClient.on_disconnect = sim.on_disconnected

    # Connect and start loop + subscribe to topic
    mqttClient.connect(sim.IP_ADDRESS, sim.PORT)
    mqttClient.loop_start()
    sim.subscribe_and_attach_callbacks_to_topics(mqttClient)

    # Main loop
    while True:
        time.sleep(2)

        #Print Queue
        print(f"Queue: {sim.reqHandler.get_lift_requests_queue()}")

        # Attempt to publish lift state update if the queue is not empty
        if sim.reqHandler.lift_queue_is_empty() == False:
            publish_lift_state_update(sim, mqttClient)

        # check if need to publish back to fleet
        if sim.to_fleet != "":
            sim.publish_lift_request_to_fleet_adapter(sim.to_fleet, mqttClient)
        
        # Check if connection is present
        if sim.check_connection() == False:
            print("Connection lost. Attempting to reconnect")