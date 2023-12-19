import paho.mqtt.client as mqtt # Using Paho MQTT Python client
import copy

        # Template for Lift Requests
LIFT_REQUEST_TEMPLATE = {
    "request_id": "123456",
    "request_level  ": None, 
    "destination_level": None, 
    "service_state" : "0",
    "publish_state" : "0"
}

class RequestHandler():
    def __init__(self):

        # Parallel arrays
        # Queue contains the list of IDs of lift requests
        # List contains all the other info, including the request ID
        self._lift_requests_queue = []
        self._lift_requests_list = []

    # Getter method for lift_requests_lift
    def get_lift_requests_list(self):
        return self._lift_requests_list
    
    # Getter method for lift_requests_queue
    def get_lift_requests_queue(self):
        return self._lift_requests_queue

    # APppends new lift_request request_id to the end
    def enqueue_lift_requests_queue(self, request_id: str):
        self._lift_requests_queue.append(request_id)
    
    # Appends new lift_request info to lift_request_list
    def enqueue_lift_requests_list(self, newReq: dict):
        self._lift_requests_list.append(newReq)

    # Modifies the publish_state attribute in the foremost lift request
    def set_lift_request_publish_state(self, state: str):
        self._lift_requests_list[0]["publish_state"] = state

    # Check if lift_requests_queue is empty, returns bool
    def lift_queue_is_empty(self) -> bool:
        if self._lift_requests_queue == []:
            return True
        else: 
            return False

    # Removing lift requests
    def resolve_lift_request(self, request_id: str):
        REQUEST_INDEX = self._lift_requests_queue.index(request_id)

        # Modify service state
        self._lift_requests_list[REQUEST_INDEX]["service_state"] = "2"

        # Resolving lift request means to 
        #   1) Remove the ID from the queue (_lift_requests_queue)
        #   2) Remove the associated data from the list (_lift_requests_list)
        self._lift_requests_queue.remove(request_id)
        self._lift_requests_list.pop(REQUEST_INDEX)
        print(f"Lift Request ID {request_id} resolved")

    # Creates and queues a new lift request based on the format
    def createAndQueueNewLiftRequest(self, request_id: str, request_level: str, destination_level: str) -> dict:
        # Creating a deep copy and setting the attributes of the lift request
        newReq = copy.deepcopy(LIFT_REQUEST_TEMPLATE)
        newReq["request_id"] = request_id
        newReq["request_level"] = request_level
        newReq["destination_level"] = destination_level
        newReq["service_state"] = "1"

        # Queuing the new lift request
        self.enqueue_lift_requests_queue(request_id)
        self.enqueue_lift_requests_list(newReq)

        print(f"New Lift Request queued. \nID: {request_id} \nRequest Floor: {request_level} \nDestination Floor:{destination_level}")
        return newReq

def main():
    print("This is a module. Do not initialise this.")

if __name__ == "__main__":
    main()