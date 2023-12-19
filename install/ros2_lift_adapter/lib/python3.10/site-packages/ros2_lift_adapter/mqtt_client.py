class MQTTClient():
    def __init__(self, IP, port):
        # Declare IP address and port of MQTT broker
        self.IP_ADDRESS = IP
        self.PORT = port

        # States connection state with the MQTT server
        # False = Disconnected, True = Connected
        self._isConnected = False

    # Assigned to client.on_connect, which runs every time the client establishes a connection with the MQTT server
    def on_connected(self, client, userdata, flags, rc):
        # Update connection state
        self._isConnected = True

        print("Connected to MQTT broker.")

    # Assigned to client.on_disconnect, which runs every time the client disconnects from the MQTT broker
    def on_disconnected(self, client, userdata, rc):
        self._isConnected = False
        print("Disconnected from MQTT broker")

    def check_connection(self) -> bool:
        if self._isConnected == True:
            return True
        else:
            return False
        
def main():
    print("This is a module. Do not initialise this.")

if __name__ == "__main__":
    main()

        