# Import modules for GUI
import tkinter as tk
from functools import partial

# Import modules for ROS2 communication
from .ros2_sim_handler import ROS2SimPublisher
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Create GUI class, for use in ROS2 environment
class GUI:
    def __init__(self):
        self.WINDOW_TITLE = "ROS2 Lift Request Simulator"

        # Initialise ROS2 
        rclpy.init(args=None)   
        self.ros2_sim_publisher = ROS2SimPublisher()

        # Setup GUI WINDOW
        self.WINDOW = tk.Tk()
        self.WINDOW.title = self.WINDOW_TITLE

        # Create "global" the request and destination levels 
        self.curr_request_level = "None"
        self.curr_destination_level = "None"

        # Create column header label widgets
        self.title_request_level_label = tk.Label(self.WINDOW, text="Select Req Level")
        self.title_destination_level_label = tk.Label(self.WINDOW, text="Select Dest Level")
        self.title_request_level_label.grid(row=0, column=0)
        self.title_destination_level_label.grid(row=0, column=1)

        # Create lift button widgets 1 to 12
        for x in list(range(1, 13, 1)):
            self.action_req_level = partial(self.update_curr_req_level_label, x)
            self.action_dest_level = partial(self.update_curr_destination_level_label, x)
            self.create_level_button(x, command=self.action_req_level, row=x, column=0)
            self.create_level_button(x, command=self.action_dest_level, row=x, column=1)

        #create selected levels label
        self.curr_request_level_label = tk.Label(self.WINDOW, text="Req: " + self.curr_request_level)
        self.curr_destination_level_label = tk.Label(self.WINDOW, text="Dest: " + self.curr_destination_level)
        self.curr_request_level_label.grid(row=13, column=0, pady=2)
        self.curr_destination_level_label.grid(row=13, column=1, pady=2) 

        # create submit request button widget
        self.submitButton = tk.Button(self.WINDOW, text="Submit request", command=self.submit_request)
        self.submitButton.grid(row=14, column=0, columnspan=2)


    
    # Function to create level button
    def create_level_button(self, level, command, row, column):
        y = tk.Button(self.WINDOW, text=str(level), width=10, command=command)
        y.grid(row=row, column=column, pady = 2)

    # Function to add leading 0 and update label
    def update_curr_req_level_label(self, level):
        if len(str(level)) == 1:
            level = "0" + str(level)
        self.curr_request_level_label.config(text=level)
        self.curr_request_level = level

    # Function to add leading 0 and update label
    def update_curr_destination_level_label(self, level):
        if len(str(level)) == 1:
            level = "0" + str(level)
        self.curr_destination_level_label.config(text=level)
        self.curr_destination_level = level

    #  Function called upon submit button pressed
    def submit_request(self):
        self.ros2_sim_publisher.publish_lift_request(str(self.curr_request_level) + ";" + str(self.curr_destination_level))
        print("Request submitted")  
        # update label text
        self.curr_request_level_label.config(text="None")
        self.curr_destination_level_label.config(text="None")
        self.curr_destination_level = "None"
        self.curr_request_level = "None"

    #  Function to start loop
    def startLoop(self):
        self.WINDOW.mainloop()
        

def main():
    start = GUI()
    start.startLoop()
if __name__ == "__main__":
    main()