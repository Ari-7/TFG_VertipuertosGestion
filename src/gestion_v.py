import sys
from os import path

current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, ".."))

if project_root_path not in sys.path:
    sys.path.append(project_root_path)



from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from flight_plan.command import Command

fp = FlightPlan()

fp.set_waypoint(
    label="W1", 
    time=0, 
    pos=[0, 0, 0], 
    vel=[10, 0, 0]
)

fp.set_waypoint(
    label="W2", 
    time=10, 
    pos=[100, 10, 0],
    vel=[10, 0, 0]
)

fp.connect_waypoints()
fp.print_waypoints()

fp.position_figure("FP_1", 1)