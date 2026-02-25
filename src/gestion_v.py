import sys
from os import path

current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, ".."))

if project_root_path not in sys.path:
    sys.path.append(project_root_path)



from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from flight_plan.command import Command

import numpy as np

"""
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

"""

#Función para aterrizaje, se asume que se le pasa el estado del vertipuerto y el wp inicial (por el que termina su ruta)

def landing_fp(vertiport, initial_wp: Waypoint):

    #Ver si hay disponibilidad en alguno de los gate_pads en el momento en el que se necesite para el UAV y si el takeoff_pad está libre
    if not vertiport.takeoff_pad_free(initial_wp.t) and vertiport.gate_available(initial_wp+50):
        #cosas
        pass
  
    fp = FlightPlan()

    fp.set_waypoint(
        label= "start",
        time= initial_wp.t,
        pos= initial_wp.pos,
        vel= initial_wp.vel
    )

    #Waypoints de por medio

    gate_pad_pos = vertiport.get_free_rest_pad() #función para que devuelva alguno de los gate_pads que está libre

    fp.set_waypoint(
        label= "end",
        time= initial_wp.t+50, #cambiar por algo con sentido
        pos= gate_pad_pos,
        vel= [0,0,0]
    )

    fp.connect_waypoints()

    return fp



#Función para takeoff, se asume que se le pasa el estado del vertipuerto y el wp por el que va a iniciar su ruta

def takeoff_fp(vertiport, exit_wp: Waypoint, t0 = 0):
    
    #Ver si hay disponibilidad en el takeoff_pad
    if not vertiport.takeoff_pad_free(t0):  
        #cosas
        pass

    fp = FlightPlan()
    
    pad_pos = np.array(vertiport.takeoff_pad)

    fp.set_waypoint(
        label="start",
        time=t0,
        pos=pad_pos,
        vel=[0,0,0]
    )

    #Waypoints de por medio

    fp.set_waypoint(
        label="end",
        time=t0+50, #cambiar por algo con sentido
        pos= pad_pos,
        vel= exit_wp.vel
    )


    fp.connect_waypoints()

    return fp