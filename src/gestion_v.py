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
import numpy as np
from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint

class VertiportManager:

    def __init__(self, id="", name="", main_pad=None, pads=None, booking_buffer=40):

        self.id = id
        self.name = name

        self.main_pad = main_pad
        self.pads = pads or {}
        self.booking_buffer = booking_buffer

        self.D = 10.0               #D del UAV
        self.h1 = 3.0               #Altura del hover bajo
        self.h2 = 30.5              #Altura del hover alto - Límite OFV
        self.slope_c = 0.125        #Pendiente de categoría C (12,5%)
        self.v_vertical = 0.8       #Velocidad vertical (m/s)
        self.v_rod = 0.5            #Velocidad de rodaje (m/s)
        self.v_approach = 3.0       #Velocidad de aproximación/salida (m/s)

        # Variables de tiempo y distancia para las maniobras
        self.time_buffer_landing = 60.0  
        self.time_to_rest = 5.0          
        self.taxi_height = 1.5        
        self.time_final_rest = 2.0
        self.departure_slope_dist = 100.0

    def get_pad_booking(self, start_time, duration, buffer):
        
            eligible = []

            for pad_id, pad in self.pads.items():

                if pad.is_available(start_time, start_time + duration, buffer):

                    num_bookings = len(pad.get_bookings())
                    eligible.append((pad, num_bookings))
                
            # Si hemos encontrado al menos un pad libre en este tiempo
            if eligible:

                eligible.sort(key=lambda x: x[1], reverse=True)
                    
                best_pad, n_bookings = eligible[0]
                return best_pad, start_time, n_bookings
                    
            return None

    def generate_fp(self, initial_wp, exit_heading, parking_duration=1200) -> 'FlightPlan':
        """
        Genera un plan de vuelo completo unificado (Landing + Takeoff).
        Realiza las validaciones de disponibilidad para toda la misión antes de reservar.
        """

        if not self.main_pad or not self.pads:
            raise ValueError("Pads no configurados")

        ### Cálculo de tiempos landing ###

        main_loc = np.array(self.main_pad.location)
        dist_total_approach = np.linalg.norm(np.array(initial_wp.pos) - main_loc)
        
        # Tiempos de fase de aterrizaje
        t_h2_land = initial_wp.t + (dist_total_approach / self.v_approach)
        t_h1_land = t_h2_land + ((self.h2 - self.h1) / self.v_vertical)
        t_start_taxi_land = t_h1_land + self.time_to_rest
        
        ### Disponibilidad TLOF ###

        if not self.main_pad.is_available(initial_wp.t, t_start_taxi_land, self.booking_buffer):
            raise RuntimeError("Error: TLOF ocupado para el aterrizaje en el tiempo solicitado.")

        ### Búsqueda stand ###

        # Se busca un pad que esté libre desde que llega (t_start_taxi_land) 
        # hasta que inicia el despegue tras la estancia.
        t_end_stay = t_start_taxi_land + parking_duration
   
        result = self.get_pad_booking(t_start_taxi_land, parking_duration, self.booking_buffer)
        
        if not result:
            raise RuntimeError("Error: No hay disponibilidad en ningún pad de descanso para la estancia.")
            
        parking_pad, _, _ = result
        park_loc = np.array(parking_pad.location)
        dist_taxi = np.linalg.norm(park_loc[:2] - main_loc[:2])
        t_end_taxi_land = t_start_taxi_land + (dist_taxi / self.v_rod)
        t_final_land = t_end_taxi_land + self.time_final_rest

        ### Tiempos takeoff ###

        t_start_takeoff = t_end_stay # El despegue inicia tras la estancia
        t_stand_h1_takeoff = t_start_takeoff + (self.h1 / self.v_vertical)
        t_at_tlof_takeoff = t_stand_h1_takeoff + (dist_taxi / self.v_rod) + self.time_to_rest
        t_h2_takeoff = t_at_tlof_takeoff + ((self.h2 - self.h1) / self.v_vertical)

        ### Disponibilidad TLOF takeoff ###

        if not self.main_pad.is_available(t_stand_h1_takeoff, t_h2_takeoff, self.booking_buffer):
            raise RuntimeError("Error: TLOF ocupado para el despegue programado.")
        


        # Reserva TLOF (Landing)
        self.main_pad.book(initial_wp.t, t_start_taxi_land, self.booking_buffer)

        # Reserva Pad de descanso
        parking_pad.book(t_start_taxi_land, t_start_takeoff, self.booking_buffer)

        # Reserva TLOF (Takeoff)
        self.main_pad.book(t_stand_h1_takeoff, t_h2_takeoff, self.booking_buffer)

        fp = FlightPlan()

        # Aterrizaje #

        # Punto de aproximación
        fp.set_waypoint(label="start", time=initial_wp.t, pos=initial_wp.pos, vel=initial_wp.vel)
        # Entrada al OFV
        fp.set_waypoint(label="OFV_h2", time=t_h2_land, pos=(main_loc + [0, 0, self.h2]).tolist(), vel=[0, 0, -self.v_vertical])
        # Hover en TLOF
        fp.set_waypoint(label="TLOF_Hover", time=t_h1_land, pos=(main_loc + [0, 0, self.h1]).tolist(), vel=[0, 0, 0])
        # Paso al pad de descanso
        fp.set_waypoint(label="Taxi_to_Stand", time=t_end_taxi_land, pos=(park_loc + [0, 0, self.h1]).tolist(), vel=[0, 0, 0])
        # Reposo en el pad de descanso
        fp.set_waypoint(label="Final_Land", time=t_final_land, pos=parking_pad.location, vel=[0, 0, 0])

        # Taxi y despegue #

        # Inicio en el pad de descanso (tras la estancia)
        fp.set_waypoint(label="Stand_Start", time=t_start_takeoff, pos=parking_pad.location, vel=[0, 0, 0])

        # El UAV se posiciona a la altura h1 para ir al TLOF
        fp.set_waypoint(label="Hover_at_h1", time=t_stand_h1_takeoff, pos=(park_loc + [0, 0, self.h1]).tolist(), vel=[0, 0, 0])

        # Llegada al TLOF a altura h1
        fp.set_waypoint(label="TLOF_h1", time=t_at_tlof_takeoff, pos=(main_loc + [0, 0, self.h1]).tolist(), vel=[0, 0, 0])

        # Ascenso vertical desde h1 hasta h2 (límite del OFV)
        fp.set_waypoint(label="OFV_h2_Exit", time=t_h2_takeoff, pos=(main_loc + [0, 0, self.h2]).tolist(), vel=[0, 0, self.v_vertical])

        # Salida final
        heading_norm = np.linalg.norm(exit_heading)
        unit_heading = np.array(exit_heading) / heading_norm
        dist_slope = self.departure_slope_dist
        z_final = self.h2 + (dist_slope * self.slope_c)
        pos_final = (np.array(main_loc + [0, 0, self.h2]) + (unit_heading * dist_slope))
        pos_final[2] = z_final
        t_final = t_h2_takeoff + (dist_slope / self.v_approach)

        fp.set_waypoint(label="Departure_Slope", time=t_final, pos=pos_final.tolist(), vel=[0, 0, 0])

        fp.connect_waypoints()
        return fp