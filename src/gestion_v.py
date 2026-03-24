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
        self.default_booking_time = 3600 
        self.departure_slope_dist = 100.0

    def get_pad(self, start_time, duration, buffer, strategy="least_used"):
            
            """
            Busca el hueco más próximo en el tiempo. 
            Si hay varios pads con el mismo tiempo de inicio mínimo, elige según la estrategia.
            """

            max_search = 7200  # 2 horas de margen
            step = 60         # Resolución de búsqueda (1 minuto)
            
            for t_offset in range(0, max_search + 1, step):
                t_current = start_time + t_offset
                t_end = t_current + duration
                
                eligible = []
                
                for pad_id, pad in self.pads.items():
                    if pad.is_available(t_current, t_end, buffer):

                        num_bookings = len(pad.get_bookings())
                        eligible.append((pad, num_bookings))
                
                # Si hemos encontrado al menos un pad libre en este tiempo
                if eligible:
                    if strategy == "least_used":
            
                        eligible.sort(key=lambda x: x[1])
                    else:

                        eligible.sort(key=lambda x: x[1], reverse=True)
                    
                    best_pad, n_bookings = eligible[0]
                    return best_pad, t_current, n_bookings
                    
            return None

    #Función para aterrizaje, se asume que se le pasa el estado del vertipuerto y el wp inicial (por el que termina su ruta)
    def landing_fp(self, initial_wp, strategy="least_used") -> 'FlightPlan':

        if not self.main_pad or not self.pads:
            raise ValueError("Pads no configurados")
        
        parking_duration = self.default_booking_time
        buffer = self.booking_buffer
        
        result = self.get_pad(initial_wp.t, parking_duration, buffer, strategy)
        
        if not result:
            raise RuntimeError("No hay disponibilidad en ningún pad")
            
        parking_pad, t_start_real, _ = result
        
        fp = FlightPlan()

        #Punto de aproximación
        fp.set_waypoint(label="start", time = t_start_real, pos = initial_wp.pos, vel = initial_wp.vel)

        main_loc = np.array(self.main_pad.location)

        dist_total = np.linalg.norm(np.array(initial_wp.pos) - main_loc)

        tiempo = (dist_total / self.v_approach) + (self.h2 / self.v_vertical) + self.time_buffer_landing

        #Entrada al OFV
        t_h2 = initial_wp.t + (dist_total / self.v_approach)
        pos_h2 = main_loc + [0, 0, self.h2]
        fp.set_waypoint(label="OFV_h2", time=t_h2, pos=pos_h2.tolist(), vel=[0, 0, -self.v_vertical])

        #Hover en TLOF
        t_land = t_h2 + ((self.h2 - self.h1) / self.v_vertical)
        fp.set_waypoint(label="TLOF_Hover", time=t_land, pos=(main_loc + [0, 0, self.h1]).tolist(), vel=[0, 0, 0])

        #Paso al pad de descanso
        t_start_rest = t_land + self.time_to_rest
        park_loc = np.array(parking_pad.location)
        dist_rest = np.linalg.norm(park_loc[:2] - main_loc[:2])
        t_end_rest = t_start_rest + (dist_rest / self.v_rod)
        
        pos_taxi = park_loc + [0, 0, self.h1]
        fp.set_waypoint(label="Taxi_to_Stand", time=t_end_rest, pos=pos_taxi.tolist(), vel=[0, 0, 0])

        #Reposo en el pad de descanso
        t_final = t_end_rest + self.time_final_rest
        fp.set_waypoint(label="Final", time=t_final, pos=parking_pad.location, vel=[0, 0, 0])

        # Realizar reservas
        self.main_pad.book(initial_wp.t, t_start_rest, self.booking_buffer)
        parking_pad.book(t_start_rest, t_final + self.default_booking_time, self.booking_buffer)

        fp.connect_waypoints()
        
        return fp

    #Función para despegue, desde el pad de descanso hasta la salida del vertipuerto
    def takeoff_fp(self, parking_id, start_time, exit_heading) -> 'FlightPlan':

        parking_pad = self.pads.get(parking_id)
        if not self.main_pad or not parking_pad:
            raise ValueError("Pads no configurados")

        # Validación del vector de salida
        heading_norm = np.linalg.norm(exit_heading)
        if heading_norm == 0:
            raise ValueError("Vector de salida nulo")

        fp = FlightPlan()
        main_loc = np.array(self.main_pad.location)
        park_loc = np.array(parking_pad.location)

        #Inicio en el pad de descanso
        fp.set_waypoint(label="Stand_Start", time=start_time, pos=parking_pad.location, vel=[0, 0, 0])

        #El UAV se posiciona a la altura h1 para ir al TLOF
        t_stand_h1 = start_time + (self.h1 / self.v_vertical)
        pos_h1_stand = park_loc + [0, 0, self.h1]
        fp.set_waypoint(label="Hover_at_h1", time=t_stand_h1, pos=pos_h1_stand.tolist(), vel=[0, 0, 0])


        dist_taxi = np.linalg.norm(main_loc[:2] - park_loc[:2])
        t_at_tlof = t_stand_h1 + (dist_taxi / self.v_rod) + self.time_to_rest
        pos_tlof_taxi = main_loc + [0, 0, self.h1]
        fp.set_waypoint(label="TLOF_h1", time=t_at_tlof, pos=pos_tlof_taxi.tolist(), vel=[0, 0, 0])

        # Continúa el ascenso vertical desde h1 hasta h2 (límite del OFV)
        t_h2 = t_at_tlof + ((self.h2 - self.h1) / self.v_vertical)
        pos_h2 = main_loc + [0, 0, self.h2]
        fp.set_waypoint(label="OFV_h2_Exit", time=t_h2, pos=pos_h2.tolist(), vel=[0, 0, self.v_vertical])

        #Salida por la superficie de ascenso (pendiente categoría C)
        unit_heading = np.array(exit_heading) / heading_norm
        dist_slope = self.departure_slope_dist
        
        z_final = self.h2 + (dist_slope * self.slope_c)
        pos_final = pos_h2 + (unit_heading * dist_slope)
        pos_final[2] = z_final

        t_final = t_h2 + (dist_slope / self.v_approach)

        fp.set_waypoint(label="Departure_Slope", time=t_final, pos=pos_final.tolist(), vel=[0, 0, 0])

        # Realizar reserva del TLOF (Se reserva desde que empieza el rodaje hasta que abandona el volumen h2)
        self.main_pad.book(t_stand_h1, t_h2, self.booking_buffer)

        fp.connect_waypoints()
        
        return fp