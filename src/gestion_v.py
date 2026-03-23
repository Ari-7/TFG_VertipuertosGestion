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

    def get_pad_menos(self, buffer, start_time, tiempo_rest):

        elegible_pads = []
        max_tiempo = 600
        step = 30
        best = None

        for t in range(0, max_tiempo +1, step):

            current_start = start_time + t
            current_end = start_time + tiempo_rest
        
            for pad_id, pad in self.pads.items():

                if pad.is_available(current_start, current_end, buffer):

                    num_book = len(pad.get_bookings())

                    if best is None or num_book < best[2]:
                        best = (pad, current_start, num_book)

                    elegible_pads.append((pad, len(pad.get_bookings())))

            if best and best[2] == 0: 
                break

        return best

    def get_pad_mas(self, buffer, start_time, tiempo_rest):

        elegible_pads = []
        max_tiempo = 600
        step = 30
        best = None

        for t in range(0, max_tiempo +1, step):

            current_start = start_time + t
            current_end = start_time + tiempo_rest
        
            for pad_id, pad in self.pads.items():

                if pad.is_available(current_start, current_end, buffer):

                    num_book = len(pad.get_bookings())

                    if best is None or num_book > best[2]:
                        best = (pad, current_start, num_book)

                    elegible_pads.append((pad, len(pad.get_bookings())))

            if best: 
                break

        return best

    #Función para aterrizaje, se asume que se le pasa el estado del vertipuerto y el wp inicial (por el que termina su ruta)
    def landing_fp(self, initial_wp, strategy="least_used") -> 'FlightPlan':

        if not self.main_pad or not self.pads:
            raise ValueError("Pads no configurados")

        main_loc = np.array(self.main_pad.location)
        dist_total = np.linalg.norm(np.array(initial_wp.pos) - main_loc)
        
        # Tiempos estimados de maniobra
        t_approach = dist_total / self.v_approach
        t_descent = (self.h2 - self.h1) / self.v_vertical
        t_arrival_at_tlof = t_approach + t_descent
        
        # Duración de la estancia en el Parking
        parking_duration = self.default_booking_time
        
        #Búsqueda disponibilidad
        t_attempt = initial_wp.t
        parking_pad = None
        found_slot = False

        while not found_slot and t_attempt < initial_wp.t + 7200:
            # Se reserva desde que empieza la aproximación hasta que despeja hacia el parking
            t_end_tlof = t_attempt + t_arrival_at_tlof + self.time_to_rest
            
            if self.main_pad.is_available(t_attempt, t_end_tlof, self.booking_buffer):
                
                if strategy == "least_used":
                    parking_pad = self.get_best_pad_least_used_at_time(t_end_tlof, parking_duration, self.booking_buffer)
                else:
                    parking_pad = self.get_best_pad_consolidated_at_time(t_end_tlof, parking_duration, self.booking_buffer)
                
                if parking_pad:
                    found_slot = True
                    break
            
            t_attempt += 60.0

        if not found_slot:
            raise RuntimeError("No se encontró un slot libre (TLOF + Parking) en las próximas 2 horas")

        # Una vez encontrado el hueco, realizamos las reservas reales
        t_start_parking = t_attempt + t_arrival_at_tlof + self.time_to_rest
        t_final_parking = t_start_parking + parking_duration
        
        self.main_pad.book(t_attempt, t_start_parking, self.booking_buffer, availability_checked=True)
        parking_pad.book(t_start_parking, t_final_parking, self.booking_buffer, availability_checked=True)

        # Procedemos a crear el FlightPlan con t_attempt (el tiempo de inicio real)
        fp = FlightPlan()
        # ... resto de la lógica de waypoints ...

        #Punto de aproximación
        fp.set_waypoint(initial_wp, label="start") #quitar el label después de ver si funciona

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
        
        pos_taxi = park_loc + [0, 0, self.taxi_height]
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

        #Movimiento al TLOF (rodaje)
        # El UAV se mueve horizontalmente desde el Stand hasta la vertical del TLOF a la altura de taxi
        dist_taxi = np.linalg.norm(main_loc[:2] - park_loc[:2])
        t_at_tlof = start_time + (dist_taxi / self.v_rod) + self.time_to_rest
        pos_tlof_taxi = main_loc + [0, 0, self.taxi_height]
        fp.set_waypoint(label="Ready_at_TLOF", time=t_at_tlof, pos=pos_tlof_taxi.tolist(), vel=[0, 0, 0])

        #Ascenso vertical OFV (h1 y h2)
        t_h1 = t_at_tlof + ((self.h1 - self.taxi_height) / self.v_vertical)
        pos_h1 = main_loc + [0, 0, self.h1]
        fp.set_waypoint(label="OFV_h1", time=t_h1, pos=pos_h1.tolist(), vel=[0, 0, self.v_vertical])

        # Continúa el ascenso vertical desde h1 hasta h2 (límite del OFV)
        t_h2 = t_h1 + ((self.h2 - self.h1) / self.v_vertical)
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
        self.main_pad.book(start_time, t_h2, self.booking_buffer)

        fp.connect_waypoints()
        
        return fp