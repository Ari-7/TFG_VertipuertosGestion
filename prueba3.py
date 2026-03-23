import numpy as np
import matplotlib.pyplot as plt
from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from vertiport.vertiport_pad import Pad
 
from src.gestion_v import VertiportManager

def get_best_pad_least_used(self, start_time, duration, buffer):
    max_search = 7200 
    step = 60        
    
    for t_offset in range(0, max_search + 1, step):
        t_start = start_time + t_offset
        t_end = t_start + duration
        
        eligible = []
        for pad in self.pads.values():
            if pad.is_available(t_start, t_end, buffer):
                eligible.append((pad, len(pad.get_bookings())))
        
        if eligible:
            # Ordenar por número de bookings (menor a mayor)
            eligible.sort(key=lambda x: x[1])
            best_pad, num = eligible[0]
            return best_pad, t_start, num
    return None

def get_best_pad_consolidated(self, start_time, duration, buffer):
    max_search = 7200
    step = 60
    
    for t_offset in range(0, max_search + 1, step):
        t_start = start_time + t_offset
        t_end = t_start + duration
        
        eligible = []
        for pad in self.pads.values():
            if pad.is_available(t_start, t_end, buffer):
                eligible.append((pad, len(pad.get_bookings())))
        
        if eligible:
            # Ordenar por número de bookings (mayor a menor)
            eligible.sort(key=lambda x: x[1], reverse=True)
            best_pad, num = eligible[0]
            return best_pad, t_start, num
    return None

manager = VertiportManager(id="V-ALB", name="Albacete Vertiport")
manager.main_pad = Pad("TLOF", "landing", "active", "OP1", (0, 0, 0))


p1 = Pad("STAND_1", "parking", "active", "OP1", (20, 0, 0))
p2 = Pad("STAND_2", "parking", "active", "OP1", (0, 20, 0))
p3 = Pad("STAND_3", "parking", "active", "OP1", (-20, 0, 0))

# Escenario de ocupación:
# STAND_1: 
p1.bookings.add((0, 1000)); p1.bookings.add((2000, 3000)); p1.bookings.add((4000, 5000))
# STAND_2:
p2.bookings.add((0, 1500))
# STAND_3: 
p3.bookings.add((0, 1500))
# (p3 está vacío)

manager.pads = {"S1": p1, "S2": p2, "S3": p3}

def run_test(strategy_name):
    print(f"\n--- TEST: ESTRATEGIA {strategy_name.upper()} ---")
    t_deseado = 900 
    duracion = 3600 
    buffer = 40
    
    if strategy_name == "least_used":
        res = get_best_pad_least_used(manager, t_deseado, duracion, buffer)
    else:
        res = get_best_pad_consolidated(manager, t_deseado, duracion, buffer)
        
    if res:
        pad, t_final, n = res
        print(f"Resultado: Seleccionado {pad.id}")
        print(f"Tiempo de inicio: {t_final} (Deseado: {t_deseado})")
        print(f"Reservas previas en ese Pad: {n}")
    else:
        print("No se encontró pad disponible.")

# Ejecución de las pruebas
run_test("least_used")    # Debería elegir STAND_3 (0 bookings)
run_test("consolidated")  # Debería elegir STAND_1 (3 bookings)

print("\n--- TEST EXTRA: STAND_3 ocupado en T=6000 ---")
p3.bookings.add((5500, 7000)) 
run_test("least_used")    # Debería elegir STAND_2 (tiene 1 booking, menos que S1)