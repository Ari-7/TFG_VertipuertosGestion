import numpy as np
import matplotlib.pyplot as plt
from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from vertiport.vertiport_pad import Pad
 
from src.gestion_v import VertiportManager
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

manager = VertiportManager(id="V-ALB", name="Albacete Vertiport")
manager.main_pad = Pad("TLOF", "landing", "active", "OP1", (0, 0, 0))


p1 = Pad("STAND_1", "parking", "active", "OP1", (20, 0, 0))
p2 = Pad("STAND_2", "parking", "active", "OP1", (0, 20, 0))
p3 = Pad("STAND_3", "parking", "active", "OP1", (-20, 0, 0))

# STAND_1: 
p1.bookings.add((0, 1000)); p1.bookings.add((2000, 3000)); p1.bookings.add((4000, 5000))
# STAND_2:
p2.bookings.add((0, 1500))
# STAND_3: 
#p3.bookings.add((0, 1500))

manager.pads = {"S1": p1, "S2": p2, "S3": p3}

def run_test(strategy):
    print(f"\n--- TEST: ESTRATEGIA {strategy.upper()} ---")
    t_deseado = 6000
    duracion = 3600
    buffer = 40
    
    if strategy == "least_used":
        res = get_pad(manager, t_deseado, duracion, buffer, strategy)
    else:
        res =  get_pad(manager, t_deseado, duracion, buffer, strategy)
        
    if res:
        pad, t_final, n = res
        print(f"Resultado: Seleccionado {pad.id}")
        print(f"Tiempo de inicio: {t_final} (Deseado: {t_deseado})")
        print(f"Reservas previas en ese Pad: {n}")
    else:
        print("No se encontró pad disponible.")

# Ejecución de las pruebas
run_test("least_used")    
run_test("most_used")  

print("\n--- TEST EXTRA: STAND_3 ocupado en T=6000 ---")
p3.bookings.add((5500, 7000)) 
run_test("least_used") 