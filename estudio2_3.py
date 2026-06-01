import math
import random
import numpy as np
from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from vertiport.vertiport_pad import Pad
 
from src.gestion_v import VertiportManager


def simular_geometria(tipo_diseno, distancia_base=15.0):

    tlof = Pad("TLOF_1", "TLOF", "Active", "OP_1", (0.0, 0.0, 0.0))
    
    stands = {}
    if tipo_diseno == "linea":

        stands = {f"S{i}": Pad(f"S_{i}", "STAND", "Active", "OP_1", (distancia_base * i, 0.0, 0.0)) for i in range(1, 4)}

    elif tipo_diseno == "estrella":

        stands = {
            "S1": Pad("S_1", "STAND", "Active", "OP_1", (distancia_base, 0.0, 0.0)),
            "S2": Pad("S_2", "STAND", "Active", "OP_1", (0.0, distancia_base, 0.0)),
            "S3": Pad("S_3", "STAND", "Active", "OP_1", (-distancia_base, 0.0, 0.0))
        }
        
    manager = VertiportManager(id="V_GEO", name="Estudio Geo", main_pad=tlof, pads=stands)
    
    exitos = 0
    t_acumulado = 0.0
    for i in range(30):
        t_acumulado += random.randint(175, 325)
        wp_in = Waypoint(f"D_In_{i}", float(t_acumulado), [100.0, 100.0, 50.0], [0.0, 0.0, 0.0])
        
        t_salida_estimada = float(t_acumulado + 3000)
        wp_out = Waypoint(f"D_Out_{i}", t_salida_estimada, [200.0, 200.0, 60.0], [0.0, 0.0, 0.0])

        try:
            manager.generate_fp(wp_in, wp_out, parking_duration=1200)
            exitos += 1
        except RuntimeError:
            pass
            
    return (exitos / 30) * 100


def ratio_optimo():
    
    resultados = {}
    
    for num_stands in range(1, 11):
        tlof = Pad("TLOF_1", "TLOF", "Active", "OP_1", (0.0, 0.0, 0.0))
        
        stands = {
            f"S{i}": Pad(f"S_{i}", "STAND", "Active", "OP_1", (15.0, 5.0 * i, 0.0))
            for i in range(1, num_stands + 1)
        }
        
        manager = VertiportManager(id="V_RATIO", name="Estudio Ratio", main_pad=tlof, pads=stands)
        
        exitos = 0
        t_acumulado = 0.0
        
        for i in range(50):

            t_acumulado += random.randint(100, 200)
            wp_in = Waypoint(f"D_{i}", float(t_acumulado), [100.0, 100.0, 50.0], [0.0, 0.0, 0.0])
            
            t_salida_estimada = float(t_acumulado + 3000)
            wp_out = Waypoint(f"D_{i}_Out", t_salida_estimada, [200.0, 200.0, 60.0], [0.0, 0.0, 0.0])
            
            try:
                manager.generate_fp(wp_in, wp_out, parking_duration=1200)
                exitos += 1
            except RuntimeError:
                pass
                
        throughput = (exitos / 50) * 100
        resultados[num_stands] = throughput
        print(f"Stands: {num_stands} -> Productividad: {throughput:.2f}%")
        
    return resultados

if __name__ == "__main__":

    print("\n--- Geometría: linea ---")
    resultado_linea = simular_geometria("linea", distancia_base=15.0)
    print(f"Productividad en linea: {resultado_linea:.2f}%")

    print("\n--- Geometría: estrella ---")
    resultado_estrella = simular_geometria("estrella", distancia_base=15.0)
    print(f"Productividad en estrella: {resultado_estrella:.2f}%")

    print("\n--- Ratio óptimo (1 a 10 stands) ---")
    resultados_ratio = ratio_optimo()