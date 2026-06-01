import math
import random
import numpy as np
from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from vertiport.vertiport_pad import Pad
 
from src.gestion_v import VertiportManager

def estudio_productividad(num_solicitudes=30, intervalo_llegada=180):
    
    print("\n" + "="*70)
    print(f"{'ANÁLISIS DE CAPACIDAD OPERATIVA':^70}")
    print("="*70)

    #Configuración infraestructura

    tlof = Pad("TLOF_1", "TLOF", "Active", "OP_1", (0.0, 0.0, 0.0))
    stands = {
        f"S{i}": Pad(f"S_{i}", "STAND", "Active", "OP_1", (15.0 * i, 0.0, 0.0))
        for i in range(1, 4)
    }

    manager = VertiportManager(id="V_Capacidad", name="Estudio TFG", main_pad=tlof, pads=stands)

    exitos = 0
    rechazos = 0
    t_acumulado = 0.0

    print(f"{'ID':<4} | {'T. Entrada':<10} | {'Estado':<12} | {'Causa de Rechazo'}")
    print("-" * 70)

    for i in range(num_solicitudes):
        
        #Tiempo de llegada aleatorio
        t_acumulado += random.randint(int(intervalo_llegada*0.7), int(intervalo_llegada*1.3))
        
        wp_in = Waypoint(
            f"Dron_{i}_In", 
            float(t_acumulado), 
            [100.0, 100.0, 60.0], 
            [0.0, 0.0, 0.0]
        )
        
        wp_out = Waypoint(
            f"Dron_{i}_Out", 
            float(t_acumulado + 3000), 
            [200.0, 200.0, 60.0], 
            [0.0, 0.0, 0.0]
        )

        try:
            manager.generate_fp(wp_in, wp_out, parking_duration=1200)
            
            # \033[92m verde
            print(f"{i+1:<4} | {t_acumulado:<10.1f} | \033[92mACEPTADO\033[0m    | -")
            exitos += 1

        except RuntimeError as e:
            # \033[91m rojo
            print(f"{i+1:<4} | {t_acumulado:<10.1f} | \033[91mRECHAZADO\033[0m   | {str(e)}")
            rechazos += 1

    # Resumen estadísticas
    print("-" * 70)
    print(f"Peticiones Totales: {num_solicitudes}")
    print(f"Éxitos: {exitos} | Rechazos: {rechazos}")
    print(f"Productividad (Throughput): {(exitos/num_solicitudes)*100:.2f}%")
    print("="*70)

if __name__ == "__main__":
    estudio_productividad(num_solicitudes=30, intervalo_llegada=250)