import numpy as np
from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint

from src.gestion_v import VertiportManager
from vertiport.vertiport_pad import Pad

def test_vertiport_operations():
    
    tlof = Pad(id="TLOF_02", type="TLOF", status="active", operator_id="OP02", location=(0, 0, 0))
    parking = Pad(id="STAND_02", type="STAND", status="active", operator_id="OP02", location=(100, 0, 0))

    operator = VertiportManager(
        id="VPT_MAD_01", 
        name="Madrid Central", 
        main_pad=tlof, 
        pads={"STAND_02": parking}
    )
    """
   
    entry_wp = Waypoint(
        label="Entry",
        t=100.0,
        pos=[80, 0, 40.5],
        vel=[5, 0, 0]
    )
    
    lp = operator.landing_fp(
        initial_wp=entry_wp, 
        parking_id="STAND_01"
    )
"""

    wp_entrada_recta = Waypoint(
        label="Entrada_recta",
        t=10.0,              # Tiempo inicial
        pos=[-600, 0, 600], 
        vel=[10, 0, -1] 
    )
    
    fp_landing = operator.landing_fp(
        initial_wp=wp_entrada_recta, 
        parking_id="STAND_02"
    )
    """
    print("Generando Plan de Despegue...")
    tp = operator.takeoff_fp(
        uav_id="Drone_01", 
        start_time=500.0, 
        parking_id="STAND_01", 
        exit_heading=[1, 1] # Hacia el Noreste
    )
"""
    if isinstance(fp_landing, FlightPlan):
        print("Plan generados correctamente.")
        
        # Tabla de waypoints para verificar tiempos y alturas h1 y h2
        print("\nWaypoints de Aterrizaje:")
        #lp.print_waypoints() 
        #lp.position_figure("FP1", 1)

        fp_landing.print_waypoints()
        fp_landing.position_figure("FP2", 2)
    else:
        print("X Error en la generación:", fp_landing)

if __name__ == "__main__":
    test_vertiport_operations()