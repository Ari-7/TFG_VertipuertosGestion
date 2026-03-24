import numpy as np
import matplotlib.pyplot as plt
from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from vertiport.vertiport_pad import Pad
 
from src.gestion_v import VertiportManager

def run_visual_tests():

    tlof_pad = Pad(id="TLOF_01", type="TLOF", status="FREE", operator_id="OP01", location=(0.0, 0.0, 0.0))
    stand_pad = Pad(id="STAND_01", type="STAND", status="FREE", operator_id="OP01", location=(50, 15, 0.0))
    
    v_manager = VertiportManager(
        id="VM_01", 
        main_pad=tlof_pad, 
        pads={"STAND_01": stand_pad}
    )

    print("\n" + "="*50)
    print("LANDING")
    print("="*50)

    try:
        
        wp_start = Waypoint(label="Approach_Start", t=10.0, pos=[-70, 0.0, 70])
        
        # Generar plan
        fp_landing = v_manager.landing_fp( wp_start, strategy="last_used")
    
        fp_landing.print_waypoints()
        
        fp_landing.position_figure("F1", 1)
        
    except Exception as e:
        print(f"Error en Test de Landing: {e}")

    print("\n" + "="*50)
    print("TAKE-OFF")
    print("="*50)

    try:

        exit_heading = [-1.0, 1.0, 0.0]
        
        # Generar plan (iniciando después del aterrizaje)
        fp_takeoff = v_manager.takeoff_fp("STAND_01", start_time=500.0, exit_heading=exit_heading)
        
        fp_takeoff.print_waypoints()
        
        fp_takeoff.position_figure("F2", 1)
        
    except Exception as e:
        print(f"Error en Test de Takeoff: {e}")

    plt.show()

if __name__ == "__main__":
    run_visual_tests()