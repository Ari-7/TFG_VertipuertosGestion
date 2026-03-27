import numpy as np
import matplotlib.pyplot as plt
from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from vertiport.vertiport_pad import Pad
from src.gestion_v import VertiportManager

def run_tests():
    # 1. configuracion del vertipuerto
    tlof = Pad(id="TLOF_MAIN", type="landing", status="active", operator_id="OP1", location=[0, 0, 0])
    
    # creamos 3 pads de descanso con distintos niveles de ocupacion previa
    s1 = Pad(id="STAND_1", type="parking", status="active", operator_id="OP1", location=[20, 20, 0])
    s2 = Pad(id="STAND_2", type="parking", status="active", operator_id="OP1", location=[-20, 20, 0])
    s3 = Pad(id="STAND_3", type="parking", status="active", operator_id="OP1", location=[0, -20, 0])

    # simulamos reservas previas para probar la estrategia consolidated (mas ocupado)
    # s1 sera el mas ocupado, s2 medio, s3 vacio
    for _ in range(5): s1.book(10000, 11000, 40) # reservas lejos del tiempo de prueba
    for _ in range(2): s2.book(10000, 11000, 40)
    
    pads_dict = {"S1": s1, "S2": s2, "S3": s3}
    manager = VertiportManager(id="V-ALB", name="Albacete", main_pad=tlof, pads=pads_dict)

    # 2. definir waypoint de entrada (uav aproximandose)
    wp_entrada = Waypoint(label="start",t=100.0, pos=[100, 100, 50], vel=[-3, -3, 0])
    heading_salida = [1, 0, 0] # salida hacia el este

    print(f"\n=== Inicio de pruebas ===")

    print("\ntest 1: condiciones ideales (esperado: exito en s1)")
    try:
        fp_exito = manager.generate_fp(wp_entrada, heading_salida, parking_duration=1200)
        if fp_exito:
            print(f"ok: plan generado. stand asignado: {fp_exito.waypoints[4].label}")
            
    except Exception as e:
        print(f"fallo inesperado en test 1: {e}")

    # tlof ocupado en landing
    print("\ntest 2: tlof bloqueado al inicio (esperado: error tlof ocupado)")
    manager.main_pad.book(100, 300, 40)
    try:
        manager.generate_fp(wp_entrada, heading_salida)
    except RuntimeError as e:
        print(f"capturado: {e}")

    print("\ntest 3: stands bloqueados (esperado: error no hay disponibilidad)")
    manager.main_pad.bookings.clear()
    for p in manager.pads.values():
        p.book(0, 5000, 40)
    try:
        manager.generate_fp(wp_entrada, heading_salida)
    except RuntimeError as e:
        print(f"capturado: {e}")

    # stands libres, tlof libre al entrar, pero ocupado justo cuando quiere salir
    print("\ntest 4: tlof bloqueado para salida (esperado: error tlof despegue)")
    for p in manager.pads.values():
        p.bookings.clear()

    manager.main_pad.book(1300, 1600, 40) 
    try:
        manager.generate_fp(wp_entrada, heading_salida, parking_duration=1200)
    except RuntimeError as e:
        print(f"capturado: {e}\n")

if __name__ == "__main__":
    run_tests()