import math
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import mplcursors

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


def ratio_optimo_single(num_stands):
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
            
    return (exitos / 50) * 100


def ejecutar_estudio_geometria(n_ejecuciones=20):
    res_linea = [simular_geometria("linea") for _ in range(n_ejecuciones)]
    res_estrella = [simular_geometria("estrella") for _ in range(n_ejecuciones)]
    
    return {
        "linea": {"media": np.mean(res_linea), "std": np.std(res_linea)},
        "estrella": {"media": np.mean(res_estrella), "std": np.std(res_estrella)}
    }


def ejecutar_estudio_ratio(n_ejecuciones=20, max_stands=10):
    resultados = {}
    for num_stands in range(1, max_stands + 1):
        corridas = [ratio_optimo_single(num_stands) for _ in range(n_ejecuciones)]
        resultados[num_stands] = {
            "media": np.mean(corridas),
            "std": np.std(corridas)
        }
    return resultados

def graficar_resultados(geo_data, ratio_data):
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))

    # Gráfico de geometrías
    geometrias = ['Línea', 'Estrella']
    medias_geo = [geo_data['linea']['media'], geo_data['estrella']['media']]
    stds_geo = [geo_data['linea']['std'], geo_data['estrella']['std']]

    bars = axes[0].bar(geometrias, medias_geo, yerr=stds_geo, capsize=6, color=['#2b5c8f', '#d95f02'], alpha=0.85, width=0.4)
    axes[0].set_title("Productividad Promedio por Geometría (N=20)", fontsize=12, fontweight='bold')
    axes[0].set_ylabel("Productividad Media (%)")
    axes[0].set_ylim(0, 105)

    for bar in bars:
        height = bar.get_height()
        axes[0].annotate(f'{height:.1f}%',
                         xy=(bar.get_x() + bar.get_width() / 2, height / 2),
                         ha='center', va='center', color='white', fontweight='bold')

    #Gráfico de Curva de Stands
    stands = list(ratio_data.keys())
    medias_ratio = [ratio_data[s]['media'] for s in stands]
    stds_ratio = [ratio_data[s]['std'] for s in stands]

    linea_puntos, = axes[1].plot(stands, medias_ratio, marker='o', color='#2ca02c', linewidth=2, label='Media')
    
    axes[1].fill_between(stands, 
                         np.array(medias_ratio) - np.array(stds_ratio), 
                         np.array(medias_ratio) + np.array(stds_ratio), 
                         color='#2ca02c', alpha=0.2, label='±1 Desv. Estándar')

    axes[1].set_title("Curva de Productividad según Número de Stands (N=20)", fontsize=12, fontweight='bold')
    axes[1].set_xlabel("Número de Stands")
    axes[1].set_ylabel("Productividad Media (%)")
    axes[1].set_xticks(stands)
    axes[1].set_ylim(0, 105)
    axes[1].legend(loc='lower right')

    cursor = mplcursors.cursor(linea_puntos, hover=True)
    
    # Personalizar la etiqueta 
    @cursor.connect("add")
    def on_add(sel):
        x_val = int(sel.target[0])
        y_val = sel.target[1]
        std_val = ratio_data[x_val]['std']
        sel.annotation.set_text(f"Stands: {x_val}\nProd. Media: {y_val:.2f}%\nDesv. Est.: ±{std_val:.2f}%")
        sel.annotation.get_bbox_patch().set(fc="white", alpha=0.9, ec="#2ca02c")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    N_REPETICIONES = 20
    print(f"Ejecutando simulaciones ({N_REPETICIONES} iteraciones por prueba)...")

    # Ejecutar pruebas iteradas
    geo_res = ejecutar_estudio_geometria(n_ejecuciones=N_REPETICIONES)
    ratio_res = ejecutar_estudio_ratio(n_ejecuciones=N_REPETICIONES, max_stands=10)

    # Mostrar resumen estructurado en tabla por pantalla
    df_ratio = pd.DataFrame.from_dict(ratio_res, orient='index')
    df_ratio.columns = ['Productividad Media (%)', 'Desv. Estándar']
    print("\n--- RESUMEN DE RATIO ÓPTIMO ---")
    print(df_ratio.round(2))

    # Generar visualización
    graficar_resultados(geo_res, ratio_res)