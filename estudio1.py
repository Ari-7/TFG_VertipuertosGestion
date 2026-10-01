import math
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import mplcursors
from collections import Counter

from flight_plan.flight_plan import FlightPlan
from flight_plan.waypoint import Waypoint
from vertiport.vertiport_pad import Pad
from src.gestion_v import VertiportManager


def graficar_productividad(num_solicitudes, exitos, rechazos, causas_counter):
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))

    #Gráfico de Donut
    etiquetas_est = [f'Aceptados\n({exitos})', f'Rechazados\n({rechazos})']
    valores_est = [exitos, rechazos]
    colores_est = ['#2ca02c', '#d62728']

    wedges, texts, autotexts = axes[0].pie(
        valores_est, 
        labels=etiquetas_est, 
        autopct='%1.1f%%',
        startangle=140,
        colors=colores_est,
        pctdistance=0.75,
        textprops=dict(color="black", fontweight='bold')
    )
    
    centre_circle = plt.Circle((0, 0), 0.50, fc='white')
    axes[0].add_artist(centre_circle)
    axes[0].set_title(f"Estado de Solicitudes (Total: {num_solicitudes})", fontsize=12, fontweight='bold')

    #Gráfico de Causas de Rechazo
    cursor_bars = None
    if rechazos > 0 and len(causas_counter) > 0:
        causas_nombres = list(causas_counter.keys())
        causas_cantidades = list(causas_counter.values())
        pcts_sobre_rechazos = [(c / rechazos) * 100 for c in causas_cantidades]

        y_pos = np.arange(len(causas_nombres))
        bars = axes[1].barh(y_pos, pcts_sobre_rechazos, color='#1f77b4', alpha=0.85, height=0.5)
        
        axes[1].set_yticks(y_pos)
        axes[1].set_yticklabels(causas_nombres)
        axes[1].invert_yaxis()
        axes[1].set_xlabel("Porcentaje sobre el Total de Rechazos (%)", fontweight='bold')
        axes[1].set_title("Distribución por Causa de Rechazo", fontsize=12, fontweight='bold')
        axes[1].set_xlim(0, 105)

        for bar, cant, pct in zip(bars, causas_cantidades, pcts_sobre_rechazos):
            width = bar.get_width()
            axes[1].annotate(f'{pct:.1f}% ({cant})',
                             xy=(width, bar.get_y() + bar.get_height() / 2),
                             xytext=(5, 0),
                             textcoords="offset points",
                             ha='left', va='center', fontweight='bold')

        #Cursor
        cursor_bars = mplcursors.cursor(bars, hover=True)
        @cursor_bars.connect("add")
        def on_hover_bars(sel):
            idx = sel.index
            causa = causas_nombres[idx]
            cant = causas_cantidades[idx]
            pct_rech = pcts_sobre_rechazos[idx]
            pct_total = (cant / num_solicitudes) * 100
            
            sel.annotation.set_text(
                f"Causa: {causa}\n"
                f"Cantidad: {cant} casos\n"
                f"% sobre rechazos: {pct_rech:.1f}%\n"
                f"% sobre total vuelos: {pct_total:.1f}%"
            )
            sel.annotation.get_bbox_patch().set(fc="white", alpha=0.9, ec="#1f77b4")
    else:
        axes[1].text(0.5, 0.5, "100% de Éxito\nNo hubo rechazos", 
                     ha='center', va='center', fontsize=14, color='#2ca02c', fontweight='bold')
        axes[1].set_title("Distribución por Causa de Rechazo", fontsize=12, fontweight='bold')
        axes[1].axis('off')

    def on_right_click(event):
        if event.button == 3 and cursor_bars is not None:
            for sel in list(cursor_bars.selections):
                cursor_bars.remove_selection(sel)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_right_click)

    plt.tight_layout()
    plt.show()

def estudio_productividad(num_solicitudes=70, intervalo_llegada=250):
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
    causas_counter = Counter()
    t_acumulado = 0.0

    print(f"{'ID':<4} | {'T. Entrada':<10} | {'Estado':<12} | {'Causa de Rechazo'}")
    print("-" * 70)

    for i in range(num_solicitudes):
        t_acumulado += random.randint(int(intervalo_llegada * 0.7), int(intervalo_llegada * 1.3))
        
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
            print(f"{i+1:<4} | {t_acumulado:<10.1f} | \033[92mACEPTADO\033[0m    | -")
            exitos += 1

        except RuntimeError as e:
            causa_msg = str(e)
            causas_counter[causa_msg] += 1
            print(f"{i+1:<4} | {t_acumulado:<10.1f} | \033[91mRECHAZADO\033[0m   | {causa_msg}")
            rechazos += 1

    #Resumen estadísticas
    print("-" * 70)
    print(f"Peticiones Totales: {num_solicitudes}")
    print(f"Éxitos: {exitos} | Rechazos: {rechazos}")
    print(f"Productividad (Throughput): {(exitos/num_solicitudes)*100:.2f}%")
    print("="*70)

    graficar_productividad(num_solicitudes, exitos, rechazos, causas_counter)


if __name__ == "__main__":
    estudio_productividad(num_solicitudes=70, intervalo_llegada=250)