import math
import matplotlib.pyplot as plt
import numpy as np

# =========================================================
# FUNCIÓN PRINCIPAL DE ACKERMANN (La que vas a importar)
# =========================================================
def calcular_ackermann(radio_giro, L=26.0, W=17.0):
    """
    Calcula los ángulos exactos para los servomotores basándose en la 
    geometría de Ackermann para un coche escala 1:10.
    
    Parámetros:
    - radio_giro: El radio de la curva que quieres dar (en cm). 
                  Positivo para la derecha, negativo para la izquierda.
    - L: Batalla (Wheelbase) en cm. (Por defecto 26.0 cm)
    - W: Ancho de Vía (Track Width) en cm. (Por defecto 17.0 cm)
    
    Retorna:
    - (angulo_llanta_interior, angulo_llanta_exterior) en grados.
    """
    # Protección: Si el radio es 0 o es una línea recta
    if radio_giro == 0 or abs(radio_giro) > 10000:
        return 0.0, 0.0
        
    # Protección: El radio no puede ser más pequeño que la mitad del coche
    # de lo contrario la matemática se rompe (singularidad)
    if abs(radio_giro) <= W / 2:
        # Forzamos al límite físico más cerrado posible
        radio_giro = (W / 2) + 0.1 if radio_giro > 0 else -(W / 2) - 0.1

    # Fórmulas de Ackermann en Radianes
    angulo_interior_rad = math.atan(L / (abs(radio_giro) - (W / 2)))
    angulo_exterior_rad = math.atan(L / (abs(radio_giro) + (W / 2)))

    # Convertimos a Grados para mandarlo a tu placa PCA9685 o Servo
    angulo_interior_deg = math.degrees(angulo_interior_rad)
    angulo_exterior_deg = math.degrees(angulo_exterior_rad)

    # Si el giro es a la izquierda (negativo), invertimos los signos
    if radio_giro < 0:
        return -angulo_interior_deg, -angulo_exterior_deg

    return angulo_interior_deg, angulo_exterior_deg


# =========================================================
# BLOQUE DE VISUALIZACIÓN (Puedes correr este archivo directo)
# =========================================================
if __name__ == "__main__":
    print("Iniciando prueba de función Ackermann...")
    
    # Prueba de consola rápida
    radio_prueba = 50.0 # Curva de 50 cm
    ang_int, ang_ext = calcular_ackermann(radio_prueba)
    print(f"Para dar una vuelta con un radio de {radio_prueba} cm:")
    print(f" -> La llanta INTERIOR debe girar: {ang_int:.2f}°")
    print(f" -> La llanta EXTERIOR debe girar: {ang_ext:.2f}°")
    print("-" * 40)
    
    # --- Generación de Gráfica ---
    # Simulamos pedirle al coche que haga radios de giro desde los 20 cm (muy cerrado) 
    # hasta los 150 cm (curva abierta)
    radios = np.linspace(20, 150, 200)
    angulos_int = []
    angulos_ext = []

    for r in radios:
        a_i, a_e = calcular_ackermann(r)
        angulos_int.append(a_i)
        angulos_ext.append(a_e)

    # Dibujar la gráfica
    plt.figure(figsize=(10, 6))
    plt.plot(radios, angulos_int, label='Llanta Interior (Gira más)', color='#e74c3c', linewidth=2.5)
    plt.plot(radios, angulos_ext, label='Llanta Exterior (Gira menos)', color='#3498db', linewidth=2.5)
    
    # Estilos de la gráfica
    plt.title('Comportamiento de la Geometría Ackermann\n(Escala 1:10 - Chasis DKS-Pro)', fontsize=14, fontweight='bold')
    plt.xlabel('Radio de Giro Solicitado por la IA (cm)', fontsize=12)
    plt.ylabel('Ángulo del Servomotor (Grados)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(fontsize=12)
    
    # Guardar o mostrar
    plt.tight_layout()
    plt.savefig('ackermann_dks_pro.png')
    print("Gráfica guardada exitosamente como 'ackermann_dks_pro.png'")
    # plt.show() # Descomenta esto si lo corres en tu laptop ASUS para que abra la ventana.