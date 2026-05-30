# Calibración del Simulador Sim2Real (Unity ↔ PC)

Estado guardado el 2026-05-25. Esta es la configuración del **simulador**.
Cuando llegue la calibración del **carro físico real**, comparar y ajustar
estos valores para que Unity replique el comportamiento real.

---

## 1. Cámara del vehículo (Unity — `VehicleBuilder.cs`)

| Parámetro | Valor actual (sim) | Notas |
|-----------|--------------------|-------|
| Posición local | `(0, 0.45, 0.45)` | Adelante del carro (no la tapa el modelo) y alta |
| Rotación | `Euler(30, 0, 0)` | 30° hacia abajo |
| FOV | `75` | Amplio para ver las 2 líneas del carril |
| Near / Far | `0.01 / 20` | |
| Fondo | gris `#8A8A8A` | No blanco (evita "pantalla blanca") |
| cullingMask | `~0` (todo) | El carro no estorba porque la cámara va adelante |
| RenderTexture | `320×240` (`CameraRT`) | Se reescala a 640×480 en el PC |

**Aprendizaje clave:** el modelo 3D del carro tapaba la cámara. Solución:
cámara ADELANTE del carro. La cámara debe MIRAR ABAJO (no horizontal) o ve
el cielo.

---

## 2. Pista (Unity — `SceneBuilder.cs`)

| Elemento | Valor |
|----------|-------|
| Largo de pista | `30 m` |
| Ancho de carril (línea a línea) | `54 cm` → líneas en `x = ±0.27` |
| Suelo | gris `#6E6E6E` (oscuro, contrasta con líneas blancas) |
| Líneas (izq/der/central) | **blancas** `#FFFFFF` |
| Central punteada | blanca, segmentos cada `0.8 m` |
| STOP | a la mitad (`z = 15 m`), derecha (`x = 0.32`), imagen `Resources/Signs/stop.png` |

---

## 3. Filtro de carril HSV (`vision/lane_pipeline.py`)

```python
HSV_WHITE_LO = [0,  0, 200]   # blanco MUY brillante
HSV_WHITE_HI = [179, 40, 255] # MUY desaturado (rechaza gris del entorno)
```

## 4. Bird's-Eye View (BEV) — SOLO simulador (en `main_simulator.py`)

```python
roi_frac = 0.25
bev_src_ratio = [
    [0.15, 1.00],  # abajo-izquierda
    [0.85, 1.00],  # abajo-derecha
    [0.60, 0.32],  # arriba-derecha
    [0.40, 0.32],  # arriba-izquierda
]
```
> El Pi real usa los defaults de `LanePipeline` (otra cámara). Estos valores
> son exclusivos del simulador.

## 5. Sesgo de carril

```python
RIGHT_BIAS = 0.70   # 0.5=centro, 1.0=línea derecha. TMR va por la derecha.
```

## 6. Dirección (servo) — INVERSIÓN

El servo del carro físico está montado al revés. El simulador replica esto:
```python
# MockSteeringDriver
STEERING_INVERTED = True
physical = 2*90 - angle_logico   # se envía a Unity el ángulo físico
```
**Sin esta inversión el carro gira al lado equivocado** (se va a la línea
izquierda en vez del carril derecho).

## 7. PID de dirección (error de carril → ángulo servo)

```python
PID_KP = 0.08
PID_KI = 0.002
PID_KD = 0.025
```

## 8. FSM — parada en STOP

| Parámetro | Valor |
|-----------|-------|
| `SIGN_BBOX_STOP_MM` | 320 mm (frena por cámara cuando la señal está cerca) |
| `STOP_TARGET_MM` | 270 mm (regla TMR: 270 ± 30) |
| `ESPERA_S` | 5.0 s |
| Altura real octágono STOP | `0.04 m` (4 cm medido) |
| `CAMERA_FOCAL_LENGTH_PX` | 490 |

---

## CÓMO REPLICAR EL CARRO FÍSICO EN UNITY

Cuando tengas la foto/datos del carro físico:
1. **Cámara**: medir altura real (cm) y ángulo de inclinación → ajustar
   `cameraMount.localPosition.y` y `Euler(x,...)` para que coincida.
2. **Carril**: confirmar ancho real (¿54 cm?) → ajustar `x` de las líneas.
3. **Vista BEV**: comparar la vista del Pi vs la de Unity; ajustar
   `bev_src_ratio` hasta que las líneas queden verticales en el ojo de águila.
4. **Colores**: igualar el gris de la pista y el brillo de las líneas al real.
5. Verificar que `err` (px) tenga el mismo signo y magnitud en sim y real.
