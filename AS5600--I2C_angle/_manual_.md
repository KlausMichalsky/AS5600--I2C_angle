# Mini-Manual: Lectura de Ángulo del Sensor AS5600 usando Wire1 en RP2040

## Descripción general
Este programa permite leer el **ángulo absoluto de un sensor AS5600** utilizando el bus **I2C secundario (Wire1)** del RP2040.  
El valor leído se obtiene como un número de 12 bits (0–4095), el cual se convierte a grados (0–360°) y se envía vía Serial.

Es ideal para proyectos de **robot de ajedrez**, control de ejes, motores, cinemática o cualquier sistema que requiera medición de ángulo precisa y estable.

---

## Pines y conexiones

| Función AS5600 | RP2040 (Zero / Pico) |
|---------------|-----------------------|
| SDA           | GP26 (Wire1 SDA)      |
| SCL           | GP27 (Wire1 SCL)      |
| VCC           | 3.3V                  |
| GND           | GND                   |

⚠️ El AS5600 funciona a **3.3V**, nunca conectarlo a 5V.  
⚠️ Mantener cables cortos o usar ferrite bead si hay ruido.

---

## Librerías utilizadas
- **Wire.h** → Comunicación I2C.
- Hardware RP2040 → Permite usar un segundo bus I2C (`Wire1`).

---

## Variables y valores clave

| Elemento | Descripción |
|----------|-------------|
| `AS5600_ADDR` | Dirección fija del sensor (0x36). |
| Registros `0x0E–0x0F` | Contienen el ángulo en formato 12 bits. |
| Rango crudo | 0–4095 (0–360°). |
| Conversión | `degrees = angle * 360.0 / 4096.0`. |

---

## Flujo de ejecución

### 1. Setup
- Inicia Serial a 115200.
- Configura `Wire1` para usar **GP26/GPIO27**.
- Realiza `Wire1.begin()` para activar el bus.
- Muestra mensaje indicando que el AS5600 está listo.

### 2. Loop
- Llama a `readAS5600Angle()` para obtener el valor crudo.
- Convierte el ángulo a grados.
- Envía el resultado al puerto Serial.
- Espera 200 ms antes de la siguiente lectura.

---

## Función `readAS5600Angle()`
1. Envía al AS5600 la dirección del registro 0x0E.  
2. Solicita 2 bytes al sensor.  
3. Si no llegan ambos datos, retorna 0 (protección contra ruido).  
4. Combina los 12 bits válidos en un entero de 0–4095.  

---

## Notas adicionales
- El método usado (`repeated start`) garantiza lecturas consistentes sin liberar el bus I2C.
- El AS5600 es extremadamente preciso, pero **puede ser sensible al ruido**, por lo que es recomendable:
  - cable corto,  
  - buena alimentación,  
  - ferrite bead,  
  - condensador cerca del sensor (100 nF recomendado).
- La lectura es no bloqueante y se puede integrar fácilmente a un sistema de control.

---
