// ========================================================================
//      🔸 A S 5 6 0 0  -  I 2 C   A N G L E   2x   S E N S O R S 🔸 
// ========================================================================
//  Archivo    : AS5600--I2C_angle_2xSensors.ino
//  Autor      : Klaus Michalsky
//  Fecha      : Feb-2026
//
//  DESCRIPCION
//  -----------------------------------------------------------------------
//  - Lee el ángulo de 2 sensores AS5600 conectado a GP4 (SDA) y GP5 (SCL) 
//    GP26 (SDA) y GP27 (SCL) usando I2C.
//  - Convierte el valor crudo de 12 bits (0–4095) a grados (0°–360°).
//  - Muestra el ángulo por Serial cada 200 ms.
//  - Usa Wire para I2C0 y Wire1 para I2C1 independiente de los pines por defecto.
//
//  HARDWARE
//  -----------------------------------------------------------------------
//  MCU     : RP2040-Zero
//  Sensor  : AS5600
//
//  ESTADO
//  -----------------------------------------------------------------------
//  ✅ Funcional
// ========================================================================

#include <Wire.h>

#define AS5600_ADDR 0x36 // Dirección fija del AS5600

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Configura I2C en GP26=SDA, GP27=SCL usando Wire1
    Wire1.setSDA(26);
    Wire1.setSCL(27);
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire1.begin();
    Wire.begin();

    Serial.println("AS5600 listo para lectura con Wire1");
    Serial.println("AS5600 listo para lectura con Wire");
}

void loop()
{
    uint16_t angle_mot1 = readAS5600Angle_mot1();       // Lectura cruda 0–4095
    uint16_t angle_mot2 = readAS5600Angle_mot2();       // Lectura cruda 0–4095
    float degrees_mot1 = (angle_mot1 * 360.0) / 4096.0; // Conversión a grados
    float degrees_mot2 = (angle_mot2 * 360.0) / 4096.0; // Conversión a grados

    Serial.print("Ángulo_Motor1: ");
    Serial.print(degrees_mot1);
    Serial.println("°");
    Serial.print("Ángulo_Motor2: ");
    Serial.print(degrees_mot2);
    Serial.println("°");

    delay(200);
}

// Función para leer el registro de ángulo del AS5600 en Motor1
uint16_t readAS5600Angle_mot1()
{
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0E);            // Registro ANGLE (High byte)
    Wire.endTransmission(false); // Mantener bus activo
    Wire.requestFrom(AS5600_ADDR, (uint8_t)2);

    if (Wire.available() < 2)
        return 0; // Evita lecturas vacías por ruido

    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    return ((high & 0x0F) << 8) | low;
}

// Función para leer el registro de ángulo del AS5600 en Motor2
uint16_t readAS5600Angle_mot2()
{
    Wire1.beginTransmission(AS5600_ADDR);
    Wire1.write(0x0E);            // Registro ANGLE (High byte)
    Wire1.endTransmission(false); // Mantener bus activo
    Wire1.requestFrom(AS5600_ADDR, (uint8_t)2);

    if (Wire1.available() < 2)
        return 0; // Evita lecturas vacías por ruido

    uint8_t high = Wire1.read();
    uint8_t low = Wire1.read();
    return ((high & 0x0F) << 8) | low;
}
