
#include <Wire.h>
#include "MS5837.h"

#define PIN_SDA 8
#define PIN_SCL 9

MS5837 sensor;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);

    Serial.println("\n=== TEST MS5837 ===");

    if (!sensor.init()) {
        Serial.println("ERREUR : MS5837 non detecte !");
        while (1) delay(1000);
    }

    // Mode 30 bar par defaut
    sensor.setModel(MS5837::MS5837_30BA);

    // Densite eau de mer en kg/m3
    sensor.setFluidDensity(1029);

    Serial.println("MS5837 initialise avec succes !");
}

void loop() {
    sensor.read();

    float temperature = sensor.temperature();
    float pressure = sensor.pressure();  // mbar
    float depth = sensor.depth();        // metres

    Serial.println("-----------------------------");

    Serial.print("Temperature : ");
    Serial.print(temperature, 2);
    Serial.println(" °C");

    Serial.print("Pression    : ");
    Serial.print(pressure, 2);
    Serial.println(" mbar");

    Serial.print("Profondeur  : ");
    Serial.print(depth, 3);
    Serial.println(" m");

    bool valid = true;

    if (isnan(temperature) || temperature < -20 || temperature > 85) {
        Serial.println("[ERREUR] Temperature incoherente");
        valid = false;
    }

    if (isnan(pressure) || pressure <= 0 || pressure > 30000) {
        Serial.println("[ERREUR] Pression incoherente");
        valid = false;
    }

    if (isnan(depth) || depth < -1 || depth > 300) {
        Serial.println("[ERREUR] Profondeur incoherente");
        valid = false;
    }

    if (valid) {
        Serial.println("[OK] Mesures coherentes");
    }

    delay(1000);
}
