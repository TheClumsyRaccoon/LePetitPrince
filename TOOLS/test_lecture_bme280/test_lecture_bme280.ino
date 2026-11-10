
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define PIN_SDA 8
#define PIN_SCL 9
#define BMP_ADDR 0x76

Adafruit_BMP280 bmp;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);

    Serial.println("\n=== TEST BMP280 ===");

    if (!bmp.begin(BMP_ADDR)) {
        Serial.println("ERREUR : BMP280 non detecte !");
        while (1) delay(1000);
    }

    Serial.println("BMP280 initialise avec succes.");

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X16,
        Adafruit_BMP280::FILTER_X4,
        Adafruit_BMP280::STANDBY_MS_500
    );
}

void loop() {
    float T = bmp.readTemperature();
    float P = bmp.readPressure() / 100.0F;

    Serial.println("\n-----------------------------");

    Serial.print("Temperature : ");
    Serial.print(T, 2);
    Serial.println(" °C");

    Serial.print("Pression    : ");
    Serial.print(P, 2);
    Serial.println(" hPa");

    bool valid = true;

    if (isnan(T) || T < -40 || T > 85) {
        Serial.println("[ERREUR] Temperature incoherente");
        valid = false;
    }

    if (isnan(P) || P < 300 || P > 1100) {
        Serial.println("[ERREUR] Pression incoherente");
        valid = false;
    }

    if (valid) {
        Serial.println("[OK] Mesures coherentes");
    }

    delay(2000);
}
