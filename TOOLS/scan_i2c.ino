#include <Wire.h>
#define PIN_SDA 8
#define PIN_SCL 9

void setup() {
    Serial.begin(115200);
    delay(1000);
    Wire.begin(PIN_SDA, PIN_SCL);
}

void loop() {
    Serial.println("Scan I2C : ");
    int n = 0;
    for (uint8_t a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            Serial.print("Trouve : 0x");
            Serial.println(a, HEX);
            n++;
        }
    }
    if (n == 0) Serial.println("Rien de trouvé :c");
    delay(3000);
}