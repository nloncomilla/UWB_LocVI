#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

// WiFi and MQTT configuration
const char* ssid = "iPhone ncls"; //Cambiar simpre que se conecte desde otro lado
const char* password = "goku1995"; 
const char* mqtt_server = "broker.emqx.io";

WiFiClient espClient;
PubSubClient client(espClient);

#define UWB_INDEX 0

#define TAG
//#define ANCHOR

// #define FREQ_850K
#define FREQ_6800K

#define UWB_TAG_COUNT 1

#define SERIAL_LOG Serial
#define SERIAL_AT mySerial2

HardwareSerial SERIAL_AT(2);

#define RESET 32

#define IO_RXD2 18
#define IO_TXD2 19

#define I2C_SDA 4
#define I2C_SCL 5

Adafruit_SSD1306 display(128, 64, &Wire, -1);

unsigned long lastMQTTCheck = 0;  // Para el control del tiempo de envío a MQTT
String lastSerialValue = "";  // Almacena el último valor leído del serial


QueueHandle_t mqttQueue;


void logoshow(void) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("MaUWB DW3000"));

    display.setCursor(0, 20);
    display.setTextSize(2);

    String temp = "";

#ifdef TAG
    temp = temp + "T" + UWB_INDEX;
#endif
#ifdef ANCHOR
    temp = temp + "A" + UWB_INDEX;
#endif
#ifdef FREQ_850K
    temp = temp + "   850k";
#endif
#ifdef FREQ_6800K
    temp = temp + "   6.8M";
#endif
    display.println(temp);

    display.setCursor(0, 40);

    temp = "Total: " + String(UWB_TAG_COUNT);
    display.println(temp);

    display.display();
    delay(2000);
}

// Función para enviar comandos AT
String sendData(String command, const int timeout, boolean debug) {
    String response = "";
    SERIAL_LOG.println(command);
    SERIAL_AT.println(command); 

    long int time = millis();

    while ((time + timeout) > millis()) {
        while (SERIAL_AT.available()) {
            char c = SERIAL_AT.read();
            response += c;
        }
    }

    if (debug) {
        SERIAL_LOG.println(response);
    }

    return response;
}

String config_cmd() {
    String temp = "AT+SETCFG=";
    temp += UWB_INDEX;

#ifdef TAG
    temp += ",0";
#endif
#ifdef ANCHOR
    temp += ",1";
#endif

#ifdef FREQ_850K
    temp += ",0";
#endif
#ifdef FREQ_6800K
    temp += ",1";
#endif

    temp += ",1"; 
    return temp;
}

String cap_cmd() {
    String temp = "AT+SETCAP=";
    temp += UWB_TAG_COUNT;

#ifdef FREQ_850K
    temp += ",15";
#endif
#ifdef FREQ_6800K
    temp += ",10";
#endif

    return temp;
}

// Configuración de WiFi
void setup_wifi() {
    delay(10);
    SERIAL_LOG.println();
    SERIAL_LOG.print("Connecting to ");
    SERIAL_LOG.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        SERIAL_LOG.print(".");
    }

    SERIAL_LOG.println("");
    SERIAL_LOG.println("WiFi connected");
    SERIAL_LOG.println("IP address: ");
    SERIAL_LOG.println(WiFi.localIP());

    // Mostrar IP en OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("WiFi Connected");
    display.println("IP address:");
    display.println(WiFi.localIP().toString());
    display.display();
}

// Reconexión a MQTT
void reconnect() {
    while (!client.connected()) {
        SERIAL_LOG.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client")) {
            SERIAL_LOG.println("connected");
        } else {
            SERIAL_LOG.print("failed, rc=");
            SERIAL_LOG.print(client.state());
            SERIAL_LOG.println(" try again in 0.5 seconds");
            delay(500);
        }
    }
}

// Función para enviar datos a MQTT
void sendDataToMQTT(String data) {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Mensaje de depuración
    SERIAL_LOG.print("Attempting to send data to MQTT: ");
    SERIAL_LOG.println(data);

    // Envío de datos al broker MQTT
    if (client.publish("esp32/uwb_data", data.c_str())) {
        SERIAL_LOG.println("Data sent successfully");
    } else {
        SERIAL_LOG.println("Failed to send data");
    }
}

// Tarea de procesamiento de datos serial
void serialTask(void * parameter) {
    String response = "";

    while (true) {
        while (SERIAL_AT.available() > 0) {
            char c = SERIAL_AT.read();
            if (c == '\r') continue;
            else if (c == '\n') {
                SERIAL_LOG.println(response);
                lastSerialValue = response;
                xQueueSend(mqttQueue, &response, portMAX_DELAY); // Enviar el valor leído a la cola
                response = "";
            } else {
                response += c;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Pequeña pausa
    }
}

// Tarea para enviar datos a MQTT
void mqttTask(void * parameter) {
    String dataToSend;

    while (true) {
        if (xQueueReceive(mqttQueue, &dataToSend, portMAX_DELAY)) {
            sendDataToMQTT(dataToSend);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Pausa corta
    }
}

// Configuración inicial del dispositivo
void setup() {
    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, HIGH);

    SERIAL_LOG.begin(115200);

    SERIAL_LOG.print(F("Hello! ESP32-S3 AT command V1.0 Test"));
    SERIAL_AT.begin(115200, SERIAL_8N1, IO_RXD2, IO_TXD2);

    SERIAL_AT.println("AT");
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(1000);
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        SERIAL_LOG.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    display.clearDisplay();

    setup_wifi();
    client.setServer(mqtt_server, 1883);

    logoshow();

    sendData("AT?", 2000, 1);
    sendData("AT+RESTORE", 5000, 1);
    sendData(config_cmd(), 2000, 1);
    sendData(cap_cmd(), 2000, 1);
    sendData("AT+SETRPT=1", 2000, 1);
    sendData("AT+SAVE", 2000, 1);
    sendData("AT+RESTART", 2000, 1);

    // Crear cola de comunicación para MQTT
    mqttQueue = xQueueCreate(10, sizeof(String));

    // Crear tareas de FreeRTOS
    xTaskCreate(serialTask, "Serial Task", 4096, NULL, 1, NULL);
    xTaskCreate(mqttTask, "MQTT Task", 4096, NULL, 1, NULL);
}

void loop() {
    // Vacío porque las tareas están gestionadas por FreeRTOS
}
