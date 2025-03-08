#include <STM32FreeRTOS.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include "DHT.h"
#include <SPI.h>
#include <LoRa.h>

// Pin definitions
#define SCK PA5
#define MOSI PA7
#define SS PA4
#define RST PB0
#define MISO PA6
#define DIO0 PD14

#define MQ2_ANALOG_PIN PC5  
#define pinLDR PC4  

#define LED_OK PB2  
#define LED_FAIL PA15  

// DHT11 Sensor
#define DHTPIN PA1
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// BMP180 Pressure Sensor
Adafruit_BMP085 bmp;

// Structure to store the latest sensor values
typedef struct {
    int brightness;
    int gas;
    int pressure;
    int temperature;
    int humidity;
    bool brightnessReceived;
    bool gasReceived;
    bool pressureReceived;
    bool temperatureReceived;
    bool humidityReceived;
} SensorData;

SensorData latestData = {0, 0, 0, 0, 0, false, false, false, false, false};

// Task data structure
typedef struct {
    char taskName[20];
    uint32_t counter;
    int value;
} TaskData;

QueueHandle_t xTaskDataQueue;  
SemaphoreHandle_t xQueueSemaphore;

// Task prototypes
void mq2Task(void *pvParameters);
void dhtTask(void *pvParameters);
void pressionTask(void *pvParameters);
void brightnessTask(void *pvParameters);
void serialWriterTask(void *pvParameters);

// LoRa parameters
int SF = 8;
int BW = 250000;
int PREAMBULE = 12;
int codingRate = 4;

void setup() {
    Serial.begin(115200);
    delay(1000);

    dht.begin();
    Wire.begin();
    
    if (!bmp.begin()) {
        Serial.println("BMP180 initialization failed!");
    }
    
    pinMode(LED_OK, OUTPUT);
    pinMode(LED_FAIL, OUTPUT);
    digitalWrite(LED_OK, LOW);
    digitalWrite(LED_FAIL, LOW);
   
    // LoRa initialization
    LoRa.setPins(SS, RST, DIO0);
    
    LoRa.setSpreadingFactor(SF);
    LoRa.setSignalBandwidth(BW);
    LoRa.setPreambleLength(PREAMBULE);
    LoRa.setTxPower(10);
    
    if (!LoRa.begin(433.5E6)) {
      Serial.println("LoRa initialization failed!");
      while (1);
    }
    Serial.println("LoRa initialized, ready to transmit.");

    // Create queue and semaphore
    xTaskDataQueue = xQueueCreate(5, sizeof(TaskData));
    //xQueueSemaphore = xSemaphoreCreateBinary();

    // Create tasks
    xTaskCreate(pressionTask, "Pressure", 1024, NULL, 1, NULL);
    xTaskCreate(brightnessTask, "Light", 1024, NULL, 1, NULL);
    xTaskCreate(mq2Task, "Gas", 1024, NULL, 1, NULL);
    xTaskCreate(dhtTask, "DHT", 1024, NULL, 1, NULL);
    xTaskCreate(serialWriterTask, "Serial", 2048, NULL, 2, NULL);

    Serial.println("Initialization complete!");
  
    vTaskStartScheduler();
}

void loop() {}

// Read brightness sensor
void brightnessTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t brightnessCounter = 0;
    const int samplePeriodMs = 500;  // Read every 500ms
    const int averagingWindowMs = 6000;  // Calculate average every 6s
    const int numSamples = averagingWindowMs / samplePeriodMs;  // Number of readings (12 readings in 6s)
    
    int brightnessBuffer[numSamples];  // Store values over 6s
    int sampleIndex = 0;

    while (1) {
        brightnessCounter++;

        // Read brightness every 500ms
        brightnessBuffer[sampleIndex] = analogRead(pinLDR);
        sampleIndex++;

        // If 6s have passed, calculate the average and send to the queue
        if (sampleIndex >= numSamples) {
            int sum = 0;
            for (int i = 0; i < numSamples; i++) {
                sum += brightnessBuffer[i];
            }
            int avgBrightness = sum / numSamples;  // Calculate the average

            // Send the average to the queue
            TaskData brightnessData;
            strcpy(brightnessData.taskName, "B :");
            brightnessData.counter = brightnessCounter;
            brightnessData.value = avgBrightness;

            xQueueSend(xTaskDataQueue, &brightnessData, portMAX_DELAY);

            // Reset the index for the next 6s period
            avgBrightness = 0;
            sampleIndex = 0;
        }

        // Wait 500ms before the next reading
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(samplePeriodMs));
    }
}

// Task to read and process MQ2 sensor data
void mq2Task(void *pvParameters) {
    pinMode(MQ2_ANALOG_PIN, INPUT); 
    static uint32_t mq2Counter = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    const int samplePeriodMs = 1000;  // Read every 1s
    const int averagingWindowMs = 6000;  // Calculate average every 6s
    const int numSamples = averagingWindowMs / samplePeriodMs;  // Number of readings (6 readings in 6s)
    
    int mq2Buffer[numSamples];  // Store values over 6s
    int sampleIndex = 0;

    while (1) {  
         ++mq2Counter;    
        // Read MQ2 sensor value
        mq2Buffer[sampleIndex] = analogRead(MQ2_ANALOG_PIN);
        sampleIndex++;

        // If 6s have passed, calculate the average and send to the queue
        if (sampleIndex >= numSamples) {
            int sum = 0;
            for (int i = 0; i < numSamples; i++) {
                sum += mq2Buffer[i];
            }
            int avgMQ2 = sum / numSamples;  // Calculate the average

            // Prepare and send the average MQ2 data to the queue
            TaskData mq2Data;
            strcpy(mq2Data.taskName, "G :");
            
            mq2Data.value = avgMQ2;
            mq2Data.counter = ++mq2Counter;
            xQueueSend(xTaskDataQueue, &mq2Data, portMAX_DELAY);
            avgMQ2 = 0;
            // Reset the index for the next 6s period
            sampleIndex = 0;
        }

        // Wait 1 second before the next reading
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(samplePeriodMs));
    }
}

// Task to read DHT11 sensor data
void dhtTask(void *pvParameters) {
    static uint32_t dhtCounter = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    TaskData humData;
    TaskData tempData;

    while (1) {
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();
        dhtCounter++;
        if (!isnan(temperature)) {
            
            strcpy(tempData.taskName, "T :");
            tempData.counter = dhtCounter;
            tempData.value = (int)temperature;
            xQueueSend(xTaskDataQueue, &tempData, portMAX_DELAY);
        }
        //else{tempData.value = -1;}

        if (!isnan(humidity)) {
            
            strcpy(humData.taskName, "H :");
            humData.counter = dhtCounter;
            humData.value = (int)humidity;
            xQueueSend(xTaskDataQueue, &humData, portMAX_DELAY);
        }
       // else{tempData.value = -1;}
       
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(6000)); 
    }
}

// Task: BMP180 Pressure
void pressionTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t pressionCounter = 0;

    while (1) {
        pressionCounter++;
        TaskData pressionData;
        strcpy(pressionData.taskName, "P :");

        float pressure = bmp.readPressure();  // Read pressure
        if (pressure > 50000 && pressure < 120000) {  // Check if pressure is realistic
            pressionData.value = (int)pressure;
            xQueueSend(xTaskDataQueue, &pressionData, portMAX_DELAY);
        } //else {pressionData.value = -1;  // Set -1 if reading is invalid
        //}
        //xQueueSend(xTaskDataQueue, &pressionData, portMAX_DELAY);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(6000));
    }
}

// Task to write data to serial and send via LoRa
void serialWriterTask(void *pvParameters) {
    TaskData receivedData;
    String sendMessage = "";
    unsigned long startWaitTime;

    while (1) {
        // Reset received values
            latestData.brightnessReceived = false;
            latestData.gasReceived = false;
            latestData.pressureReceived = false;
            latestData.temperatureReceived = false;
            latestData.humidityReceived = false;
       
            if (xQueueReceive(xTaskDataQueue, &receivedData, portMAX_DELAY) == pdPASS) {
                
                // Store received data
                if (strcmp(receivedData.taskName, "B :") == 0) {
                    latestData.brightness = receivedData.value;
                    latestData.brightnessReceived = true;
                } else if (strcmp(receivedData.taskName, "G :") == 0) {
                    latestData.gas = receivedData.value;
                    latestData.gasReceived = true;
                } else if (strcmp(receivedData.taskName, "P :") == 0) {
                    latestData.pressure = receivedData.value;
                    latestData.pressureReceived = true;
                } else if (strcmp(receivedData.taskName, "T :") == 0) {
                    latestData.temperature = receivedData.value;
                    latestData.temperatureReceived = true;
                } else if (strcmp(receivedData.taskName, "H :") == 0) {
                    latestData.humidity = receivedData.value;
                    latestData.humidityReceived = true;
                }
            }

                        // Check if all data is received
            if (latestData.brightnessReceived &&
                latestData.gasReceived &&
                latestData.pressureReceived &&
                latestData.temperatureReceived &&
                latestData.humidityReceived) {
                
                // Build the message to send
                sendMessage = "B:" + String(latestData.brightness) + ",";
                sendMessage += "G:" + String(latestData.gas) + ",";
                sendMessage += "P:" + String(latestData.pressure) + ",";
                sendMessage += "T:" + String(latestData.temperature) + ",";
                sendMessage += "H:" + String(latestData.humidity);

                Serial.println("Sending LoRa Packet:");
                Serial.println(sendMessage);

                // Send via LoRa
                LoRa.beginPacket();
                LoRa.print(sendMessage);
                LoRa.endPacket();
                digitalWrite(LED_OK, !digitalRead(LED_OK));

                int payloadSize = sendMessage.length();

                // Calculate Time On Air
                float timeOnAir = calculateTimeOnAir(payloadSize, SF, BW, codingRate, PREAMBULE);
                Serial.print("Time on Air: ");
                Serial.print(timeOnAir);
                Serial.println(" ms");
                //Serial.print(" -----------------------------------------------------------------------\n");
                }else{

        // Check if any sensors are missing
        bool missingData = !(latestData.brightnessReceived &&
                             latestData.gasReceived &&
                             latestData.pressureReceived &&
                             latestData.temperatureReceived &&
                             latestData.humidityReceived);

        // If sensors are missing, wait for an additional 2 seconds
        if (missingData) {
            startWaitTime = millis();
            while (millis() - startWaitTime < 2000) {  
                if (xQueueReceive(xTaskDataQueue, &receivedData, portMAX_DELAY) == pdPASS) {
                    
                    // Store received data during the additional wait period
                    if (strcmp(receivedData.taskName, "B :") == 0) {
                        latestData.brightness = receivedData.value;
                        latestData.brightnessReceived = true;
                    } else if (strcmp(receivedData.taskName, "G :") == 0) {
                        latestData.gas = receivedData.value;
                        latestData.gasReceived = true;
                    } else if (strcmp(receivedData.taskName, "P :") == 0) {
                        latestData.pressure = receivedData.value;
                        latestData.pressureReceived = true;
                    } else if (strcmp(receivedData.taskName, "T :") == 0) {
                        latestData.temperature = receivedData.value;
                        latestData.temperatureReceived = true;
                    } else if (strcmp(receivedData.taskName, "H :") == 0) {
                        latestData.humidity = receivedData.value;
                        latestData.humidityReceived = true;
                    }
                }
            }
        }

        String missingSensors = "";
        if (!latestData.brightnessReceived) missingSensors += "Brightness, ";
        if (!latestData.gasReceived) missingSensors += "Gas, ";
        if (!latestData.pressureReceived) missingSensors += "Pressure, ";
        if (!latestData.temperatureReceived) missingSensors += "Temperature, ";
        if (!latestData.humidityReceived) missingSensors += "Humidity, ";

        // IF A SENSOR IS MISSING, DISPLAY A MESSAGE AND WAIT 2s
        if (missingSensors.length() > 0) {
            Serial.print("Missing data: ");
            Serial.println(missingSensors);
           //Serial.print(" -----------------------------------------------------------------------\n");
            
        }

        // Assign -1 to sensors that did not respond after 4s max
        if (!latestData.brightnessReceived) latestData.brightness = -1;
        if (!latestData.gasReceived) latestData.gas = -1;
        if (!latestData.pressureReceived) latestData.pressure = -1;
        if (!latestData.temperatureReceived) latestData.temperature = -1;
        if (!latestData.humidityReceived) latestData.humidity = -1;

        // Build the message while maintaining the fixed order of sensors
        sendMessage = "B:" + String(latestData.brightness) + ",";
        sendMessage += "G:" + String(latestData.gas) + ",";
        sendMessage += "P:" + String(latestData.pressure) + ",";
        sendMessage += "T:" + String(latestData.temperature) + ",";
        sendMessage += "H:" + String(latestData.humidity);

        Serial.println("Sending LoRa Packet:");
        Serial.println(sendMessage);

        // Send via LoRa
        LoRa.beginPacket();
        LoRa.print(sendMessage);
        LoRa.endPacket();
        digitalWrite(LED_OK, !digitalRead(LED_OK));
    
        int payloadSize = sendMessage.length();

        // Calculate Time On Air
        float timeOnAir = calculateTimeOnAir(payloadSize, SF, BW, codingRate, PREAMBULE);
        Serial.print("Time on Air: ");
        Serial.print(timeOnAir);
        Serial.println(" ms");
        Serial.print(" -----------------------------------------------------------------------\n");
   }
}



// Calculate the number of payload symbols
int calculateNpayload(int payloadSize,int SF,  int CR, bool hasCRC, bool isImplicitHeader, bool lowDataRateOptimize) {
    // CR Coding Rate (1 for 4/5, 2 for 4/6, 3 for 4/7, 4 for 4/8)
    int Coding_Rate = hasCRC ? 1 : 0;  // 1 if CRC enabled, otherwise 0
    int IH = isImplicitHeader ? 1 : 0; // 1 if implicit header enabled, otherwise 0
    int DE = lowDataRateOptimize ? 1 : 0; // This option is automatically enabled when: SF>=11 and low BW

    // Calculate the number of payload symbols
    int numerator = (8 * payloadSize - 4 * SF + 28 + 16 * Coding_Rate  - 20 * IH);
    int denominator = 4 * (SF - 2 * DE);
    
    int Npayload = 8 + max((int)ceil((double)numerator / denominator) * (CR + 4), 0);
    return Npayload;
}

// Calculate Time on Air
float calculateTimeOnAir(int payloadSize,  int SF,  int BW,  int CR,  int preambleLength) {
    float Tsymbol = pow(2, SF) / (float) BW; // Symbol time
    float Tpreamble = (preambleLength + 4.25) * Tsymbol; // Preamble time

    // Calculate the number of payload symbols
    int Npayload = calculateNpayload(payloadSize, SF, CR ,0,0,0);

    float Tpayload = Npayload * Tsymbol; // Payload time
    float ToA = Tpreamble + Tpayload; // Total Time on Air in seconds

    return ToA * 1000; // Convert to milliseconds
}
