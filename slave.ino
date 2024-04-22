/** RF24Mesh_Example.ino by TMRh20

   This example sketch shows how to manually configure a node via RF24Mesh, and send data to the
   master node.
   The nodes will refresh their network address as soon as a single write fails. This allows the
   nodes to change position in relation to each other and the master node.
*/


#include "RF24.h"
#include "RF24Network.h"
#include "RF24Mesh.h"
#include <SPI.h>
#include <ArduinoJson.h>
#include <AM2302-Sensor.h>
// #include "test.pb.h"
// #include "pb_encode.h"
//#include <printf.h>
#undef MAX_PAYLOAD_SIZE
#define MAX_PAYLOAD_SIZE 3200
/**** Configure the nrf24l01 CE and CS pins ****/
RF24 radio(7, 8);
RF24Network network(radio);

RF24Mesh mesh(radio, network);

/*
 * User Configuration: nodeID - A unique identifier for each radio. Allows addressing
 * to change dynamically with physical changes to the mesh.
 *
 * In this example, configuration takes place below, prior to uploading the sketch to the device
 * A unique value from 1-255 must be configured for each node.
 */
#define GUID 3722872
#define nodeID (GUID % 254) + 1

uint32_t displayTimer = 0;

struct payload_t {
  unsigned long ms;
  unsigned long counter;
};


//const uint32_t MEM_GUID PROGMEM = EMPTY_GUID;
//uint32_t GUID = EMPTY_GUID;

struct SensorData {
    int32_t guid;
    int32_t dht_status;
    float hum_value;
    float temp_value;
    int32_t gas_status;
    float gas_value;
    char hardware_name[10] = "arduiuno02";
};
SensorData sensor_data;

void setup() {
  Serial.begin(115200);
  /*while (!Serial) {
    // some boards need this because of native USB capability
  }*/
  /*GUID = pgm_read_dword(MEM_GUID)
  if (GUID == EMPTY_GUID) {
    randomSeed(analogRead(0));
    GUID = random(1, 4000000);
  }*/
  // Set the nodeID manually
  mesh.setNodeID(nodeID);

  // Set the PA Level to MIN and disable LNA for testing & power supply related issues
  radio.begin();
  radio.setPALevel(RF24_PA_MIN, 0);

  // Connect to the mesh
  Serial.println(F("Connecting to the mesh..."));
  if (!mesh.begin()) {
    if (radio.isChipConnected()) {
      do {
        // mesh.renewAddress() will return MESH_DEFAULT_ADDRESS on failure to connect
        Serial.println(F("Could not connect to network.\nConnecting to the mesh..."));
      } while (mesh.renewAddress() == MESH_DEFAULT_ADDRESS);
    } else {
      Serial.println(F("Radio hardware not responding."));
      while (1) {
        // hold in an infinite loop
      }
    }
  }
}

#define DHT_PIN 2
AM2302::AM2302_Sensor am2302{DHT_PIN};

void collectDHT() {
  auto status = am2302.read();
  float hum = 0.0;
  float temp = 0.0;
  sensor_data.dht_status = status;
  if (status == 0) {
    temp = am2302.get_Temperature();
    hum = am2302.get_Humidity();
  }
  sensor_data.hum_value = hum;
  sensor_data.temp_value = temp;
}

#define MQ_PIN 5

void collectMQ2() {
  sensor_data.gas_status = 0;

  if (displayTimer < 20000) {
    sensor_data.gas_value = 0;
  } else {
    auto sensorValue = analogRead(MQ_PIN);
    sensor_data.gas_value = sensorValue;
  }
}

void collectJson() {
  sensor_data.guid = GUID;
  collectDHT();
  collectMQ2();
}


void loop() {

  mesh.update();

  // Send to the master node every second
  if (millis() - displayTimer >= 20000) {
    displayTimer = millis();

    // Send an 'M' type message containing the current millis()
    collectJson();
    byte byteArray[sizeof(sensor_data)];
    memcpy(byteArray, &sensor_data, sizeof(sensor_data));
    if (!mesh.write(byteArray, 'M', sizeof(sensor_data))) {
      
      // If a write fails, check connectivity to the mesh network
      if (!mesh.checkConnection()) {
        //refresh the network address
        Serial.println("Renewing Address");
        if (mesh.renewAddress() == MESH_DEFAULT_ADDRESS) {
          //If address renewal fails, reconfigure the radio and restart the mesh
          //This allows recovery from most if not all radio errors
          mesh.begin();
        }
      } else {
        Serial.println("Send fail, Test OK");
      }
    } else {
      Serial.print("Send OK: ");
      Serial.println(displayTimer);
    }
  }

  while (network.available()) {
    RF24NetworkHeader header;
    payload_t payload;
    network.read(header, &payload, sizeof(payload));
    Serial.print("Received packet #");
    Serial.print(payload.counter);
    Serial.print(" at ");
    Serial.println(payload.ms);
  }
}
