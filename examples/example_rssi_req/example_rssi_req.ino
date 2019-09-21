/* Includes ---------------------- */
#include <LoRaMESH.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define SlaveID 21

uint8_t bufferPayload[MAX_PAYLOAD_SIZE] = {0};
uint8_t receivedBuffer[MAX_PAYLOAD_SIZE] = {0};
uint8_t payloadSize = 0,receivedSize = 0, rssi_ida = 0, rssi_volta = 0;
uint16_t localId, receivedId, remoteId,gateway,localNet;
uint32_t localUniqueId;
uint8_t command, receivedCommand;


void setup() {
  Serial.begin(9600);
  SerialCommandsInit(9600);//(rx_pin,tx_pin)    
  if(LocalRead(&localId, &localNet, &localUniqueId) != MESH_OK){
    Serial.print("Couldn't read local ID\n\n");
  }
  else
  {
    Serial.print("Local ID: ");
    Serial.println(localId);
    Serial.print("Local NET: ");
    Serial.println(localNet);
  }
  delay(2000);
}

void send_rssi_req(){

    remoteId = SlaveID;
    bufferPayload[0] = 0;
    bufferPayload[1] = 0;
    bufferPayload[2] = 0;
    payloadSize = 3;
    PrepareFrameCommand(remoteId,CMD_READRSSI, bufferPayload, payloadSize); //Montado frame de requisição de RSSI para SlaveID
    if(SendPacket() == MESH_OK){
        Serial.println("rssi_req_sent");
    }
    else{
      Serial.println("rssi_req_not_sent");
    }
}

uint8_t receive_rssi(){
    if(ReceivePacketCommand(&receivedId, &receivedCommand, receivedBuffer, &receivedSize, 6000) == MESH_OK)
    {
      rssi_ida = receivedBuffer[2];
      rssi_volta = receivedBuffer[3];
      Serial.print("RSSI_Ida: ");
      Serial.print(rssi_ida);
      Serial.print(" RSSI_Volta:");
      Serial.println(rssi_volta);
    }
    else{
      Serial.println("MESH_ERROR_RECEIVE_RSSI");
      rssi_volta = 0;
    } 
    return rssi_volta;
}


void loop() {

    send_rssi_req();
    delay(10);
    Serial.print("receivedRssi = ");
    Serial.println(receive_rssi());
}
