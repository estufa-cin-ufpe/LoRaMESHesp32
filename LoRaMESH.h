#ifndef _LORA_MESH_
#define _LORA_MESH_

#include <stdint.h>
#include <Arduino.h>

/* ----- Serial1 da ESP será a serial utilizada para COMANDOS da LoRaMESH ----- */

/* ----- Defines ------ */

#define MAX_PAYLOAD_SIZE 232
#define MAX_BUFFER_SIZE 237
#define rxPin 16  //rxPin to Serial1 Esp32
#define txPin 17  //txPin to Serial1 Esp32

/* -----  Default Comands List ----- */
typedef enum
{
  CMD_LORAPARAMETER   = 0xD6,   /* Gets or Sets the LoRa modulation parameters */
  CMD_LOCALREAD       = 0xE2,   /* Gets the ID, NET and UNIQUE ID info from the local device */
  CMD_REMOTEREAD      = 0xD4,   /* Gets the ID, NET and UNIQUE ID info from a remote device */
  CMD_WRITECONFIG     = 0xCA,   /* Writes configuration info to the device, i.e. NET and ID */
  CMD_GPIOCONFIG      = 0xC2,   /* Configures a given GPIO pin to a desired mode, gets or sets its value */
  CMD_DIAGNOSIS       = 0xE7,   /* Gets diagnosis information from the device */
  CMD_READNOISE       = 0xD8,   /* Reads the noise floor on the current channel */
  CMD_READRSSI        = 0xD5,   /* Reads the RSSI between the device and a neighbour */
  CMD_TRACEROUTE      = 0xD2,   /* Traces the hops from the device to the master */
  CMD_SENDTRANSP      = 0x28    /* Sends a packet to the device's transparent serial port */
} Cmd_Typedef;


/* GPIO Enum */
typedef enum
{
  GPIO0,
  GPIO1,
  GPIO2,
  GPIO3,
  GPIO4,
  GPIO5,
  GPIO6,
  GPIO7
} GPIO_Typedef;

/* GPIO mode enum */
typedef enum
{
  DIGITAL_IN,
  DIGITAL_OUT,
  ANALOG_IN = 3
} Mode_Typedef;


/* Pull resistor enum */
//se esta parte não estiver comentada código não compila
/*
typedef enum
{
  PULL_OFF,
  PULLUP,
  PULLDOWN
} Pull_Typedef;
*/


typedef enum
{
  MESH_OK,
  MESH_ERROR
} MeshStatus_Typedef;

/* ----- Public global variables ----- */

/* ----- Public Functions Prototype ----- */

/**
  * @brief Initializes Commands interface and the SoftwareSerial object on gpio 16(Rx)/ gpio17(Tx) pins and Baudrate
*/
void SerialCommandsInit(uint32_t baudRate);


/**
  * @brief Prepares a frame to transmission via commands interface
  * @param id: Target device's ID
  * @param command: Byte that indicates the command
  * @param payload: pointer to the array holding the payload
  * @param payloadSize: payload size
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef PrepareFrameCommand(uint16_t id, uint8_t command, uint8_t* payload, uint8_t payloadSize);


/**
  * @brief Sends a frame previously prepared by PrepareFrame
  * @param None
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef SendPacket();


/**
  * @brief Receives a packet from the commands interface
  * @param id[out]: ID from the received message
  * @param command[out]: received command
  * @param payload[out]: buffer where the received payload will be copied to
  * @param payloadSize[out]: received payload size
  * @param timeout: reception timeout, in ms
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef ReceivePacketCommand(uint16_t* id, uint8_t* command, uint8_t* payload, uint8_t* payloadSize, uint32_t timeout);


/**
  * @brief Gets the ID, NET and UNIQUE ID info from the local device
  * @param id[out]: Local device's id
  * @param net[out]: Configured NET on local device
  * @param uniqueId[out]: Device Unique ID 
  * @retval MESH_OK or MESH_ERROR
  */
MeshStatus_Typedef LocalRead(uint16_t* id, uint16_t* net, uint32_t* uniqueId);


/**
  * @brief  Computes CRC16.
  * @param  data_in: Pointer to the input buffer.
  * @param  length: Buffer size
  * @retval CRC16
  */
uint16_t ComputeCRC(uint8_t* data_in, uint16_t length);

#endif