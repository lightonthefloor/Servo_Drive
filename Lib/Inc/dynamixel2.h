#ifndef _DYNAMIXEL2_HEADER_
#define _DYNAMIXEL2_HEADER_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#ifdef __cplusplus
// extern "C" {
//#endif

// #define PRINT_INSTRUCTION 1

#define MAX_ID (0xFC)
#define BROADCAST_ID (0xFE) // BroadCast ID 

// Communication Errorrobotis1	1 List
#define COMM_TXSUCCESS (0) // Succeed transmit instruction packet
#define COMM_RXSUCCESS (1) // Succeed get status packet
#define COMM_TXFAIL (2)    // Failed transmit instruction packet
#define COMM_RXFAIL (3)    // Failed get status packet
#define COMM_TXERROR (4)   // Incorrect instruction packet
#define COMM_RXWAITING (5) // Now recieving status packet
#define COMM_RXTIMEOUT (6) // There is no status packet
#define COMM_RXCORRUPT (7) // Incorrect status packet

// Error Status List
#define ERRBIT_ALERT                                                           \
  (128) // When the device has a problem, it is et as 1. Check "Device Status
        // Check" value.

#define ERR_RESULT_FAIL (1) // Failed to process the instruction packet.
#define ERR_INSTRUCTION (2) // Instruction error
#define ERR_CRC (3)         // CRC check error
#define ERR_DATA_RANGE (4)  // Data range error
#define ERR_DATA_LENGTH (5) // Data length error
#define ERR_DATA_LIMIT (6)  // Data limit error
#define ERR_ACCESS (7)      // Access error

// for Protocol 1.0
#define PRT1_PKT_ID (2)
#define PRT1_PKT_LENGTH (3)
#define PRT1_PKT_INSTRUCTION (4)
#define PRT1_PKT_ERRBIT (4)
#define PRT1_PKT_PARAMETER0 (5)

// for Protocol 2.0
#define PRT2_PKT_HEADER0 (0)
#define PRT2_PKT_HEADER1 (1)
#define PRT2_PKT_HEADER2 (2)
#define PRT2_PKT_RESERVED (3)
#define PRT2_PKT_ID (4)
#define PRT2_PKT_LENGTH_L (5)
#define PRT2_PKT_LENGTH_H (6)
#define PRT2_PKT_INSTRUCTION (7)
#define PRT2_INSTRUCTION_PKT_PARAMETER0 (8)
#define PRT2_PKT_ERRBIT (8)
#define PRT2_STATUS_PKT_PARAMETER0 (9)

// Instruction for Dynamixel Protocol

// Common Instruction for 1.0 and 2.0
#define INST_PING             (0x01)  //Instruction that checks whether the Packet has arrived to a device with the same ID as Packet ID
#define INST_READ             (0x02)  //Instruction to read data from the Device
#define INST_WRITE            (0x03)  //Instruction to write data on the Device
#define INST_REG_WRITE        (0x04)  //Instruction that registers the Instruction Packet to a standby status; Packet is later executed through the Action command
#define INST_ACTION           (0x05)  //Instruction that executes the Packet that was registered beforehand using Reg Write
#define INST_RESET            (0x06)  //Instruction that resets the Control Table to its initial factory default settings
// #define INST_REBOOT           (0x08)  //Instruction to reboot the Device
#define INST_CLEAR            (0x10)  //Instruction to reset certain information
#define INST_CTRL_TABLE_BKP   (0x20)  //Instruction to store current Control Table status data to a Backup area or to restore EEPROM data.
#define INST_STATUS           (0x55)  //Return packet for the Instruction Packet
// #define INST_SYNC_READ        (0x82)  //For multiple devices, Instruction to read data from the same Address with the same length at once
#define INST_SYNC_WRITE       (0x83)  //For multiple devices, Instruction to write data on the same Address with the same length at once
#define INST_FAST_SYNC_READ   (0x8A)  //For multiple devices, Instruction to read data from the same Address with the same length at once
#define INST_BULK_READ        (0x92)  //For multiple devices, Instruction to read data from different Addresses with different lengths at once
// #define INST_BULK_WRITE       (0x93)  //For multiple devices, Instruction to write data on different Addresses with different lengths at once
#define INST_FAST_BULK_READ   (0x9A)  //For multiple devices, Instruction to read data from different Addresses with different lengths at once


// Added Instruction for 2.0
#define INST_REBOOT           (0x08)  //Instruction to reboot the Device
#define INST_STATUS           (0x55)  //Return packet for the Instruction Packet
#define INST_SYNC_READ        (0x82)  //For multiple devices, Instruction to read data from the same Address with the same length at once
#define INST_BULK_WRITE       (0x93)  //For multiple devices, Instruction to write data on different Addresses with different lengths at once

#define PING_INFO_MODEL_NUM (1)
#define PING_INFO_FIRM_VER (2)

// utility for value
#define DXL_MAKEWORD(a, b)                                                     \
  ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) |           \
                    ((unsigned short)((unsigned char)(((unsigned long)(b)) &   \
                                                      0xff)))                  \
                        << 8))
#define DXL_MAKEDWORD(a, b)                                                    \
  ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) |          \
                  ((unsigned int)((unsigned short)(((unsigned long)(b)) &      \
                                                   0xffff)))                   \
                      << 16))
#define DXL_LOWORD(l) ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define DXL_HIWORD(l) ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w) ((unsigned char)(((unsigned long)(w)) & 0xff))
#define DXL_HIBYTE(w) ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

// Common Method for 1.0 & 2.0
// device control method
int dxl_initialize(int port_num, int baud_rate);
int dxl_change_baudrate(int baud_rate);
int dxl_terminate(void);

// get communication result method
int dxl_get_comm_result(void);

// Dynamixel Protocol 1.0
// 1.0 packet communocation method
void dxl_tx_packet(void);
void dxl_rx_packet(void);
void dxl_txrx_packet(void);

// get/set packet methods ]
void dxl_set_txpacket_id(int id);
void dxl_set_txpacket_instruction(int instruction);
void dxl_set_txpacket_parameter(int index, int value);
void dxl_set_txpacket_length(int length);
int dxl_get_rxpacket_error(int error);
int dxl_get_rxpacket_error_byte(void);

int dxl_get_rxpacket_parameter(int index);
int dxl_get_rxpacket_length(void);

// high communication method
void dxl_ping(int id);
int dxl_read_byte(int id, int address);
void dxl_write_byte(int id, int address, int value);
int dxl_read_word(int id, int address);
void dxl_write_word(int id, int address, int value);

// Dynamixel Protocol 2.0
// 1.0 packet communocation method
void dxl2_tx_packet(void);
void dxl2_rx_packet(void);
void dxl2_txrx_packet(void);

// get/set packet methods
void dxl2_set_txpacket_id(unsigned char id);
void dxl2_set_txpacket_instruction(unsigned char instruction);
void dxl2_set_txpacket_parameter(unsigned short index, unsigned char value);
void dxl2_set_txpacket_length(unsigned short length);
int dxl2_get_rxpacket_error_byte(void);

int dxl2_get_rxpacket_parameter(int index);
int dxl2_get_rxpacket_length(void);

// high communication method
void dxl2_ping(unsigned char id);
int dxl2_get_ping_result(unsigned char id, int info_num);
void dxl2_broadcast_ping(void);

void dxl2_reboot(unsigned char id);
void dxl2_factory_reset(unsigned char id, int option);

unsigned char dxl2_read_byte(unsigned char id, int address);
void dxl2_write_byte(unsigned char id, int address, unsigned char value);
unsigned short dxl2_read_word(unsigned char id, int address);
void dxl2_write_word(unsigned char id, int address, unsigned short value);
unsigned long dxl2_read_dword(unsigned char id, int address);
void dxl2_write_dword(unsigned char id, int address, unsigned long value);

//////////////// method for sync/bulk read ////////////////
unsigned char dxl2_get_bulk_read_data_byte(unsigned char id,
                                           unsigned int start_address);
unsigned short dxl2_get_bulk_read_data_word(unsigned char id,
                                            unsigned int start_address);
unsigned long dxl2_get_bulk_read_data_dword(unsigned char id,
                                            unsigned int start_address);

unsigned char dxl2_get_sync_read_data_byte(unsigned char id,
                                           unsigned int start_address);
unsigned short dxl2_get_sync_read_data_word(unsigned char id,
                                            unsigned int start_address);
unsigned long dxl2_get_sync_read_data_dword(unsigned char id,
                                            unsigned int start_address);

// void   dxl2_sync_write(unsigned char param[], int param_length);
void dxl2_sync_write(unsigned char address, unsigned char data_length, unsigned char id[], uint32_t value[], unsigned short num);
//
//#ifdef __cplusplus
//}
//#endif
#endif
