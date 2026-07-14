#ifndef __TOOL_PID_H__
#define __TOOL_PID_H__

#include "stm32f10x.h"

#define PID_ASSISTANT_EN          0

/* Data receive buffer size */
#define PROT_FRAME_LEN_RECV       128

/* Length of the checksum data */
#define PROT_FRAME_LEN_CHECKSUM   1


#define FRAME_HEADER              0x59485A53 // Frame header

/* Channel macro definitions */
#define CURVES_CH1 0x01
#define CURVES_CH2 0x02
#define CURVES_CH3 0x03
#define CURVES_CH4 0x04
#define CURVES_CH5 0x05

/* Commands (slave device -> host) */
#define SEND_TARGET_CMD 0x01 // Send the target value of the host channel
#define SEND_FACT_CMD 0x02   // Send the actual channel value
#define SEND_P_I_D_CMD 0x03  // Send the PID values (sync the values displayed on the host)
#define SEND_START_CMD 0x04  // Send the start command (sync the host button state)
#define SEND_STOP_CMD 0x05   // Send the stop command (sync the host button state)
#define SEND_PERIOD_CMD 0x06 // Send the period (sync the value displayed on the host)

/* Commands (host -> slave device) */
#define SET_P_I_D_CMD 0x10  // Set the PID values
#define SET_TARGET_CMD 0x11 // Set the target value
#define START_CMD 0x12      // Start command
#define STOP_CMD 0x13       // Stop command
#define RESET_CMD 0x14      // Reset command
#define SET_PERIOD_CMD 0x15 // Set the period

/* Null command */
#define CMD_NONE 0xFF // Null command

/* Index value macro definitions */
#define HEAD_INDEX_VAL 0x3u // Frame header index value (4 bytes)
#define CHX_INDEX_VAL 0x4u  // Channel index value (1 byte)
#define LEN_INDEX_VAL 0x5u  // Packet length index value (4 bytes)
#define CMD_INDEX_VAL 0x9u  // Command index value (1 byte)

#define EXCHANGE_H_L_BIT(data) ((((data) << 24) & 0xFF000000) | \
                                (((data) << 8) & 0x00FF0000) |  \
                                (((data) >> 8) & 0x0000FF00) |  \
                                (((data) >> 24) & 0x000000FF)) // Swap the high and low bytes

#define COMPOUND_32BIT(data) (((*(data - 0) << 24) & 0xFF000000) | \
                              ((*(data - 1) << 16) & 0x00FF0000) | \
                              ((*(data - 2) << 8) & 0x0000FF00) |  \
                              ((*(data - 3) << 0) & 0x000000FF)) // Combine into one word


/* Union (for convenient data conversion) */
typedef union
{
    float f;
    int i;
} type_cast_t;


#pragma pack(1) //Effect: the C compiler will align on n bytes.
/* Data header structure */
typedef struct packet_head
{
    uint32_t head; // Frame header
    uint8_t ch;    // Channel
    uint32_t len;  // Packet length
    uint8_t cmd;   // Command
    // uint8_t sum;      // Checksum

} packet_head_t;
#pragma pack() //Effect: cancel the custom byte alignment.


/**
 * @brief   Received data processing
 * @param   *data:  the array of data to be processed.
 * @param   data_len: the size of the data
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len);

/**
 * @brief   Initialize the receive protocol
 * @param   void
 * @return  initialization result.
 */
int32_t protocol_init(void);

/**
 * @brief   Processing of the received data
 * @param   void
 * @return  -1: no valid command was found.
 */
int8_t receiving_process(void);

/**
  * @brief Set the value on the host
  * @param cmd: command
  * @param ch: curve channel
  * @param data: parameter pointer
  * @param num: number of parameters
  * @retval None
  */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num);

uint8_t start_tool(void);


#endif
