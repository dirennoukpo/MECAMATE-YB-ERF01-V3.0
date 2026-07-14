#include "tool_pid.h"
#include <string.h>
#include "yb_debug.h"
#include "bsp_usart.h"

#include "app_motion.h"
#include "app_pid.h"
#include "bsp.h"

uint8_t g_start_tool = 0;
float g_speed = 500;

struct prot_frame_parser_t
{
    uint8_t *recv_ptr;
    uint16_t r_oft;
    uint16_t w_oft;
    uint16_t frame_len;
    uint16_t found_frame_head;
};

static struct prot_frame_parser_t parser;

static uint8_t recv_buf[PROT_FRAME_LEN_RECV];

/**
  * @brief Compute the checksum
  * @param ptr: data to be computed
  * @param len: length to be computed
  * @retval Checksum
  */
uint8_t check_sum(uint8_t init, uint8_t *ptr, uint8_t len)
{
    uint8_t sum = init;

    while (len--)
    {
        sum += *ptr;
        ptr++;
    }

    return sum;
}

/**
 * @brief   Get the frame type (frame command)
 * @param   *frame:  data frame
 * @param   head_oft: offset of the frame header
 * @return  Frame length.
 */
static uint8_t get_frame_type(uint8_t *frame, uint16_t head_oft)
{
    return (frame[(head_oft + CMD_INDEX_VAL) % PROT_FRAME_LEN_RECV] & 0xFF);
}

/**
 * @brief   Get the frame length
 * @param   *buf:  data buffer.
 * @param   head_oft: offset of the frame header
 * @return  Frame length.
 */
static uint16_t get_frame_len(uint8_t *frame, uint16_t head_oft)
{
    return ((frame[(head_oft + LEN_INDEX_VAL + 0) % PROT_FRAME_LEN_RECV] << 0) |
            (frame[(head_oft + LEN_INDEX_VAL + 1) % PROT_FRAME_LEN_RECV] << 8) |
            (frame[(head_oft + LEN_INDEX_VAL + 2) % PROT_FRAME_LEN_RECV] << 16) |
            (frame[(head_oft + LEN_INDEX_VAL + 3) % PROT_FRAME_LEN_RECV] << 24)); // Compose the frame length
}

/**
 * @brief   Get the crc-16 checksum value
 * @param   *frame:  data buffer.
 * @param   head_oft: offset of the frame header
 * @param   head_oft: frame length
 * @return  Frame length.
 */
static uint8_t get_frame_checksum(uint8_t *frame, uint16_t head_oft, uint16_t frame_len)
{
    return (frame[(head_oft + frame_len - 1) % PROT_FRAME_LEN_RECV]);
}

/**
 * @brief   Find the frame header
 * @param   *buf:  data buffer.
 * @param   ring_buf_len: buffer size
 * @param   start: start position
 * @param   len: length to be searched
 * @return  -1: frame header not found, other values: position of the frame header.
 */
static int32_t recvbuf_find_header(uint8_t *buf, uint16_t ring_buf_len, uint16_t start, uint16_t len)
{
    uint16_t i = 0;

    for (i = 0; i < (len - 3); i++)
    {
        if (((buf[(start + i + 0) % ring_buf_len] << 0) |
             (buf[(start + i + 1) % ring_buf_len] << 8) |
             (buf[(start + i + 2) % ring_buf_len] << 16) |
             (buf[(start + i + 3) % ring_buf_len] << 24)) == FRAME_HEADER)
        {
            return ((start + i) % ring_buf_len);
        }
    }
    return -1;
}

/**
 * @brief   Compute the length of the unparsed data
 * @param   *buf:  data buffer.
 * @param   ring_buf_len: buffer size
 * @param   start: start position
 * @param   end: end position
 * @return  Length of the unparsed data
 */
static int32_t recvbuf_get_len_to_parse(uint16_t frame_len, uint16_t ring_buf_len, uint16_t start, uint16_t end)
{
    uint16_t unparsed_data_len = 0;

    if (start <= end)
        unparsed_data_len = end - start;
    else
        unparsed_data_len = ring_buf_len - start + end;

    if (frame_len > unparsed_data_len)
        return 0;
    else
        return unparsed_data_len;
}

/**
 * @brief   Write the received data into the buffer
 * @param   *buf:  data buffer.
 * @param   ring_buf_len: buffer size
 * @param   w_oft: write offset
 * @param   *data: data to be written
 * @param   *data_len: length of the data to be written
 * @return  void.
 */
static void recvbuf_put_data(uint8_t *buf, uint16_t ring_buf_len, uint16_t w_oft,
                             uint8_t *data, uint16_t data_len)
{
    if ((w_oft + data_len) > ring_buf_len) // Beyond the end of the buffer
    {
        uint16_t data_len_part = ring_buf_len - w_oft; // Remaining length of the buffer

        /* Write the data into the buffer in two segments*/
        memcpy(buf + w_oft, data, data_len_part);                    // Write to the buffer tail
        memcpy(buf, data + data_len_part, data_len - data_len_part); // Write to the buffer head
    }
    else
        memcpy(buf + w_oft, data, data_len); // Write the data into the buffer
}

/**
 * @brief   Query the frame type (command)
 * @param   *data:  frame data
 * @param   data_len: size of the frame data
 * @return  Frame type (command).
 */
static uint8_t protocol_frame_parse(uint8_t *data, uint16_t *data_len)
{
    uint8_t frame_type = CMD_NONE;
    uint16_t need_to_parse_len = 0;
    int16_t header_oft = -1;
    uint8_t checksum = 0;

    need_to_parse_len = recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV, parser.r_oft, parser.w_oft); // Get the length of the unparsed data
    if (need_to_parse_len < 9)                                                                                       // Certainly cannot find both the frame header and the frame length yet
        return frame_type;

    /* The frame header has not been found yet, a search is needed*/
    if (0 == parser.found_frame_head)
    {
        /* The sync header is four bytes; the last byte of the unparsed data may happen to be the first byte of the sync header,
           therefore when searching for the sync header the last byte is not parsed, nor is it discarded*/
        header_oft = recvbuf_find_header(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.r_oft, need_to_parse_len);
        if (0 <= header_oft)
        {
            /* The frame header has been found*/
            parser.found_frame_head = 1;
            parser.r_oft = header_oft;

            /* Check whether the frame length can be computed*/
            if (recvbuf_get_len_to_parse(parser.frame_len, PROT_FRAME_LEN_RECV,
                                         parser.r_oft, parser.w_oft) < 9)
                return frame_type;
        }
        else
        {
            /* Still no frame header found in the unparsed data, discard all the data parsed this time*/
            parser.r_oft = ((parser.r_oft + need_to_parse_len - 3) % PROT_FRAME_LEN_RECV);
            return frame_type;
        }
    }

    /* Compute the frame length and determine whether the data can be parsed*/
    if (0 == parser.frame_len)
    {
        parser.frame_len = get_frame_len(parser.recv_ptr, parser.r_oft);
        if (need_to_parse_len < parser.frame_len)
            return frame_type;
    }

    /* The frame header position is confirmed and the unparsed data exceeds the frame length, the checksum can be computed*/
    if ((parser.frame_len + parser.r_oft - PROT_FRAME_LEN_CHECKSUM) > PROT_FRAME_LEN_RECV)
    {
        /* The data frame is split in two parts, one at the buffer tail, one at the buffer head */
        checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft,
                             PROT_FRAME_LEN_RECV - parser.r_oft);
        checksum = check_sum(checksum, parser.recv_ptr, parser.frame_len - PROT_FRAME_LEN_CHECKSUM + parser.r_oft - PROT_FRAME_LEN_RECV);
    }
    else
    {
        /* The data frame can be fetched in one go*/
        checksum = check_sum(checksum, parser.recv_ptr + parser.r_oft, parser.frame_len - PROT_FRAME_LEN_CHECKSUM);
    }

    if (checksum == get_frame_checksum(parser.recv_ptr, parser.r_oft, parser.frame_len))
    {
        /* 校验成功，拷贝整帧数据 */
        if ((parser.r_oft + parser.frame_len) > PROT_FRAME_LEN_RECV)
        {
            /* 数据帧被分为两部分，一部分在缓冲区尾，一部分在缓冲区头*/
            uint16_t data_len_part = PROT_FRAME_LEN_RECV - parser.r_oft;
            memcpy(data, parser.recv_ptr + parser.r_oft, data_len_part);
            memcpy(data + data_len_part, parser.recv_ptr, parser.frame_len - data_len_part);
        }
        else
        {
            /* The data frame can be fetched in one go*/
            memcpy(data, parser.recv_ptr + parser.r_oft, parser.frame_len);
        }
        *data_len = parser.frame_len;
        frame_type = get_frame_type(parser.recv_ptr, parser.r_oft);

        /* 丢弃缓冲区中的命令帧*/
        parser.r_oft = (parser.r_oft + parser.frame_len) % PROT_FRAME_LEN_RECV;
    }
    else
    {
        /* 校验错误，说明之前找到的帧头只是偶然出现的废数据*/
        parser.r_oft = (parser.r_oft + 1) % PROT_FRAME_LEN_RECV;
    }
    parser.frame_len = 0;
    parser.found_frame_head = 0;

    return frame_type;
}

/**
 * @brief   接收数据处理
 * @param   *data:  要计算的数据的数组.
 * @param   data_len: 数据的大小
 * @return  void.
 */
void protocol_data_recv(uint8_t *data, uint16_t data_len)
{
    recvbuf_put_data(parser.recv_ptr, PROT_FRAME_LEN_RECV, parser.w_oft, data, data_len); // 接收数据
    parser.w_oft = (parser.w_oft + data_len) % PROT_FRAME_LEN_RECV;                       // 计算写偏移
}

/**
 * @brief   初始化接收协议
 * @param   void
 * @return  初始化结果.
 */
int32_t protocol_init(void)
{
    memset(&parser, 0, sizeof(struct prot_frame_parser_t));

    /* 初始化分配数据接收与解析缓冲区*/
    parser.recv_ptr = recv_buf;

    return 0;
}

/**
 * @brief   接收的数据处理
 * @param   void
 * @return  -1：没有找到一个正确的命令.
 */
int8_t receiving_process(void)
{
    uint8_t frame_data[128];     // 要能放下最长的帧
    uint16_t frame_len = 0;      // 帧长度
    uint8_t cmd_type = CMD_NONE; // 命令类型
    packet_head_t packet;

    while (1)
    {
        cmd_type = protocol_frame_parse(frame_data, &frame_len);
        switch (cmd_type)
        {
        case CMD_NONE:
        {
            return -1;
        }

        case SET_P_I_D_CMD:
        {
            type_cast_t temp_p, temp_i, temp_d;

            packet.ch = frame_data[CHX_INDEX_VAL];

            temp_p.i = COMPOUND_32BIT(&frame_data[13]);
            temp_i.i = COMPOUND_32BIT(&frame_data[17]);
            temp_d.i = COMPOUND_32BIT(&frame_data[21]);

            if (packet.ch == CURVES_CH1)
            {
                // 设置 P I D
                DEBUG("Set Target:%f, %f, %f\n", temp_p.f, temp_i.f, temp_d.f);
                PID_Set_Motor_Parm(MAX_MOTOR, temp_p.f, temp_i.f, temp_d.f);
            }
            else if (packet.ch == CURVES_CH2)
            {
                // 设置 P I D
                DEBUG("Set Target:%f, %f, %f\n", temp_p.f, temp_i.f, temp_d.f);
            }
        }
        break;

        case SET_TARGET_CMD:
        {
            int target_temp = COMPOUND_32BIT(&frame_data[13]); // 得到数据
            packet.ch = frame_data[CHX_INDEX_VAL];

            /* 只设置速度的目标值，电流的目标置是由速度pid的输出决定的 */
            if (packet.ch == CURVES_CH1)
            {
                // set_pid_target(&pid_location, target_temp); // 设置目标值
                DEBUG("Set Target:%d\n", target_temp);
                // Motion_Set_Speed(0, target_temp, 0, 0);
                g_speed = target_temp;
                PID_Set_Motor_Target(MAX_MOTOR, g_speed);
            }
        }
        break;

        case START_CMD:
        {
            Motion_Set_Speed(0, 0, 0, 0);
            PID_Set_Motor_Target(MAX_MOTOR, g_speed);
            DEBUG("START MOTOR\n");
            g_start_tool = 1;
        }
        break;

        case STOP_CMD:
        {
            // 停止电机
            DEBUG("STOP MOTOR\n");
            Motion_Stop(STOP_BRAKE);
            g_start_tool = 0;
        }
        break;

        case RESET_CMD:
        {
            // 复位系统
            DEBUG("RESET MCU\n");
            Bsp_Reset_MCU();
        }
        break;

        // case SET_PERIOD_CMD: // 周期设置
        // {
        //     ;
        // }
        // break;

        default:
            return -1;
        }
    }
}

/**
  * @brief 设置上位机的值
  * @param cmd：命令
  * @param ch: 曲线通道
  * @param data：参数指针
  * @param num：参数个数
  * @retval 无
  */
void set_computer_value(uint8_t cmd, uint8_t ch, void *data, uint8_t num)
{
    uint8_t sum = 0; // 校验和
    num *= 4;        // 一个参数 4 个字节

    static packet_head_t set_packet;

    set_packet.head = FRAME_HEADER; // 包头 0x59485A53
    set_packet.ch = ch;             // 设置通道
    set_packet.len = 0x0B + num;    // 包长
    set_packet.cmd = cmd;           // 设置命令

    sum = check_sum(0, (uint8_t *)&set_packet, sizeof(set_packet)); // 计算包头校验和
    sum = check_sum(sum, (uint8_t *)data, num);                     // 计算参数校验和

    USART1_Send_ArrayU8((uint8_t *)&set_packet, sizeof(set_packet)); // 发送数据头
    USART1_Send_ArrayU8((uint8_t *)data, num);                       // 发送参数
    USART1_Send_ArrayU8((uint8_t *)&sum, sizeof(sum));               // 发送校验和
}

uint8_t start_tool(void)
{
    return g_start_tool;
}

/**********************************************************************************************/
