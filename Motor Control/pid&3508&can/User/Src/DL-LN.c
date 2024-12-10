/**
 * @file DL-LN.h
 * @author 万鹏 7415 (2789152534@qq.com)
 * @date 2024-11-28 21:56:15
 * @brief 应用库，无线自组网通讯模块，基于DL-LN32P
 * @version 0.1
 * @note
*/

#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"

/***************************************DL-LN信息读取相关函数****************************************************/
/**
 * @brief 看我！别忘了配置！！！
*/

#define DL_LN_UART &huart1  // DL-LN32P 连接的串口
// 端口定义 尽可能不要冲突
/*外部端口*/
#define READ_PORT 0X90
#define LINK_TEST_PORT 0X85
#define SET_PORT 0X95
/*内部端口*/
#define LOCAL_PORT 0X21
#define LINK_TEST_PORT 0X23

// 特殊信息定义
#define LOCAL_ADDRESS 0X00
#define msg_length_to_PC 20     // 向上位机发送的数据长度，不要太大浪费
#define MAX_PACKET_SIZE 256     // 定义最大包长
#define msg_length_to_send 256   // 向其他模块发送的数据长度，不要太大浪费

#define TEMP 0xAA       // 占位符，存在的意义是填充占位，等待更替
#define CMD_TEMP 0xAA   // 占位符，存在的意义是填充命令位
#define ADR_TEMP 0XAA   // 占位符，存在的意义是填充地址位


// 设置-命令定义
#define SET_ADDRESS_CMD 0x11
#define SET_NETWORK_ID_CMD 0x12
#define SET_CHANNEL_CMD 0x13
#define SET_BAUD_RATE_CMD 0x14
#define RESTART_CMD 0x10

// 设置-模式定义
#define SET_ADDRESS_MODE 0x01
#define SET_NETWORK_ID_MODE 0x02
#define SET_CHANNEL_MODE 0x03
#define SET_BAUD_RATE_MODE 0x04

// 数据接收缓冲区
uint8_t DL_LN_rx_buffer[20];  // 接收缓冲区
uint8_t PC_tx_buffer[100];    // 转发给上位机的数据缓冲区
int read_state = 0;

// 读取命令定义
const uint8_t READ_ADDRESS_CMD[] = {0xFE, 0x05, READ_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, 0x01, 0xFF};
const uint8_t READ_NETWORK_ID_CMD[] = {0xFE, 0x05, READ_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, 0x02, 0xFF};
const uint8_t READ_CHANNEL_CMD[] = {0xFE, 0x05, READ_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, 0x03, 0xFF};
const uint8_t READ_BAUD_RATE_CMD[] = {0xFE, 0x05, READ_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, 0x04, 0xFF};
// 设置命令定义
uint8_t SET_CMD_ADR_ID[] = {0xFE, 0x07, SET_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, CMD_TEMP, TEMP, TEMP, 0xFF};
uint8_t SET_CMD_CH_BPS[] = {0xFE, 0x06, SET_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, CMD_TEMP, TEMP, 0xFF};
// 链路测试命令定义
uint8_t LINK_QUALITY_TEST[] = {0xFE, 0x06, LINK_TEST_PORT, LINK_TEST_PORT, ADR_TEMP, ADR_TEMP, ADR_TEMP, ADR_TEMP, 0xFF}
// 重启命令定义
uint8_t RESTART[] = {0xFE, 0x05, SET_PORT, LOCAL_PORT, LOCAL_ADDRESS, LOCAL_ADDRESS, RESTART_CMD, 0xFF}
// 普通发送信息
unint8_t SEND_MSG[msg_length_to_send];
/**
 * @brief 将波特率代码转换为实际波特率，就是波特率解码，参见手册
 * @param baud_code 波特率代码
 * @return 实际波特率
 * @note 表函数，不引出，外部没用
 */
uint32_t DL_LN_decode_baud_rate(uint8_t baud_code) {
    switch (baud_code) {
        case 0x00: return 2400;
        case 0x01: return 4800;
        case 0x02: return 9600;
        case 0x03: return 14400;
        case 0x04: return 19200;
        case 0x05: return 28800;
        case 0x06: return 38400;
        case 0x07: return 57600;
        case 0x08: return 115200;
        case 0x09: return 230400;
        case 0x0A: return 125000;
        case 0x0B: return 250000;
        case 0x0C: return 500000;
        default: return 0; // 未知波特率，一般不会出现
    }
}

/**
 * @brief 将实际波特率转换为波特率代码，就是波特率编码
 * @param baud_code 实际波特率
 * @return 波特率代码
 * @note 表函数，不引出，外部没用
 */
uint8_t DL_LN_encode_baud_rate(uint32_t baud_rate) {
    switch (baud_rate) {
        case 2400: return 0x00;
        case 4800: return 0x01;
        case 9600: return 0x02;
        case 14400: return 0x03;
        case 19200: return 0x04;
        case 28800: return 0x05;
        case 38400: return 0x06;
        case 57600: return 0x07;
        case 115200: return 0x08;
        case 230400: return 0x09;
        case 125000: return 0x0A;
        case 250000: return 0x0B;
        case 500000: return 0x0C;
        default: return 0xFF; // 未知波特率，表示错误
    }
}


/**
 * @brief 解析模块信息并根据当前状态依次读取和发送相关数据。
 * 
 * 该函数根据`read_state`的值逐步解析模块的波特率、模块地址、网络ID和频道信息。
 * 每读取一个数据后，更新`read_state`并发送相应的读取命令来获取下一个信息。
 */
void DL_LN_parse_module_info() {
    if (read_state == 4)
    {read_state = 0;}   // 这代表上一次读取完了，再读取就要重置为0，重新开始了捏

    switch (read_state) {
        case 0: {
            // 解析波特率
            uint8_t baud_code = DL_LN_rx_buffer[6];
            uint32_t baud_rate = DL_LN_decode_baud_rate(baud_code);
            char baud_rate_message[msg_length_to_PC];
            if (baud_rate != 0) {
                sprintf(baud_rate_message, "Baud Rate: %lu bps\r\n", baud_rate);
            } else {
                sprintf(baud_rate_message, "Baud Rate: Unknown Code 0x%02X\r\n", baud_code);
            }
            HAL_UART_Transmit_IT(DL_LN_UART, (uint8_t*)baud_rate_message, strlen(baud_rate_message));

            // 更新状态为1，准备读取模块地址
            read_state = 1;
            DL_LN_send_command(READ_ADDRESS_CMD, 8);  // 继续发送下一个读取命令
            break;
        }
        case 1: {
            // 解析模块地址
            uint16_t module_address = DL_LN_rx_buffer[7] | (DL_LN_rx_buffer[8] << 8);
            char address_message[msg_length_to_PC];
            sprintf(address_message, "Module Address: 0x%04X\r\n", module_address);
            HAL_UART_Transmit_IT(DL_LN_UART, (uint8_t*)address_message, strlen(address_message));

            // 更新状态为2，准备读取网络ID
            read_state = 2;
            DL_LN_send_command(READ_NETWORK_ID_CMD, 8);  // 继续发送下一个读取命令
            break;
        }
        case 2: {
            // 解析网络ID
            uint16_t network_id = DL_LN_rx_buffer[7] | (DL_LN_rx_buffer[8] << 8);
            char network_id_message[msg_length_to_PC];
            sprintf(network_id_message, "Network ID: 0x%04X\r\n", network_id);
            HAL_UART_Transmit_IT(DL_LN_UART, (uint8_t*)network_id_message, strlen(network_id_message));

            // 更新状态为3，准备读取频道
            read_state = 3;
            DL_LN_send_command(READ_CHANNEL_CMD, 8);  // 继续发送下一个读取命令
            break;
        }
        case 3: {
            // 解析频道
            uint8_t channel = DL_LN_rx_buffer[7];
            char channel_message[msg_length_to_PC];
            sprintf(channel_message, "Channel: 0x%02X\r\n", channel);
            HAL_UART_Transmit_IT(DL_LN_UART, (uint8_t*)channel_message, strlen(channel_message));

            // 更新状态为4，表示完成所有读取
            read_state = 4;
            break;
        }
        default:
            break;
    }
}


/**
 * @brief 发送命令到UART接口，使用异步传输。
 * 
 * 该函数通过UART接口异步发送给定的命令。它使用`HAL_UART_Transmit_IT`函数来传输命令数据，确保命令在后台被发送。
 * 
 * @param command 发送的命令数据
 * @param length 命令数据的长度
 */
void DL_LN_send_command(const uint8_t *command, uint8_t length) 
{
    HAL_UART_Transmit_IT(DL_LN_UART, (uint8_t *)command, length);  // 使用异步传输
}

/**
 * @brief 发送读取模块信息的命令并启动数据接收。
 * 
 * 该函数依次发送读取模块地址、网络ID、频道和波特率的命令，并启动UART的接收中断来接收返回的数据。
 * 每发送一个命令后，都会通过`HAL_UART_Receive_IT`启动接收，确保能够及时接收数据。
 * 
 * @note 该函数中的`HAL_Delay`用来在发送每个命令后等待一段时间，以确保模块能够处理命令。
 */
void DL_LN_read() {
    // 发送读取模块信息命令
    DL_LN_send_command(READ_ADDRESS_CMD, 8);
    HAL_UART_Receive_IT(DL_LN_UART, DL_LN_rx_buffer, sizeof(DL_LN_rx_buffer));  // 启动接收

    HAL_Delay(5);
    DL_LN_send_command(READ_NETWORK_ID_CMD, 8);
    HAL_UART_Receive_IT(DL_LN_UART, DL_LN_rx_buffer, sizeof(DL_LN_rx_buffer));  // 启动接收

    HAL_Delay(5);
    DL_LN_send_command(READ_CHANNEL_CMD, 8);
    HAL_UART_Receive_IT(DL_LN_UART, DL_LN_rx_buffer, sizeof(DL_LN_rx_buffer));  // 启动接收

    HAL_Delay(5);
    DL_LN_send_command(READ_BAUD_RATE_CMD, 8);
    HAL_UART_Receive_IT(DL_LN_UART, DL_LN_rx_buffer, sizeof(DL_LN_rx_buffer));  // 启动接收
}


/***************************************DL-LN链路质量测试相关函数****************************************************/

/**
 * @brief 解析链路质量信息（RSSI），并将其发送到上位机。
 * 
 * 该函数从接收到的字节流中提取RSSI数据并将其转换为链路质量值。如果接收到的数据为`0x80`，则认为没有数据,存储为0
 * 然后将链路质量信息格式化为字符串，并通过UART异步发送给上位机。
 */
void DL_LN_parse_link_quality()
{
    uint8_t rssi_data = DL_LN_rx_buffer[7];  // 接收到的信号强度指示（RSSI）数据
    int8_t link_quality = (rssi_data != 0x80) ? (int8_t)rssi_data : 0; // 0x80 表示无数据

    // 格式化链路质量信息并发送到上位机
    char message[msg_length_to_PC];
    int len = sprintf(message, "Link Quality: %d\n", link_quality);
    HAL_UART_Transmit_IT(DL_LN_UART, (uint8_t*)message, len);  // 异步发送链路质量信息
}

/**
 * @brief UART接收完成中断回调函数，根据接收到的数据类型进行相应处理。
 * 
 * 该函数会在每次UART接收完成时触发。它首先检查接收到的数据包头，然后根据不同的命令类型解析数据。
 * 如果是链路质量测试数据包，则调用`DL_LN_parse_link_quality`进行解析；如果是模块信息读取数据包，则调用`DL_LN_parse_module_info`进行解析。
 * 在处理完成后，函数继续启动UART接收。
 * 
 * @param huart 指向UART处理结构体的指针
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // 判断是哪一个 UART 收到数据（如果有多个 UART，需要区分）
    if (huart->Instance == DL_LN_UART) {
        // 先检查包头来识别不同的命令类型
        if (DL_LN_rx_buffer[0] == 0xFE) {
            if (DL_LN_rx_buffer[3] == LINK_TEST_PORT) {
                // 链路质量测试数据包
                DL_LN_parse_link_quality();
            } 
            else if (DL_LN_rx_buffer[3] == READ_PORT) {
                // 解析模块信息
                DL_LN_parse_module_info();  // 根据当前状态解析信息
            }
        }

        // 继续接收数据
        HAL_UART_Receive_IT(DL_LN_UART, DL_LN_rx_buffer, sizeof(DL_LN_rx_buffer));  // 继续接收
    }
}

/**
 * @brief 发送链路质量测试命令，测试模块之间的链路质量。
 * 
 * 该函数将两个模块的地址（以小端格式）写入链路质量测试命令数据包，并通过UART接口发送该数据包。
 * 数据包的发送使用`HAL_UART_Transmit_IT`异步传输方式，确保链路质量测试命令被成功传输。
 * 
 * @param module_1_address 第一个模块的地址
 * @param module_2_address 第二个模块的地址
 */
void DL_LN_link_quality_test(uint16_t module_1_address, uint16_t module_2_address) {
    // 发送模块B的地址（小端格式）
    LINK_QUALITY_TEST[4] = module_1_address & 0xFF;        // 小端格式：低字节
    LINK_QUALITY_TEST[5] = (module_1_address >> 8) & 0xFF; // 小端格式：高字节
    
    // 发送模块C的地址（小端格式）
    LINK_QUALITY_TEST[6] = module_2_address & 0xFF;        // 小端格式：低字节
    LINK_QUALITY_TEST[7] = (module_2_address >> 8) & 0xFF; // 小端格式：高字节

    // 发送数据包
    HAL_UART_Transmit_IT(DL_LN_UART, tx_buffer, sizeof(tx_buffer));
}

/***************************************DL-LN信息设置相关函数****************************************************/

/**
 * @brief 异步发送命令数据到 DL-LN 模块。
 * 
 * @param command 指向待发送数据的指针。
 * @param length 数据长度（字节数）。
 */
void DL_LN_send_command(const uint8_t *command, uint8_t length) {
    HAL_UART_Transmit_IT(DL_LN_UART, (uint8_t *)command, length);  // 异步发送
}

/**
 * @brief 设置 DL-LN 模块的各种参数。
 * 
 * @param mode 配置模式，指定要设置的参数类型。
 *             - SET_ADDRESS_MODE: 设置模块地址
 *             - SET_NETWORK_ID_MODE: 设置网络 ID
 *             - SET_CHANNEL_MODE: 设置信道
 *             - SET_BAUD_RATE_MODE: 设置波特率
 * @param param1 参数 1，根据不同 mode 有不同意义：
 *               - 地址或网络 ID 时为 16 位值（低字节为 param1 & 0xFF，高字节为 param1 >> 8）
 *               - 信道时为信道号（单字节）
 * @param param2 参数 2，仅用于设置波特率时传入波特率值（如 9600）。
 */
void DL_LN_set(uint8_t mode, uint16_t param1, uint8_t param2) {
    memset(SET_CMD_ADR_ID, 0, sizeof(SET_CMD_ADR_ID));  // 清空缓冲区
    memset(SET_CMD_CH_BPS, 0, sizeof(SET_CMD_CH_BPS));  // 清空缓冲区

    switch (mode) {
        case SET_ADDRESS_MODE:  // 设置地址
            SET_CMD_ADR_ID[6] = SET_ADDRESS_CMD;  // 命令字
            SET_CMD_ADR_ID[7] = param1 & 0xFF;  // 地址低字节
            SET_CMD_ADR_ID[8] = (param1 >> 8) & 0xFF;  // 地址高字节
            DL_LN_send_command(SET_CMD_ADR_ID, sizeof(SET_CMD_ADR_ID));
            break;

        case SET_NETWORK_ID_MODE:  // 设置网络ID
            SET_CMD_ADR_ID[6] = SET_NETWORK_ID_CMD;  // 命令字
            SET_CMD_ADR_ID[7] = param1 & 0xFF;  // 网络ID低字节
            SET_CMD_ADR_ID[8] = (param1 >> 8) & 0xFF;  // 网络ID高字节
            DL_LN_send_command(SET_CMD_ADR_ID, sizeof(SET_CMD_ADR_ID));
            break;

        case SET_CHANNEL_MODE:  // 设置信道
            SET_CMD_CH_BPS[6] = SET_CHANNEL_CMD;  // 命令字
            SET_CMD_CH_BPS[7] = param1;  // 新信道值
            DL_LN_send_command(SET_CMD_CH_BPS, sizeof(SET_CMD_CH_BPS));
            break;

        case SET_BAUD_RATE_MODE:  // 设置波特率
            uint8_t bps_encoded = DL_LN_encode_baud_rate(param2);
            SET_CMD_CH_BPS[6] = SET_BAUD_RATE_CMD;  // 命令字
            SET_CMD_CH_BPS[7] = bps_encoded;  // 新波特率值
            DL_LN_send_command(SET_CMD_CH_BPS, sizeof(SET_CMD_CH_BPS));
            break;

        default:
            // 如果传入了无效的 mode，什么也不做
            break;
    }
}

/**
 * @brief 发送重启命令以使模块生效最新设置。
 */
void DL_LN_restart() {
    DL_LN_send_command(RESTART, sizeof(RESTART));
}

/***************************************DL-LN信息设置相关函数****************************************************/

/**
 * @brief 封装数据包并发送
 * 
 * @param send_port 发送端口
 * @param recv_port 接收端口
 * @param target_address 目标地址（大端模式输入）例如0x1234
 * @param data 包内容指针
 */
void DL_LN_send_packet(uint8_t send_port, uint8_t recv_port, uint16_t target_address, const uint8_t *data) {
    
    // 数据内容长度
    uint8_t data_length = strlen((const char *)data); // 计算数据长度
    if (data_length > (MAX_PACKET_SIZE - 7)) {
        return; // 数据长度超出限制
    }

    // 初始化数据包缓冲区
    uint8_t SEND_MSG[MAX_PACKET_SIZE];
    memset(SEND_MSG, 0, MAX_PACKET_SIZE);

    // 包头
    SEND_MSG[0] = 0xFE;

    // 数据长度（自动计算从第3位到包尾前一位的字节数）
    uint8_t packet_length = 1 + 1+ 2 + data_length; // 发送端口(1) + 接收端口(1) + 目标地址(2) + 数据长度
    SEND_MSG[1] = packet_length;

    // 发送端口
    SEND_MSG[2] = send_port;

    // 接收端口
    SEND_MSG[3] = recv_port;

    // 目标地址（小端模式存储）
    SEND_MSG[4] = target_address & 0xFF;        // 低字节
    SEND_MSG[5] = (target_address >> 8) & 0xFF; // 高字节

    // 数据内容
    memcpy(&SEND_MSG[6], data, data_length);

    // 包尾
    SEND_MSG[6 + data_length] = 0xFF;

    // 通过 UART 发送数据包
    HAL_UART_Transmit_IT(DL_LN_UART, SEND_MSG, 7 + data_length);
}