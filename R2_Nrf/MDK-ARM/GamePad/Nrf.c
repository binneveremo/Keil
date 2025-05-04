#include "Nrf.h"
#include "math.h"




//当一个函数需要支持多种数据类型时，可以使用宏定义来生成多个函数
#define MATHBSP_TYPEBASED_FUNC_DEFINITION_LIST(definition, func)                            \
    definition(double,      func##_double)                                                  \
    definition(float,       func##_float)                                                   \
    definition(uint64_t,    func##_uint64)                                                  \
    definition(int64_t,     func##_int64)                                                   \
    definition(uint32_t,    func##_uint32)                                                  \
    definition(int32_t,     func##_int32)                                                   \
    definition(uint16_t,    func##_uint16)                                                  \
    definition(int16_t,     func##_int16)                                                   \
    definition(uint8_t,     func##_uint8)                                                   \
    definition(int8_t,      func##_int8)
#define MATHBSP_TYPEBASED_INTEGER_FUNC_DEFINITION_LIST(definition, func)                    \
    definition(uint64_t,    func##_uint64)                                                  \
    definition(int64_t,     func##_int64)                                                   \
    definition(uint32_t,    func##_uint32)                                                  \
    definition(int32_t,     func##_int32)                                                   \
    definition(uint16_t,    func##_uint16)                                                  \
    definition(int16_t,     func##_int16)                                                   \
    definition(uint8_t,     func##_uint8)                                                   \
    definition(int8_t,      func##_int8)


//归并排序
#define DEFINE_MERGESORT_FUNC(type, func_name)                                              \
void _##func_name##_Merge(type* source, type* buff, uint32_t start, uint32_t mid, uint32_t end) {   \
    uint32_t i = start, j = mid + 1, k = start;                                             \
    while (i <= mid && j <= end) {                                                          \
        if (source[i] <= source[j]) buff[k++] = source[i++];                                \
        else buff[k++] = source[j++];                                                       \
    }                                                                                       \
    while (i <= mid) buff[k++] = source[i++];                                               \
    while (j <= end) buff[k++] = source[j++];                                               \
    for (i = start; i <= end; i++) source[i] = buff[i];                                     \
}                                                                                           \
void _##func_name##_Sort(type* source, type* buff, uint32_t start, uint32_t end) {          \
    if(start >= end) return;                                                                \
    uint32_t mid = (start + end) / 2;                                                       \
    _##func_name##_Sort(source, buff, start, mid);                                          \
    _##func_name##_Sort(source, buff, mid + 1, end);                                        \
    _##func_name##_Merge(source, buff, start, mid, end);                                    \
}                                                                                           \
type* func_name(type* source, type* buff, uint32_t len) {                                   \
    if(source != NULL && buff!= NULL && len > 1)                                            \
        _##func_name##_Sort(source, buff, 0, len - 1);                                      \
    return buff;                                                                            \
}
MATHBSP_TYPEBASED_FUNC_DEFINITION_LIST(DEFINE_MERGESORT_FUNC, MathBsp_MergeSort)

//快速排序
#define DEFINE_QUICKSORT_FUNC(type, func_name)                                              \
uint32_t _##func_name##_Partition(type* source, uint32_t low, uint32_t high) {              \
    type pivot = source[high];                                                              \
    uint32_t i = (low - 1);                                                                 \
    for (uint32_t j = low; j <= high - 1; j++) {                                            \
        if (source[j] < pivot) {                                                            \
            i++;                                                                            \
            type t = source[i];                                                             \
            source[i] = source[j];                                                          \
            source[j] = t;                                                                  \
        }                                                                                   \
    }                                                                                       \
    type t = source[i + 1];                                                                 \
    source[i + 1] = source[high];                                                           \
    source[high] = t;                                                                       \
    return (i + 1);                                                                         \
}                                                                                           \
void _##func_name##_Sort(type* source, uint32_t low, uint32_t high) {                       \
    if (low < high) {                                                                       \
        uint32_t pi = _##func_name##_Partition(source, low, high);                          \
        _##func_name##_Sort(source, low, pi - 1);                                           \
        _##func_name##_Sort(source, pi + 1, high);                                          \
    }                                                                                       \
}                                                                                           \
type* func_name(type* source, uint32_t len) {                                               \
    _##func_name##_Sort(source, 0, len - 1);                                                \
    return source;                                                                          \
}
MATHBSP_TYPEBASED_FUNC_DEFINITION_LIST(DEFINE_QUICKSORT_FUNC, MathBsp_QuickSort)


//堆排序
#define DEFINE_HEAPSORT_FUNC(type, func_name)                                               \
void _##func_name##_Heapify(type* source, uint32_t len, uint32_t i) {                       \
    uint32_t largest = i;                                                                   \
    uint32_t l = 2 * i + 1;                                                                 \
    uint32_t r = 2 * i + 2;                                                                 \
    if (l < len && source[l] > source[largest]) largest = l;                                \
    if (r < len && source[r] > source[largest]) largest = r;                                \
    if (largest != i) {                                                                     \
        type t = source[i];                                                                 \
        source[i] = source[largest];                                                        \
        source[largest] = t;                                                                \
        _##func_name##_Heapify(source, len, largest);                                       \
    }                                                                                       \
}                                                                                           \
type* func_name(type* source, uint32_t len) {                                               \
    for (int i = len / 2 - 1; i >= 0; i--) _##func_name##_Heapify(source, len, i);          \
    for (int i = len - 1; i > 0; i--) {                                                     \
        type t = source[0];                                                                 \
        source[0] = source[i];                                                              \
        source[i] = t;                                                                      \
        _##func_name##_Heapify(source, i, 0);                                               \
    }                                                                                       \
    return source;                                                                          \
}
MATHBSP_TYPEBASED_FUNC_DEFINITION_LIST(DEFINE_HEAPSORT_FUNC, MathBsp_HeapSort)

//**位运算相关**/ 

//获取一个数的二进制表示中最高位的1的位置
#define DEFINE_GETHIGHESTBIT_FUNC(type, func_name)                                          \
uint32_t func_name(type num) {                                                              \
    type mask = 1;                                                                          \
    uint32_t pos = 0;                                                                       \
    for (uint32_t i = 0; i < sizeof(type) * 8; i++) {                                       \
        if (num & (type)(mask << i)) pos = i + 1;                                           \
    }                                                                                       \
    return pos;                                                                             \
}
MATHBSP_TYPEBASED_INTEGER_FUNC_DEFINITION_LIST(DEFINE_GETHIGHESTBIT_FUNC, MathBsp_GetHighestBit)
//获取一个数的二进制表示中最低位的1的位置
#define DEFINE_GETLOWESTBIT_FUNC(type, func_name)                                           \
uint32_t func_name(type num) {                                                              \
    type mask = 1;                                                                          \
    uint32_t pos = 0;                                                                       \
    for (uint32_t i = 0; i < sizeof(type) * 8; i++) {                                       \
        if (num & (type)(mask << i)){ pos = i + 1; break; }                                 \
    }                                                                                       \
    return pos;                                                                             \
}
MATHBSP_TYPEBASED_INTEGER_FUNC_DEFINITION_LIST(DEFINE_GETLOWESTBIT_FUNC, MathBsp_GetLowestBit)

//将一个整数的第n位设置为1/0
#define DEFINE_SETBIT_FUNC(type, func_name)                                                 \
type func_name(type num, uint32_t n, bool bitstate) {                                       \
    if (n > sizeof(type) * 8 || n == 0) return num;                                         \
    type mask = 1;                                                                          \
    if (bitstate) return num | (type)(mask << (n - 1));                                     \
    else return num & ~(type)(mask << (n - 1));                                             \
}
MATHBSP_TYPEBASED_INTEGER_FUNC_DEFINITION_LIST(DEFINE_SETBIT_FUNC, MathBsp_SetBit)      

//计算一个整数的二进制表示中1的个数
#define DEFINE_COUNTBIT_FUNC(type, func_name)                                               \
uint32_t func_name(type num) {                                                              \
    uint32_t count = 0;                                                                     \
    for (uint32_t i = 0; i < sizeof(type) * 8; i++) {                                       \
        if (num & (type)((type)1 << i)) count++;											\
    }                                                                                       \
    return count;                                                                           \
}
MATHBSP_TYPEBASED_INTEGER_FUNC_DEFINITION_LIST(DEFINE_COUNTBIT_FUNC, MathBsp_CountBit)

//获取一个二进制整数的第n位是1还是0
#define DEFINE_GETBIT_FUNC(type, func_name)                                                 \
bool func_name(type num, uint32_t n) {                                                      \
    if (n > sizeof(type) * 8 || n == 0) return false;                                       \
    type mask = 1;                                                                          \
    return num & (type)(mask << (n - 1));                                                   \
}
MATHBSP_TYPEBASED_INTEGER_FUNC_DEFINITION_LIST(DEFINE_GETBIT_FUNC, MathBsp_GetBit)

































//nRF24L01寄存器操作命令
#define NRF_CMD_READ_REG                0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_CMD_WRITE_REG               0x20  //写配置寄存器,低5位为寄存器地址
#define NRF_CMD_READ_RX_PAYLOAD         0x61  //读RX有效数据,1~32字节
#define NRF_CMD_WRITE_TX_PAYLOAD        0xA0  //写TX有效数据,1~32字节
#define NRF_CMD_FLUSH_TX                0xE1  //清除TX FIFO寄存器.发射模式下用
#define NRF_CMD_FLUSH_RX                0xE2  //清除RX FIFO寄存器.接收模式下用，,在传输应答信号过程中不应执行此指令
#define NRF_CMD_REUSE_TX_PAYLOAD        0xE3  //数据重发,重新使用上一包数据,CE为高,数据包被不断发送
#define NRF_CMD_READ_RX_PAYLOAD_WID     0x60  //读RX FIFO顶层R_RX_PLOAD载荷长度,若超出32字节则清除RX FIFO
#define NRF_CMD_WRITE_ACK_PAYLOAD       0xA8  //RX模式下,将PIPEppp(ppp的范围为000~101)中的AcK包与载荷一起写入,最多可以有3个挂起的ACK包.
#define NRF_CMD_WRITE_TX_PAYLOAD_NOACK  0xB0  //发送数据,但禁止自动应答,发射模式用
#define NRF_CMD_NOP                     0xFF  //空操作,可以用来读状态寄存器

#define NRF_REG_CONFIG                  0x00  //配置收发状态,CRC校验模式以及收发状态响应方式
#define NRF_REG_EN_AA                   0x01  //自动应答功能设置
#define NRF_REG_EN_RXADDR               0x02  //可用信道设置
#define NRF_REG_SETUP_AW                0x03  //收发地址宽度设置
#define NRF_REG_SETUP_RETR              0x04  //自动重发功能设置
#define NRF_REG_RF_CH                   0x05  //工作频率设置
#define NRF_REG_RF_SETUP                0x06  //发射速率、功耗功能设置0b0010 0111
#define NRF_REG_STATUS                  0x07  //状态寄存器（写1清除对应的中断）
#define NRF_REG_OBSERVE_TX              0x08  //发送监测功能
#define NRF_REG_CD                      0x09  //地址检测
#define NRF_REG_RX_ADDR_P0              0x0A  //频道0接收数据地址（通道0的地址 注： 位39到位0可以随意改 ）
#define NRF_REG_RX_ADDR_P1              0x0B  //频道1接收数据地址（通道1的地址 注： 位39到位0可以随意改 ）
#define NRF_REG_RX_ADDR_P2              0x0C  //频道2接收数据地址（通道2的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P3              0x0D  //频道3接收数据地址（通道3的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P4              0x0E  //频道4接收数据地址（通道4的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_RX_ADDR_P5              0x0F  //频道5接收数据地址（通道5的地址，注：位39到位8同通道1,位7到位0可以随意改）
#define NRF_REG_TX_ADDR                 0x10  //发送地址寄存器（发射地址，注： 位39到位0可以随意改）
#define NRF_REG_RX_PW_P0                0x11  //接收数据通道0有效数据宽度
#define NRF_REG_RX_PW_P1                0x12  //接收数据通道1有效数据宽度
#define NRF_REG_RX_PW_P2                0x13  //接收数据通道2有效数据宽度
#define NRF_REG_RX_PW_P3                0x14  //接收数据通道3有效数据宽度
#define NRF_REG_RX_PW_P4                0x15  //接收数据通道4有效数据宽度
#define NRF_REG_RX_PW_P5                0x16  //接收数据通道5有效数据宽度
#define NRF_REG_FIFO_STATUS             0x17  //FIFO栈入栈出状态寄存器设置（只读）
#define NRF_REG_DYNPD                   0x1C  //nrf24l01+ 动态数据长度寄存器
#define NRF_REG_FEATURE                 0x1D  //nrf24l01+ 功能寄存器

#define NRF_WID_RX_ADR             5        //5字节的地址宽度
#define NRF_WID_TX_ADR             5        //5字节的地址宽度
#define NRF_WID_RX_PLOAD           32       //32字节的用户数据宽度
#define NRF_WID_ACK_PAYLOAD        5        //5字节的用户数据宽度


void _Nrf_CE_Low(Nrf_t* nrf)//将CE置低，向nRF24L01发送控制信号
{
    HAL_GPIO_WritePin(nrf->ce.port, nrf->ce.pin, GPIO_PIN_RESET);
}
void _Nrf_CE_High(Nrf_t* nrf)//将CE置高，向nRF24L01发送控制信号
{
    HAL_GPIO_WritePin(nrf->ce.port, nrf->ce.pin, GPIO_PIN_SET);
}
void _Nrf_CS_Low(Nrf_t* nrf)//使能片选，开启SPI传输
{
    HAL_GPIO_WritePin(nrf->nss.port, nrf->nss.pin, GPIO_PIN_RESET);
}
void _Nrf_CS_High(Nrf_t* nrf)//失能片选，关闭SPI传输
{
    HAL_GPIO_WritePin(nrf->nss.port, nrf->nss.pin, GPIO_PIN_SET);
}
void _Nrf_RST_Low(Nrf_t* nrf)//将RST置低，复位nRF24L01
{
    HAL_GPIO_WritePin(nrf->rst.port, nrf->rst.pin, GPIO_PIN_RESET);
}
void _Nrf_RST_High(Nrf_t* nrf)//将RST置高，复位nRF24L01
{
    HAL_GPIO_WritePin(nrf->rst.port, nrf->rst.pin, GPIO_PIN_SET);
}
GPIO_PinState _Nrf_IRQ_Read(Nrf_t* nrf)//读取IRQ引脚的电平
{   
    return HAL_GPIO_ReadPin(nrf->irq.port, nrf->irq.pin);
}
uint8_t _Nrf_SPI_ByteExchange(Nrf_t* nrf, uint8_t data)//SPI字节交换函数
{
    uint8_t rx_data;
	HAL_SPI_TransmitReceive(nrf->hspi, &data, &rx_data, 1, 100);
    return rx_data;
}
/*-----------------------SPI接口的命令设置-------------------------*/
uint8_t _Nrf_Write_Register(Nrf_t* nrf, uint8_t reg_offset, uint8_t data)//写寄存器
{
    uint8_t status;
    _Nrf_CS_Low(nrf);
    status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_WRITE_REG | reg_offset);
    _Nrf_SPI_ByteExchange(nrf, data);
    _Nrf_CS_High(nrf);
    return status;
}
uint8_t _Nrf_Read_Register(Nrf_t* nrf, uint8_t reg_offset)//读寄存器
{
    uint8_t reg_val;
    _Nrf_CS_Low(nrf);
    _Nrf_SPI_ByteExchange(nrf, NRF_CMD_READ_REG | reg_offset);
    reg_val = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_NOP);
    _Nrf_CS_High(nrf);
    return reg_val;
}
uint8_t _Nrf_Write_TX_Payload(Nrf_t* nrf, uint8_t *pBuf, uint8_t len)//写TX Payload
{
    uint8_t status;
    _Nrf_CS_Low(nrf);
    status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_WRITE_TX_PAYLOAD);   //SPI发送写TX_PAYLOAD的命令
    while (len--)
    {
        _Nrf_SPI_ByteExchange(nrf, *pBuf++); //SPI发送数据，依次写入到E01C的TX FIFO
    }
    _Nrf_CS_High(nrf);
    return status;
}
uint8_t _Nrf_Read_RX_Payload(Nrf_t* nrf, uint8_t *pBuf, uint8_t len)//读RX Payload
{
    uint8_t status;
    _Nrf_CS_Low(nrf);
    status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_READ_RX_PAYLOAD);    //SPI发送读RX_PAYLOAD的命令
    while (len--)
    {
        *pBuf++ = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_NOP);    //SPI读取数据，依次读出E01C的RX FIFO
    }
    _Nrf_CS_High(nrf);
    return status;
}
uint8_t _Nrf_Flush_TX(Nrf_t* nrf)//清空TX FIFO
{
    uint8_t status;
    _Nrf_CS_Low(nrf);
    status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_FLUSH_TX);   //SPI发送清空TX FIFO的命令
    _Nrf_CS_High(nrf);
    return status;
}
uint8_t _Nrf_Flush_RX(Nrf_t* nrf)//清空RX FIFO
{
    uint8_t status;
    _Nrf_CS_Low(nrf);
    status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_FLUSH_RX);   //SPI发送清空RX FIFO的命令
    _Nrf_CS_High(nrf);
    return status;
}
uint8_t _Nrf_Reuse_TX_Payload(Nrf_t* nrf)//重复使用上一包数据
{
    uint8_t status;
    _Nrf_CS_Low(nrf);
    status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_REUSE_TX_PAYLOAD);    //SPI发送重复使用上一包数据的命令
    _Nrf_CS_High(nrf);
    return status;
}
uint8_t _Nrf_Nop(Nrf_t* nrf)
{
    _Nrf_CS_Low(nrf);
    uint8_t status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_NOP);    //SPI发送重复使用上一包数据的命令
    _Nrf_CS_High(nrf);
    return status;
}
uint8_t _Nrf_Read_Buffer(Nrf_t* nrf, uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    _Nrf_CS_Low(nrf);                               //使能SPI传输
    uint8_t status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_READ_REG | reg);        //发送寄存器值(位置),并读取状态值
    while(len--)                            //读取数据
    {
        *pBuf++ = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_NOP);  //读出数据
    }
    _Nrf_CS_High(nrf);                               //关闭SPI传输
    return status;   
}
uint8_t _Nrf_Write_Buffer(Nrf_t* nrf, uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    _Nrf_CS_Low(nrf);                               //使能SPI传输
    uint8_t status = _Nrf_SPI_ByteExchange(nrf, NRF_CMD_WRITE_REG | reg);        //发送寄存器值(位置),并读取状态值
    while(len--)                            //读取数据
    {
        _Nrf_SPI_ByteExchange(nrf, *pBuf++);         //读出数据
    }
    _Nrf_CS_High(nrf);                               //关闭SPI传输
    return status;   
}
/*----------------------------------寄存器----------------------------------*/
//配置寄存器CONFIG
typedef struct {
    uint8_t prim_rx : 1;      //（第0位）1:接收模式, 0:发射模式
    uint8_t pwr_up : 1;       //（第1位）1:上电, 0:掉电
    uint8_t crc0 : 1;         //（第2位）CRC校验的模式：0：1字节(8位校验)，1：2字节(16位校验)
    uint8_t en_crc : 1;       //（第3位）是1否0使能CRC校验。如果EN_AA中任意一位被使能，CRC也必须被使能(1)
    uint8_t mask_max_rt : 1;  //（第4位）可屏蔽中断MAX_RT, 1:IRQ引脚不会产生MAX_RT中断 0:MAX_RT中断产生时IRQ引脚电平为低
    uint8_t mask_tx_ds : 1;   //（第5位）可屏蔽中断TX_DS, 1:IRQ引脚不会产生TX_DS中断 0:TX_DS中断产生时IRQ引脚电平为低
    uint8_t mask_rx_dr : 1;   //（第6位）可屏蔽中断RX_RD, 1:IRQ引脚不会产生RX_RD中断 0:RX_RD中断产生时IRQ引脚电平为低
    uint8_t reserved : 1;    //（第7位）保留位，不用
} _NRF_REG_CONFIG;//配置寄存器结构体
//自动应答使能寄存器EN_AA，此功能禁止后可与nRF2401通信
typedef struct {
    uint8_t enaa_p0 : 1;      //（第0位）通道0自动应答使能
    uint8_t enaa_p1 : 1;      //（第1位）通道1自动应答使能
    uint8_t enaa_p2 : 1;      //（第2位）通道2自动应答使能
    uint8_t enaa_p3 : 1;      //（第3位）通道3自动应答使能
    uint8_t enaa_p4 : 1;      //（第4位）通道4自动应答使能
    uint8_t enaa_p5 : 1;      //（第5位）通道5自动应答使能
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _NRF_REG_EN_AA;//自动应答使能寄存器结构体
//接收地址使能寄存器EN_RXADDR
typedef struct {
    uint8_t erx_p0 : 1;       //（第0位）通道0接收使能
    uint8_t erx_p1 : 1;       //（第1位）通道1接收使能
    uint8_t erx_p2 : 1;       //（第2位）通道2接收使能
    uint8_t erx_p3 : 1;       //（第3位）通道3接收使能
    uint8_t erx_p4 : 1;       //（第4位）通道4接收使能
    uint8_t erx_p5 : 1;       //（第5位）通道5接收使能
    uint8_t reserved : 2;     //（第6、7位）保留位，不用
} _NRF_REG_EN_RXADDR;//接收地址使能寄存器结构体
//地址宽度寄存器SETUP_AW
typedef struct {
    uint8_t aw : 2;           //（第0、1位）RX/TX地址字段宽度设置：00：无效，01：3字节，10：4字节，11：5字节
    uint8_t reserved : 6;     //（第2-7位）保留位，不用
} _NRF_REG_SETUP_AW;//地址宽度寄存器结构体
//自动重发延时寄存器SETUP_RETR
typedef struct {
    uint8_t arc : 4;          //（第0-3位）自动重发计数器，最大重发次数为0-15
    uint8_t ard : 4;          //（第4-7位）自动重发延时，0：250us，1：500us，2：750us，3：1000us，4：1250us，5：1500us，6：1750us，7：2000us，8：2250us，9：2500us，10：2750us，11：3000us，12：3250us，13：3500us，14：3750us，15：4000us (+-86us)
} _NRF_REG_SETUP_RETR;//自动重发延时寄存器结构体
//射频通道寄存器RF_CH
typedef struct {
    uint8_t rf_ch : 7;        //（第0-6位）射频通道选择，2400+RF_CH MHz
    uint8_t reserved : 1;     //（第7位）保留位，不用
} _NRF_REG_RF_CH;//射频通道寄存器结构体
//射频设置寄存器RF_SETUP
typedef struct {
    uint8_t lna_hcurr : 1;    //（第0位）低噪声放大器增益，1：高增益，0：低增益
    uint8_t rf_pwr : 2;       //（第1、2位）发射功率，00：-18dBm，01：-12dBm，10：-6dBm，11：0dBm
    uint8_t rf_dr : 1;   //（第3位）空中速率，1：2Mbps，0：1Mbps
    uint8_t pll_lock : 1;     //（第4位）锁定PLL，1：锁定，0：未锁定
    uint8_t reserved : 3;     //（第5-7位）保留位，不用
} _NRF_REG_RF_SETUP;//射频设置寄存器结构体
//状态寄存器STATUS
typedef struct {
    uint8_t tx_full : 1;      //（第0位）TX FIFO满标志，1：满，0：未满
    uint8_t rx_p_no : 3;      //（第1-3位）接收数据通道号，000：通道0，001：通道1，010：通道2，011：通道3，100：通道4，101：通道5，110：未使用，111：RX FIFO为空
    uint8_t max_rt : 1;       //（第4位）达到最大重发次数中断，1：达到，0：未达到，如果MAX_RT被置位，则必须被清除才能继续发送数据包
    uint8_t tx_ds : 1;        //（第5位）数据发送完成中断，1：完成，0：未完成
    uint8_t rx_dr : 1;        //（第6位）接收到数据中断，1：接收到数据，0：未接收到数据
    uint8_t reserved : 1;     //（第7位）保留位，不用
} _NRF_REG_STATUS;//状态寄存器结构体
//发送检测寄存器OBSERVE_TX
typedef struct {
    uint8_t arc_cnt : 4;      //（第0-3位）自动重发计数器，最大重发次数为0-15
    uint8_t plos_cnt : 4;     //（第4-7位）丢包计数器。当写RF_CH寄存器时，该值被清零，当发射超过15次时，此寄存器溢出清零
} _NRF_REG_OBSERVE_TX;//发送检测寄存器结构体
//载波检测寄存器CD
typedef struct {
    uint8_t cd : 1;           //（第0位）载波检测，1：接收到的信号强度大于-64dBm，0：接收到的信号强度小于-64dBm
    uint8_t reserved : 7;     //（第1-7位）保留位，不用
} _NRF_REG_CD;//载波检测寄存器结构体
//接收数据通道0地址寄存器RX_ADDR_P0
typedef struct {
    uint8_t rx_addr_p0[5];          //（第0-39位）接收数据通道0地址，低字节在前，所写字节数由SETUP_AW寄存器决定
} _NRF_REG_RX_ADDR_P0;//接收数据通道0地址寄存器结构体
//接收数据通道1地址寄存器RX_ADDR_P1
typedef struct {
    uint8_t rx_addr_p1[5];          //（第0-39位）接收数据通道1地址，低字节在前，所写字节数由SETUP_AW寄存器决定
} _NRF_REG_RX_ADDR_P1;//接收数据通道1地址寄存器结构体
//接收数据通道2地址寄存器RX_ADDR_P2
typedef struct {
    uint8_t rx_addr_p2;             //（第0-7位）接收数据通道2地址，最低字节可设置，高字节必须与RX_ADDR_P1[39:8]相等
} _NRF_REG_RX_ADDR_P2;//接收数据通道2地址寄存器结构体
//接收数据通道3地址寄存器RX_ADDR_P3
typedef struct {
    uint8_t rx_addr_p3;             //（第0-7位）接收数据通道3地址，最低字节可设置，高字节必须与RX_ADDR_P1[39:8]相等
} _NRF_REG_RX_ADDR_P3;//接收数据通道3地址寄存器结构体
//接收数据通道4地址寄存器RX_ADDR_P4
typedef struct {
    uint8_t rx_addr_p4;             //（第0-7位）接收数据通道4地址，最低字节可设置，高字节必须与RX_ADDR_P1[39:8]相等
} _NRF_REG_RX_ADDR_P4;//接收数据通道4地址寄存器结构体
//接收数据通道5地址寄存器RX_ADDR_P5
typedef struct {
    uint8_t rx_addr_p5;             //（第0-7位）接收数据通道5地址，最低字节可设置，高字节必须与RX_ADDR_P1[39:8]相等
} _NRF_REG_RX_ADDR_P5;//接收数据通道5地址寄存器结构体
//发送地址寄存器TX_ADDR
typedef struct {
    uint8_t tx_addr[5];             //（第0-39位）发送地址，低字节在前
} _NRF_REG_TX_ADDR;//发送地址寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P0
typedef struct {
    uint8_t rx_pw_p0 : 6;           //（第0-5位）接收数据通道0有效数据宽度，0-32字节
    uint8_t reserved : 2;           //（第6、7位）保留位，不用
} _NRF_REG_RX_PW_P0;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P1
typedef struct {
    uint8_t rx_pw_p1 : 6;           //（第0-5位）接收数据通道1有效数据宽度，0-32字节
    uint8_t reserved : 2;           //（第6、7位）保留位，不用
} _NRF_REG_RX_PW_P1;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P2
typedef struct {
    uint8_t rx_pw_p2    : 6;        //（第0-5位）接收数据通道2有效数据宽度，0-32字节
    uint8_t reserved    : 2;        //（第6、7位）保留位，不用
} _NRF_REG_RX_PW_P2;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P3
typedef struct {
    uint8_t rx_pw_p3    : 6;        //（第0-5位）接收数据通道3有效数据宽度，0-32字节
    uint8_t reserved    : 2;        //（第6、7位）保留位，不用
} _NRF_REG_RX_PW_P3;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P4
typedef struct {
    uint8_t rx_pw_p4    : 6;        //（第0-5位）接收数据通道4有效数据宽度，0-32字节
    uint8_t reserved    : 2;        //（第6、7位）保留位，不用
} _NRF_REG_RX_PW_P4;//接收数据通道有效数据宽度寄存器结构体
//接收数据通道有效数据宽度寄存器RX_PW_P5
typedef struct {
    uint8_t rx_pw_p5    : 6;        //（第0-5位）接收数据通道5有效数据宽度，0-32字节
    uint8_t reserved    : 2;        //（第6、7位）保留位，不用
} _NRF_REG_RX_PW_P5;//接收数据通道有效数据宽度寄存器结构体
//FIFO状态寄存器FIFO_STATUS
typedef struct {
    uint8_t rx_empty    : 1;        //（第0位）RX FIFO为空标志，1：空，0：非空
    uint8_t rx_full     : 1;        //（第1位）RX FIFO满标志，1：满，0：未满
    uint8_t reserved2_3 : 2;        //（第2、3位）保留位，不用
    uint8_t tx_empty    : 1;        //（第4位）TX FIFO为空标志，1：空，0：非空
    uint8_t tx_full     : 1;        //（第5位）TX FIFO满标志，1：满，0：未满
    uint8_t tx_reuse    : 1;        //（第6位）若TX_REUSE = 1则当CE为高电平时不断发送上一包数据。TX_REUSE通过SPI命令REUSE_TX_PL设置，通过W_TX_PAYLOAD或W_TX_PAYLOAD_NOACK清除
    uint8_t reserved7   : 1;        //（第7位）保留位，不用
} _NRF_REG_FIFO_STATUS;//FIFO状态寄存器结构体
typedef struct {
    uint8_t dpl_p0      : 1;        //（第0位）通道0动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p1      : 1;        //（第1位）通道1动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p2      : 1;        //（第2位）通道2动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p3      : 1;        //（第3位）通道3动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p4      : 1;        //（第4位）通道4动态载荷长度，1：使能，0：禁止
    uint8_t dpl_p5      : 1;        //（第5位）通道5动态载荷长度，1：使能，0：禁止
    uint8_t reserved    : 2;        //（第6、7位）保留位，不用
} _NRF_REG_DYNPD;//动态载荷长度寄存器//DPL_P<N>：启用数据通道<N>的动态载荷长度，需要EN_DPL和ENAA_P<N>
typedef struct 
{
    uint8_t en_dyn_ack : 1; //（第0位）允许W_TX_PAYLOAD_NOACK命令
    uint8_t en_ack_pay : 1; //（第1位）允许载荷带ACK
    uint8_t en_dpl : 1;     //（第2位）启用动态载荷长度
    uint8_t reversed : 5;   //（第3-7位）保留位，不用
} _NRF_REG_FEATURE;

uint8_t _Nrf_Read_RegStruct_8bits(Nrf_t* nrf, void* reg_struct, uint8_t reg_offset)//读寄存器结构体
{
    *((uint8_t*)reg_struct) = _Nrf_Read_Register(nrf, reg_offset);
    return *((uint8_t*)reg_struct);
}
uint8_t _Nrf_Write_RegStruct_8bits(Nrf_t* nrf, void* reg_struct, uint8_t reg_offset)//写寄存器结构体
{
    return _Nrf_Write_Register(nrf, reg_offset, *((uint8_t*)reg_struct));
}
uint8_t _Nrf_Read_RegStruct_40bits(Nrf_t* nrf, void* reg_struct, uint8_t reg_offset)//读寄存器结构体
{
    return _Nrf_Read_Buffer(nrf, reg_offset, (uint8_t*)reg_struct, 5);
}
uint8_t _Nrf_Write_RegStruct_40bits(Nrf_t* nrf, void* reg_struct, uint8_t reg_offset)//写寄存器结构体
{
    return _Nrf_Write_Buffer(nrf, reg_offset, (uint8_t*)reg_struct, 5);
}
void _Nrf_ModeSwitch(Nrf_t* nrf, enum Nrf_Mode_t mode)
{
    _Nrf_CE_Low(nrf);
    _NRF_REG_CONFIG config;
    _Nrf_Read_RegStruct_8bits(nrf, &config, NRF_REG_CONFIG);//读取配置寄存器
    if(mode == Nrf_Mode_Transmit)
        config.prim_rx = 0;//切换到发射模式
    else if(mode == Nrf_Mode_Receive)
        config.prim_rx = 1;//切换到接收模式
    _Nrf_Write_RegStruct_8bits(nrf, &config, NRF_REG_CONFIG);//写入配置寄存器
    _Nrf_CE_High(nrf);
}
void _Nrf_Set_Config(Nrf_t* nrf)//配置寄存器
{
    _Nrf_RST_Low(nrf);
    _Nrf_CE_Low(nrf);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_SETUP_AW){.aw = 3}, NRF_REG_SETUP_AW);
    _NRF_REG_RX_ADDR_P1 p1_struct;
    for(uint32_t i = 0; i < 4; i++)//填充P1-P5的高位相同地址
        p1_struct.rx_addr_p1[i + 1] = nrf->address_receive.high_addr[i];
    p1_struct.rx_addr_p1[0] = nrf->address_receive.p1;  //填充P1的低位地址
    _Nrf_Write_RegStruct_40bits(nrf, &p1_struct, NRF_REG_RX_ADDR_P1);//写入P1的地址
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_RX_ADDR_P2){.rx_addr_p2 = nrf->address_receive.p2}, NRF_REG_RX_ADDR_P2);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_RX_ADDR_P3){.rx_addr_p3 = nrf->address_receive.p3}, NRF_REG_RX_ADDR_P3);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_RX_ADDR_P4){.rx_addr_p4 = nrf->address_receive.p4}, NRF_REG_RX_ADDR_P4);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_RX_ADDR_P5){.rx_addr_p5 = nrf->address_receive.p5}, NRF_REG_RX_ADDR_P5);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_EN_AA){.enaa_p0 = 1,.enaa_p1 = 1,.enaa_p2 = 1,.enaa_p3 = 1,.enaa_p4 = 1,.enaa_p5 = 1}, NRF_REG_EN_AA);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_EN_RXADDR){.erx_p0 = 1,.erx_p1 = 1,.erx_p2 = 1,.erx_p3 = 1,.erx_p4 = 1,.erx_p5 = 1}, NRF_REG_EN_RXADDR);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_SETUP_RETR){.arc = 15, .ard = 4}, NRF_REG_SETUP_RETR);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_RF_CH){.rf_ch = nrf->rf_channel}, NRF_REG_RF_CH);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_RF_SETUP){.lna_hcurr = 1, .rf_pwr = 3, .rf_dr = 1}, NRF_REG_RF_SETUP);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_CONFIG){.prim_rx = 1, .pwr_up = 1, .crc0 = 1, .en_crc = 1}, NRF_REG_CONFIG);
    _Nrf_Flush_TX(nrf);
    _Nrf_Flush_RX(nrf);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_DYNPD){.dpl_p0 = 1,.dpl_p1 = 1,.dpl_p2 = 1,.dpl_p3 = 1,.dpl_p4 = 1,.dpl_p5 = 1}, NRF_REG_DYNPD);
    _Nrf_Write_RegStruct_8bits(nrf, &(_NRF_REG_FEATURE){.en_dyn_ack = 0, .en_ack_pay = 1, .en_dpl = 1}, NRF_REG_FEATURE);
    _Nrf_CE_High(nrf);
    nrf->tx_state = Nrf_Transmit_Idle;
    _Nrf_ModeSwitch(nrf, Nrf_Mode_Receive);
}
uint8_t _Nrf_CheckConnectivity(Nrf_t* nrf)
{
	_Nrf_RST_Low(nrf);
	
    _NRF_REG_SETUP_RETR check_reg;
    _Nrf_Read_RegStruct_8bits(nrf, &check_reg, NRF_REG_SETUP_RETR);//读取
    check_reg.ard = ~check_reg.ard;
    uint8_t temp = check_reg.ard;
    _Nrf_Write_RegStruct_8bits(nrf, &check_reg, NRF_REG_SETUP_RETR);
    _Nrf_Read_RegStruct_8bits(nrf, &check_reg, NRF_REG_SETUP_RETR);
    if(check_reg.ard != temp)
        return 0;
    else
    {
        check_reg.ard = ~check_reg.ard;
        _Nrf_Write_RegStruct_8bits(nrf, &check_reg, NRF_REG_SETUP_RETR);
        return 1;
    }
}
void Nrf_Init(Nrf_t* nrf)
{
	_Nrf_RST_Low(nrf);
    while(_Nrf_CheckConnectivity(nrf) == 0);
    _Nrf_Set_Config(nrf);
    _NRF_REG_STATUS status;
    _Nrf_Read_RegStruct_8bits(nrf, &status, NRF_REG_STATUS);//读取配置寄存器
    _Nrf_Write_RegStruct_8bits(nrf, &status, NRF_REG_STATUS);//中断触发时status对应位置1，向对应的标志位写1清除中断标志
    _Nrf_ModeSwitch(nrf, Nrf_Mode_Receive);
}
void _Nrf_CalculateNrfSignalQuality(Nrf_t* nrf)
{
    uint32_t success_cnt = MathBsp_CountBit_uint64(nrf->signal_quality.check_buf[0]) + MathBsp_CountBit_uint64(nrf->signal_quality.check_buf[1]);
    nrf->signal_quality.quality = (float)success_cnt / 128.0;
    if(nrf->signal_quality.quality < 0.7)
        nrf->signal_quality.level = Nrf_SignalQualityLevel_Weak;
    else if(nrf->signal_quality.quality < 0.9)
        nrf->signal_quality.level = Nrf_SignalQualityLevel_Normal;
    else
        nrf->signal_quality.level = Nrf_SignalQualityLevel_Strong;
}

void _Nrf_TransmitFailed(Nrf_t* nrf)
{
    uint64_t op = 1;
    if(nrf->signal_quality.check_index > 63)
    {
        op <<= (nrf->signal_quality.check_index - 64);
        nrf->signal_quality.check_buf[1] &= ~op;
    }
    else
    {
        op <<= nrf->signal_quality.check_index;
        nrf->signal_quality.check_buf[0] &= ~op;
    }
    nrf->signal_quality.check_index = (nrf->signal_quality.check_index + 1) % 128;
    _Nrf_CalculateNrfSignalQuality(nrf);
}
void _Nrf_TransmitSuccess(Nrf_t* nrf)
{
    uint64_t op = 1;
    if(nrf->signal_quality.check_index > 63)
    {
        op <<= (nrf->signal_quality.check_index - 64);
        nrf->signal_quality.check_buf[1] |= op;
    }
    else
    {
        op <<= nrf->signal_quality.check_index;
        nrf->signal_quality.check_buf[0] |= op;
    }
    nrf->signal_quality.check_index = (nrf->signal_quality.check_index + 1) % 128;
    _Nrf_CalculateNrfSignalQuality(nrf);
}
void Nrf_EXTI_Callback(Nrf_t* nrf,uint16_t gpio_pin)
{
    if(gpio_pin != nrf->irq.pin)
        return;
    _NRF_REG_STATUS status;
    _Nrf_Read_RegStruct_8bits(nrf, &status, NRF_REG_STATUS);//读取配置寄存器
    _Nrf_Write_RegStruct_8bits(nrf, &status, NRF_REG_STATUS);//中断触发时status对应位置1，向对应的标志位写1清除中断标志
    _Nrf_ModeSwitch(nrf, Nrf_Mode_Receive);//切换到接收模式
    if(status.rx_dr && status.rx_p_no != 7)//如果成功接收到了数据
    {
        nrf->rx_data[status.rx_p_no].len = _Nrf_Read_Register(nrf, NRF_CMD_READ_RX_PAYLOAD_WID);//读取接收到的数据包长度
        if(nrf->rx_data[status.rx_p_no].len)
        {
            _Nrf_Read_Buffer(nrf, NRF_CMD_READ_RX_PAYLOAD, nrf->rx_data[status.rx_p_no].buf, nrf->rx_data[status.rx_p_no].len);//读取接收到的数据包
            if(nrf->nrf_rx_callback) //如果接收回调函数不为空，则调用回调函数
                nrf->nrf_rx_callback(status.rx_p_no, (void*)nrf->rx_data[status.rx_p_no].buf, nrf->rx_data[status.rx_p_no].len);
            _Nrf_Flush_RX(nrf);
        }
        else//如果数据包长度大于32，则说明数据包有误，应该丢弃
        {
            _Nrf_Flush_RX(nrf);//清空接收缓冲区
        }
    }
    else if(status.tx_ds)//发送成功
    {
        _Nrf_TransmitSuccess(nrf);
        nrf->tx_state = Nrf_Transmit_Success;
        _Nrf_Flush_TX(nrf);//清空发送缓冲区
    }
    else if(status.max_rt)//达到最大重发次数
    {
        _Nrf_TransmitFailed(nrf);
        nrf->tx_state = Nrf_Transmit_Failed;
        _Nrf_Flush_TX(nrf);//清空发送缓冲区
    }
}
void Nrf_Transmit(Nrf_t* nrf, uint8_t* data, uint8_t len)
{
    nrf->tx_state = Nrf_Transmit_Ongoing;
    _Nrf_ModeSwitch(nrf, Nrf_Mode_Transmit);//切换到发射模式
    _Nrf_CE_Low(nrf);
    _Nrf_Write_Buffer(nrf, NRF_REG_TX_ADDR, nrf->address_transmit, NRF_WID_TX_ADR);//写入发送地址
    _Nrf_Write_Buffer(nrf, NRF_REG_RX_ADDR_P0, nrf->address_transmit, NRF_WID_RX_ADR);//P0通道接收地址与发送地址相同，用于接收接收机回传的ACK
    _Nrf_Write_Buffer(nrf, NRF_CMD_WRITE_TX_PAYLOAD, data, len);//写入发送数据包
    _Nrf_CE_High(nrf);
}
void Nrf_SetTransmitAddress(Nrf_t* nrf, uint8_t* transmit_addr)
{
    for(uint32_t i = 0; i < 5; i++)
        nrf->address_transmit[i] = transmit_addr[i];
}   
void Nrf_TurnOff(Nrf_t* nrf)
{
    _Nrf_RST_High(nrf);
}
void Nrf_TurnOn(Nrf_t* nrf)
{
    _Nrf_RST_Low(nrf);
}
void Nrf_Reset(Nrf_t* nrf)
{
    _Nrf_RST_Low(nrf);
    _Nrf_Set_Config(nrf);
    _NRF_REG_STATUS status;
    _Nrf_Read_RegStruct_8bits(nrf, &status, NRF_REG_STATUS);//读取配置寄存器
    _Nrf_Write_RegStruct_8bits(nrf, &status, NRF_REG_STATUS);//中断触发时status对应位置1，向对应的标志位写1清除中断标志
    _Nrf_ModeSwitch(nrf, Nrf_Mode_Receive);
}
