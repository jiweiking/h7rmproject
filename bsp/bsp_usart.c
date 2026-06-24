#include "bsp_usart.h"
#include "remote_control.h"
#include "circular_buffer.h"
#include "string.h"


extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart10;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;


/***************中断callback函数***************/
/**
 * @brief 接收函数回调
 * 请在其它文件中定义hook函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size) 
{
  if(huart->Instance==DBUS_UART_OBJ)
    DBUS_IDLERX_HOOK();
  else if(huart->Instance==USART1)
    USART1_IDLERX_HOOK(huart,Size);
  else if(huart->Instance==UART7)
    UART7_IDLERX_HOOK(huart,Size);
  else if(huart->Instance==USART10)
    USART10_IDLERX_HOOK(huart,Size);
  else if(huart->Instance==USART2)
    USART2_IDLERX_HOOK(huart,Size);
  else if(huart->Instance==USART3)
    USART3_IDLERX_HOOK(huart,Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance==DBUS_UART_OBJ)
    DBUS_ERR_HOOK();
  else if(huart->Instance==USART1)
    USART1_ERR_HOOK();
  else if(huart->Instance==UART7)
    UART7_ERR_HOOK();
  else if(huart->Instance==USART10)
    USART10_ERR_HOOK();
  else if(huart->Instance==USART2)
    USART2_ERR_HOOK();
  else if(huart->Instance==USART3)
    USART3_ERR_HOOK();
}

/***************全局设置***************/
#define MAX_RING_BUF_SIZE 4096
#define DMA_DOUBLE_BUFFER_MODE 0


/***************USART1变量***************/
CircBuf_t USART1_RxCBuf,USART1_TxCBuf;/*环形缓冲区句柄*/
#define USART1_BUF_SIZE 1024
/*DMA缓冲区*/
unsigned char USART1_RxBuf0[ USART1_BUF_SIZE ] = {0};
unsigned char USART1_RxBuf1[ USART1_BUF_SIZE ] = {0};
unsigned char USART1_TxBuf[ 300 ] = {0};
/*环形缓冲区*/
unsigned char USART1_TxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};
unsigned char USART1_RxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};


/***************UART7变量***************/
CircBuf_t UART7_RxCBuf,UART7_TxCBuf;/*环形缓冲区句柄*/
#define UART7_BUF_SIZE 1024
/*DMA缓冲区*/
unsigned char UART7_RxBuf0[ UART7_BUF_SIZE ] = {0};
unsigned char UART7_RxBuf1[ UART7_BUF_SIZE ] = {0};
unsigned char UART7_TxBuf[ 300 ] = {0};
/*环形缓冲区*/
unsigned char UART7_TxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};
unsigned char UART7_RxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};


/***************USART10(4pin口1)变量***************/
CircBuf_t USART10_RxCBuf,USART10_TxCBuf;/*环形缓冲区句柄*/
#define USART10_BUF_SIZE 1024
/*DMA缓冲区*/
unsigned char USART10_RxBuf0[ USART10_BUF_SIZE ] = {0};
unsigned char USART10_RxBuf1[ USART10_BUF_SIZE ] = {0};
unsigned char USART10_TxBuf[ 300 ] = {0};
/*环形缓冲区*/
unsigned char USART10_TxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};
unsigned char USART10_RxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};


/***************USART2(RS485通信)变量***************/
CircBuf_t USART2_RxCBuf,USART2_TxCBuf;/*环形缓冲区句柄*/
#define USART2_BUF_SIZE 32
/*DMA缓冲区*/
unsigned char USART2_RxBuf0[ USART2_BUF_SIZE ] = {0};
unsigned char USART2_RxBuf1[ USART2_BUF_SIZE ] = {0};
unsigned char USART2_TxBuf[ 300 ] = {0};
/*环形缓冲区*/
unsigned char USART2_TxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};
unsigned char USART2_RxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};


/***************USART3(RS485通信)变量***************/
CircBuf_t USART3_RxCBuf,USART3_TxCBuf;/*环形缓冲区句柄*/
#define USART3_BUF_SIZE 32
/*DMA缓冲区*/
unsigned char USART3_RxBuf0[ USART3_BUF_SIZE ] = {0};
unsigned char USART3_RxBuf1[ USART3_BUF_SIZE ] = {0};
unsigned char USART3_TxBuf[ 300 ] = {0};
/*环形缓冲区*/
unsigned char USART3_TxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};
unsigned char USART3_RxCBuf_Arr[ MAX_RING_BUF_SIZE ] = {0};


/***************模板***************/
#define SINGLE_BUF_RX_TEMPLATE(huart_ptr,rx_circbuf,buf_ptr,buf_size,Size) \
    CircBuf_Push(&rx_circbuf,buf_ptr,Size);\
    HAL_UARTEx_ReceiveToIdle_DMA(huart_ptr,buf_ptr,buf_size);\

#define TX_CPLT_HOOK(huart,tx_circbuf,buf_ptr)\
    unsigned int buf_len = CircBuf_GetUsedSize(&tx_circbuf);\
    if(buf_len > 0) {\
        uint16_t send_len = CircBuf_Pop(&tx_circbuf, buf_ptr, 300);\
        HAL_UART_Transmit_DMA(&huart, buf_ptr, send_len);\
    }\

/**
 * @brief 发送代码模板,
 * 因tx代码结构相同,直接使用宏来方便修改
 * @date 2024/10/8
 * @param huart uart的handle
 * @param tx_circbuf 发送环形缓冲区
 * @param buf_ptr 发送缓冲区指针
 * @param data
 * @param len 缓冲区大小
 * @note circbuf 先push又pop的操作有些多余,
 * 可以改为传输完成中断装载新数据,send函数里只对tx_circlbuf操作
 */
#define TX_TEMPLATE(huart,tx_circbuf,buf_ptr)\
    unsigned int result = CircBuf_Push(&tx_circbuf, data, len);\
    if(HAL_DMA_STATE_READY == HAL_DMA_GetState(huart.hdmatx)) {\
        uint16_t send_len = CircBuf_Pop(&tx_circbuf, buf_ptr, 300);\
        HAL_UART_Transmit_DMA(&huart, buf_ptr, send_len);\
    }\
    return result;\




/***************USART1函数***************/
/**
 * @brief USART1初始化
 */
void usart1_init(void)
{

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,USART1_RxBuf0,(uint16_t)USART1_BUF_SIZE);
  /*初始化环形缓冲*/
  CircBuf_Init(&USART1_TxCBuf, USART1_TxCBuf_Arr, MAX_RING_BUF_SIZE);  
  CircBuf_Init(&USART1_RxCBuf, USART1_RxCBuf_Arr, MAX_RING_BUF_SIZE);
  HAL_DMA_RegisterCallback((&huart1)->hdmarx, HAL_DMA_XFER_CPLT_CB_ID, USART1_TX_CPLT_HOOK);
  __HAL_DMA_ENABLE((&huart1)->hdmarx);
  __HAL_DMA_ENABLE((&huart1)->hdmatx);
}

/**
 * @brief USART1空闲中断hook函数
 */
void USART1_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size)
{
  SINGLE_BUF_RX_TEMPLATE(huart,USART1_RxCBuf, USART1_RxBuf0, USART1_BUF_SIZE,Size);
}

/**
 * @brief USART1错误中断hook函数
 */
void USART1_ERR_HOOK(void)
{
  memset(USART1_RxBuf0,0,USART1_BUF_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,USART1_RxBuf0,USART1_BUF_SIZE);
}

void USART1_TX_CPLT_HOOK(DMA_HandleTypeDef* hdma)
{
  TX_CPLT_HOOK(huart1,USART1_TxCBuf,USART1_TxBuf);
}

unsigned int USART1_Send(uint8_t *data, unsigned short len)
{
  TX_TEMPLATE(huart1,USART1_TxCBuf,USART1_TxBuf);
}

unsigned int USART1_Recv(unsigned char *data, unsigned short len)
{    
  unsigned int result = 0;

  if(data != NULL)
      result = CircBuf_Pop(&USART1_RxCBuf, data, len);

  return result;
}

unsigned char USART1_At( unsigned short offset)
{
    return CircBuf_At(&USART1_RxCBuf, offset);
}

void USART1_Drop( unsigned short LenToDrop)
{
    CircBuf_Drop(&USART1_RxCBuf, LenToDrop);
}

unsigned int USART1_GetDataCount( void )
{
    return CircBuf_GetUsedSize(&USART1_RxCBuf);
}

void USART1_Free(void)
{
    CircBuf_Free(&USART1_RxCBuf);
}



/***************UART7函数***************/
/**
 * @brief UART7初始化
 */
void uart7_init(void)
{

  HAL_UARTEx_ReceiveToIdle_DMA(&huart7,UART7_RxBuf0,(uint16_t)UART7_BUF_SIZE);
  /*初始化环形缓冲*/
  CircBuf_Init(&UART7_TxCBuf, UART7_TxCBuf_Arr, MAX_RING_BUF_SIZE);  
  CircBuf_Init(&UART7_RxCBuf, UART7_RxCBuf_Arr, MAX_RING_BUF_SIZE);
  HAL_DMA_RegisterCallback((&huart7)->hdmarx, HAL_DMA_XFER_CPLT_CB_ID, UART7_TX_CPLT_HOOK);
  __HAL_DMA_ENABLE((&huart7)->hdmarx);
  __HAL_DMA_ENABLE((&huart7)->hdmatx);
}

/**
 * @brief UART7空闲中断hook函数
 */
void UART7_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size)
{
  SINGLE_BUF_RX_TEMPLATE(huart,UART7_RxCBuf, UART7_RxBuf0, UART7_BUF_SIZE,Size);
}

/**
 * @brief UART7错误中断hook函数
 */
void UART7_ERR_HOOK(void)
{
  memset(UART7_RxBuf0,0,UART7_BUF_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart7,UART7_RxBuf0,UART7_BUF_SIZE);
}
void UART7_TX_CPLT_HOOK(DMA_HandleTypeDef* hdma)
{
  TX_CPLT_HOOK(huart7,UART7_TxCBuf,UART7_TxBuf);
}

unsigned int UART7_Send(uint8_t *data, unsigned short len)
{
  TX_TEMPLATE(huart7,UART7_TxCBuf,UART7_TxBuf);
}

unsigned int UART7_Recv(unsigned char *data, unsigned short len)
{    
  unsigned int result = 0;

  if(data != NULL)
      result = CircBuf_Pop(&UART7_RxCBuf, data, len);

  return result;
}

unsigned char UART7_At( unsigned short offset)
{
    return CircBuf_At(&UART7_RxCBuf, offset);
}

void UART7_Drop( unsigned short LenToDrop)
{
    CircBuf_Drop(&UART7_RxCBuf, LenToDrop);
}

unsigned int UART7_GetDataCount( void )
{
    return CircBuf_GetUsedSize(&UART7_RxCBuf);
}

void UART7_Free(void)
{
    CircBuf_Free(&UART7_RxCBuf);
}



/***************USART10函数***************/
/**
 * @brief USART10初始化
 */
void usart10_init(void)
{

  HAL_UARTEx_ReceiveToIdle_DMA(&huart10,USART10_RxBuf0,(uint16_t)USART10_BUF_SIZE);
  /*初始化环形缓冲*/
  CircBuf_Init(&USART10_TxCBuf, USART10_TxCBuf_Arr, MAX_RING_BUF_SIZE);  
  CircBuf_Init(&USART10_RxCBuf, USART10_RxCBuf_Arr, MAX_RING_BUF_SIZE);
  HAL_DMA_RegisterCallback((&huart10)->hdmarx, HAL_DMA_XFER_CPLT_CB_ID, USART10_TX_CPLT_HOOK);
  __HAL_DMA_ENABLE((&huart10)->hdmarx);
  __HAL_DMA_ENABLE((&huart10)->hdmatx);
}

/**
 * @brief USART10空闲中断hook函数
 */
void USART10_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size)
{
  SINGLE_BUF_RX_TEMPLATE(huart,USART10_RxCBuf, USART10_RxBuf0, USART10_BUF_SIZE,Size);
}

/**
 * @brief USART10错误中断hook函数
 */
void USART10_ERR_HOOK(void)
{
  memset(USART10_RxBuf0,0,USART10_BUF_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart10,USART10_RxBuf0,USART10_BUF_SIZE);
}
void USART10_TX_CPLT_HOOK(DMA_HandleTypeDef* hdma)
{
  TX_CPLT_HOOK(huart10,USART10_TxCBuf,USART10_TxBuf);
}

unsigned int USART10_Send(uint8_t *data, unsigned short len)
{
  TX_TEMPLATE(huart10,USART10_TxCBuf,USART10_TxBuf);
}

unsigned int USART10_Recv(unsigned char *data, unsigned short len)
{    
  unsigned int result = 0;

  if(data != NULL)
      result = CircBuf_Pop(&USART10_RxCBuf, data, len);

  return result;
}

unsigned char USART10_At( unsigned short offset)
{
    return CircBuf_At(&USART10_RxCBuf, offset);
}

void USART10_Drop( unsigned short LenToDrop)
{
    CircBuf_Drop(&USART10_RxCBuf, LenToDrop);
}

unsigned int USART10_GetDataCount( void )
{
    return CircBuf_GetUsedSize(&USART10_RxCBuf);
}

void USART10_Free(void)
{
    CircBuf_Free(&USART10_RxCBuf);
}



/***************USART2函数***************/
/**
 * @brief USART2初始化
 */
void usart2_init(void)
{

  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,USART2_RxBuf0,(uint16_t)USART2_BUF_SIZE);
  /*初始化环形缓冲*/
  CircBuf_Init(&USART2_TxCBuf, USART2_TxCBuf_Arr, MAX_RING_BUF_SIZE);  
  CircBuf_Init(&USART2_RxCBuf, USART2_RxCBuf_Arr, MAX_RING_BUF_SIZE);
  HAL_DMA_RegisterCallback((&huart2)->hdmarx, HAL_DMA_XFER_CPLT_CB_ID, USART2_TX_CPLT_HOOK);
  __HAL_DMA_ENABLE((&huart2)->hdmarx);
  __HAL_DMA_ENABLE((&huart2)->hdmatx);
}

/**
 * @brief USART2空闲中断hook函数
 */
void USART2_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size)
{
  SINGLE_BUF_RX_TEMPLATE(huart,USART2_RxCBuf, USART2_RxBuf0, USART2_BUF_SIZE,Size);
}

/**
 * @brief USART2错误中断hook函数
 */
void USART2_ERR_HOOK(void)
{
  /*FE错误清除了缓冲，未找到原因暂且注释*/
  //memset(USART2_RxBuf0,0,USART2_BUF_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2,USART2_RxBuf0,USART2_BUF_SIZE);
}
void USART2_TX_CPLT_HOOK(DMA_HandleTypeDef* hdma)
{
  TX_CPLT_HOOK(huart2,USART2_TxCBuf,USART2_TxBuf);
}

unsigned int USART2_Send(uint8_t *data, unsigned short len)
{
  TX_TEMPLATE(huart2,USART2_TxCBuf,USART2_TxBuf);
}

unsigned int USART2_Recv(unsigned char *data, unsigned short len)
{    
  unsigned int result = 0;

  if(data != NULL)
      result = CircBuf_Pop(&USART2_RxCBuf, data, len);

  return result;
}

unsigned char USART2_At( unsigned short offset)
{
    return CircBuf_At(&USART2_RxCBuf, offset);
}

void USART2_Drop( unsigned short LenToDrop)
{
    CircBuf_Drop(&USART2_RxCBuf, LenToDrop);
}

unsigned int USART2_GetDataCount( void )
{
    return CircBuf_GetUsedSize(&USART2_RxCBuf);
}

void USART2_Free(void)
{
    CircBuf_Free(&USART2_RxCBuf);
}



/***************USART3函数***************/

/**
 * @brief USART3初始化
 */
void usart3_init(void)
{

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,USART3_RxBuf0,(uint16_t)USART3_BUF_SIZE);
  /*初始化环形缓冲*/
  CircBuf_Init(&USART3_TxCBuf, USART3_TxCBuf_Arr, MAX_RING_BUF_SIZE);  
  CircBuf_Init(&USART3_RxCBuf, USART3_RxCBuf_Arr, MAX_RING_BUF_SIZE);
  HAL_DMA_RegisterCallback((&huart3)->hdmarx, HAL_DMA_XFER_CPLT_CB_ID, USART3_TX_CPLT_HOOK);
  __HAL_DMA_ENABLE((&huart3)->hdmarx);
  __HAL_DMA_ENABLE((&huart3)->hdmatx);
}

/**
 * @brief USART3空闲中断hook函数
 */
void USART3_IDLERX_HOOK(UART_HandleTypeDef *huart,uint16_t Size)
{
  SINGLE_BUF_RX_TEMPLATE(huart,USART3_RxCBuf, USART3_RxBuf0, USART3_BUF_SIZE,Size);
}

/**
 * @brief USART3错误中断hook函数
 */
void USART3_ERR_HOOK(void)
{
  memset(USART3_RxBuf0,0,USART3_BUF_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,USART3_RxBuf0,USART3_BUF_SIZE);
}
void USART3_TX_CPLT_HOOK(DMA_HandleTypeDef* hdma)
{
  TX_CPLT_HOOK(huart3,USART3_TxCBuf,USART3_TxBuf);
}

unsigned int USART3_Send(uint8_t *data, unsigned short len)
{
  TX_TEMPLATE(huart3,USART3_TxCBuf,USART3_TxBuf);
}

unsigned int USART3_Recv(unsigned char *data, unsigned short len)
{    
  unsigned int result = 0;

  if(data != NULL)
      result = CircBuf_Pop(&USART3_RxCBuf, data, len);

  return result;
}

unsigned char USART3_At( unsigned short offset)
{
    return CircBuf_At(&USART3_RxCBuf, offset);
}

void USART3_Drop( unsigned short LenToDrop)
{
    CircBuf_Drop(&USART3_RxCBuf, LenToDrop);
}

unsigned int USART3_GetDataCount( void )
{
    return CircBuf_GetUsedSize(&USART3_RxCBuf);
}

void USART3_Free(void)
{
    CircBuf_Free(&USART3_RxCBuf);
}
