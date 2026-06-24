#include "usart_measure_task.h"
#include "bsp_usart.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "M8010_motor.h" 
#include "referee.h"

/*反馈数据大小*/
#define FEEDBACK_DATA_SIZE 16

/**/
static unsigned char Transmission_usart2[80] = {0};  //unsigned char类型长度
static unsigned char Transmission_usart3[80] = {0};  //unsigned char类型长度
static unsigned int usart2_length;
static unsigned int usart3_length;

/*referee*/
static unsigned int frame_size;

void usart2_measure_task(void const *pvParameters) 
{
  while(1)
  {
    next_usart2:
    usart2_length = USART2_GetDataCount();  // 得出数据的长度，包括帧头、帧尾、ID和有用的数据

        if(usart2_length >= 10)
        {
            if(USART2_At(0) == 0xFD && USART2_At(1) == 0xEE)  // 判断数据的起始值是否为0xFD 0xEE
            {
                USART2_Recv(Transmission_usart2, FEEDBACK_DATA_SIZE);  // 把数据出栈并存储在Transmission_BufferOfusart2，数据处理在中断里
                //usart2_motor_rx = *SERVO_Recv((MOTOR_recv *)Transmission_usart2);
                SERVO_Recv(&joint1_motor,Transmission_usart2);
                USART2_Drop(4096);
            }
            else
            {
                USART2_Drop(1);
                vTaskDelay(1);
                
				usart2_length = USART2_GetDataCount();
                if(usart2_length > 3000)
                {
                    USART2_Drop(4096);
                }
                
                goto next_usart2;
            }
        }
        else
        {
            //USART2_Drop(1);/*在没有数据的时候抛弃数据导致环形缓冲区异常*/
            for(;;)
            {
				    usart2_length =  USART2_GetDataCount();
                if( usart2_length > 0)
                {
                    if(USART2_At(0) == 0xFD && USART2_At(1) == 0xEE)  // Frame head
                    {
                        break;
                    }
                    else
                    {
                        USART2_Drop(1);
                        vTaskDelay(1);
                    }
                     usart2_length =  USART2_GetDataCount();
                    if(usart2_length > 3000)
                    {
                        USART2_Drop(4096);
                        break;
                    }
                }
                else
                {
                    break;
                }
            }
        }
        vTaskDelay(1);
    }
}


void usart3_measure_task(void const *pvParameters) 
{
//    while(1)
//    {
//        next_usart3:
//        usart3_length = USART3_GetDataCount();
//        
//        if(usart3_length >= 10)
//        {
//            if(USART3_At(0) == 0xFD && USART3_At(1) == 0xEE)
//            {
//                USART3_Recv(Transmission_usart3, FEEDBACK_DATA_SIZE);
//                //usart3_motor_rx = *SERVO_Recv((MOTOR_recv *)Transmission_usart3);
//                USART3_Drop(4096);
//            }
//            else
//            {
//                USART3_Drop(1);
//                vTaskDelay(1);
//                
//                usart3_length = USART3_GetDataCount();
//                if(usart3_length > 3000)
//                {
//                    USART3_Drop(4096);
//                }
//                
//                goto next_usart3;
//            }
//        }
//        else
//        {
//            USART3_Drop(1);
//            for(;;)
//            {
//                usart3_length =  USART3_GetDataCount();
//                if( usart3_length > 0)
//                {
//                    if(USART3_At(0) == 0xFD && USART3_At(1) == 0xEE)
//                    {
//                        break;
//                    }
//                    else
//                    {
//                        USART3_Drop(1);
//                        vTaskDelay(1);
//                    }
//                    
//                    usart3_length =  USART3_GetDataCount();
//                    if(usart3_length > 3000)
//                    {
//                        USART3_Drop(4096);
//                        break;
//                    }
//                }
//                else
//                {
//                    break;
//                }
//            }
//        }
    vTaskDelay(1);
//    }
}
     

