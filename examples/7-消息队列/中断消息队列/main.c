/**
  ******************************************************************************
  * @file    Project/STM32F2xx_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    13-April-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

void UART_Init(void);

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/*120Mhz时钟时，当ulCount为1时，函数耗时3个时钟，延时=3*1/120us=1/40us*/
/*
SystemCoreClock=120000000

us级延时,延时n微秒
SysCtlDelay(n*(SystemCoreClock/3000000));

ms级延时,延时n毫秒
SysCtlDelay(n*(SystemCoreClock/3000));

m级延时,延时n秒
SysCtlDelay(n*(SystemCoreClock/3));
*/

#if defined   (__CC_ARM) /*!< ARM Compiler */
__asm void
SysCtlDelay(unsigned long ulCount)
{
    subs    r0, #1;
    bne     SysCtlDelay;
    bx      lr;
}
#elif defined ( __ICCARM__ ) /*!< IAR Compiler */
void
SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
       "    bne.n   SysCtlDelay\n"
       "    bx      lr");
}

#elif defined (__GNUC__) /*!< GNU Compiler */
void __attribute__((naked))
SysCtlDelay(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
       "    bne     SysCtlDelay\n"
       "    bx      lr");
}

#elif defined  (__TASKING__) /*!< TASKING Compiler */                           
/*无*/
#endif /* __CC_ARM */

/*start_task*/
#define START_TASK_PRIO		1  //任务优先级
#define START_STK_SIZE 		128 //栈大小
TaskHandle_t StartTask_Handler;    //任务句柄
void start_task(void *pvParameters);//任务函数

/*Task1*/
#define TASK1_PRIO		2
#define TASK1_SIZE 		50  
TaskHandle_t Task1_Handler;
void TASK1_entry(void *pvParameters);



//消息队列
#define IRQ_Q_NUM       4   	//中断消息队列的数量 
QueueHandle_t IRQ_Queue;	//中断消息队列句柄


//接收状态
//bit15，	接收完成标志,接收到0X0A
//bit14~0，	接收到的有效字节数目
uint16_t USART_RX_STA=0;       //接收状态标记
#define USART_REC_LEN    100
uint8_t USART_RX_BUF[USART_REC_LEN];

uint8_t TASK1_RX_BUF[USART_REC_LEN];

/*Task1*/
void TASK1_entry(void *pvParameters)
{
  BaseType_t xTaskWokenByReceive=pdFALSE;
  BaseType_t err;
  while (1)
  {
    GPIO_SetBits(GPIOE,GPIO_Pin_4);  //熄灭LED灯
    vTaskDelay(500);                      //延时500ms
    GPIO_ResetBits(GPIOE,GPIO_Pin_4);//点亮LED灯
    vTaskDelay(500);
    
    if(IRQ_Queue!=NULL)
    {
      memset(TASK1_RX_BUF,0,USART_REC_LEN);	//清除缓冲区
      err=xQueueReceiveFromISR(IRQ_Queue,TASK1_RX_BUF,&xTaskWokenByReceive);//请求消息Message_Queue
      if(err==pdTRUE)			//接收到消息
      {
        printf("TASK1: recv msg from IRQ_Queue:%s\n", TASK1_RX_BUF);
      }
    }
    portYIELD_FROM_ISR(xTaskWokenByReceive);//如果需要的话进行一次任务切换
  }
}

//开始任务任务函数
void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();           //进入临界区
  IRQ_Queue=xQueueCreate(IRQ_Q_NUM,USART_REC_LEN);        //创建消息Key_Queue
  //创建TASK1任务
  xTaskCreate((TaskFunction_t )TASK1_entry,//任务函数
              (const char*    )"TASK1",   //任务名称
              (uint16_t       )TASK1_SIZE,//任务堆栈大小
              (void*          )NULL,	     //传递给任务函数的参数
              (UBaseType_t    )TASK1_PRIO,//任务优先级
              (TaskHandle_t*  )&Task1_Handler);//任务句柄       
  vTaskDelete(StartTask_Handler); //删除开始任务
  taskEXIT_CRITICAL();            //退出临界区
}


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  GPIO_InitTypeDef  GPIO_Init_s;
  
  /* 使能GPIOE端口时钟 */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  /* 配置LED管脚 */
  GPIO_Init_s.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init_s.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init_s.GPIO_OType = GPIO_OType_PP;
  GPIO_Init_s.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init_s.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_Init_s);
  
  GPIO_SetBits(GPIOE,GPIO_Pin_4);//熄灭LED灯
  
  UART_Init();
  
  printf("\r\n======================================================================");
  printf("\r\n=               (C) COPYRIGHT 2021                                   =");
  printf("\r\n=                                                                    =");
  printf("\r\n=                  ST207 FreeRTOS                                    =");
  printf("\r\n=                                                                    =");
  printf("\r\n=                                           By Firefly               =");
  printf("\r\n======================================================================");
  printf("\r\n\r\n");
  
  //创建开始任务
  xTaskCreate((TaskFunction_t )start_task,            //任务函数
              (const char*    )"start_task",          //任务名称
              (uint16_t       )START_STK_SIZE,        //任务堆栈大小
              (void*          )NULL,                  //传递给任务函数的参数
              (UBaseType_t    )START_TASK_PRIO,       //任务优先级
              (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
  vTaskStartScheduler();          //开启任务调度
}

void UART_Init(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable UART1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(GPIOA, 9, GPIO_AF_USART1);
  
  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(GPIOA, 10, GPIO_AF_USART1);
  
  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  /* USART configuration */
  USART_Init(USART1, &USART_InitStructure);
  
  /*USART Recevie interrupt*/
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);
}
void USART1_IRQHandler(void)
{
  uint8_t Res;
  BaseType_t xHigherPriorityTaskWoken;
  BaseType_t result;
  
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
  {
    Res =USART_ReceiveData(USART1);	//读取接收到的数据
    if((USART_RX_STA&0x8000)==0)//接收未完成
    {
      if(Res==0x0A)
        USART_RX_STA|= 0x8000;	//接收完成了 
      else
      {
        USART_RX_BUF[USART_RX_STA&0X7FFF]=Res ;
        USART_RX_STA++;
        if(USART_RX_STA>(USART_REC_LEN-1))
          USART_RX_STA=0;//接收数据错误,重新开始接收	  
      }		 
      
    }   		 
  }
  
  //向队列发送接收到的数据
  if((USART_RX_STA&0x8000)&&(IRQ_Queue!=NULL))
  {
    result = xQueueSendFromISR(IRQ_Queue,USART_RX_BUF,&xHigherPriorityTaskWoken);//向队列中发送数据
    if (result == errQUEUE_FULL)
    {
      taskENTER_CRITICAL();
      printf("USART: FreeRTOS_mq_send ERR\n");
      taskEXIT_CRITICAL();
    }
    USART_RX_STA=0;	
    memset(USART_RX_BUF,0,USART_REC_LEN);//清除数据接收缓冲区USART_RX_BUF,用于下一次数据接收
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
  }
  
}
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}





/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
