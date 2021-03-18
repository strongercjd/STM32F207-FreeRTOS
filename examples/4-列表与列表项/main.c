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


//定义一个测试用的列表和3个列表项
List_t TestList;	//测试用列表
ListItem_t ListItem1;	//测试用列表项1
ListItem_t ListItem2;	//测试用列表项2
ListItem_t ListItem3;	//测试用列表项3


/*Task1*/
void TASK1_entry(void *pvParameters)
{
  //第一步：初始化列表和列表项
  vListInitialise(&TestList);
  vListInitialiseItem(&ListItem1);
  vListInitialiseItem(&ListItem2);
  vListInitialiseItem(&ListItem3);
  
  ListItem1.xItemValue=40;			//ListItem1列表项值为40
  ListItem2.xItemValue=60;			//ListItem2列表项值为60
  ListItem3.xItemValue=50;			//ListItem3列表项值为50
  
  //第二步：打印列表和其他列表项的地址
  printf("/*******************列表和列表项地址*******************/\r\n");
  printf("项目                              地址				    \r\n");
  printf("TestList                          %#x					\r\n",(int)&TestList);
  printf("TestList->pxIndex                 %#x					\r\n",(int)TestList.pxIndex);
  printf("TestList->xListEnd                %#x					\r\n",(int)(&TestList.xListEnd));
  printf("ListItem1                         %#x					\r\n",(int)&ListItem1);
  printf("ListItem2                         %#x					\r\n",(int)&ListItem2);
  printf("ListItem3                         %#x					\r\n",(int)&ListItem3);
  printf("/************************结束**************************/\r\n");
  
  SysCtlDelay(2*(SystemCoreClock/3));
  
  //第三步：向列表TestList添加列表项ListItem1，并通过串口打印所有
  //列表项中成员变量pxNext和pxPrevious的值，通过这两个值观察列表
  //项在列表中的连接情况。
  vListInsert(&TestList,&ListItem1);		//插入列表项ListItem1
  printf("/******************添加列表项ListItem1*****************/\r\n");
  printf("项目                              地址				    \r\n");
  printf("TestList->xListEnd->pxNext        %#x					\r\n",(int)(TestList.xListEnd.pxNext));
  printf("ListItem1->pxNext                 %#x					\r\n",(int)(ListItem1.pxNext));
  printf("/*******************前后向连接分割线********************/\r\n");
  printf("TestList->xListEnd->pxPrevious    %#x					\r\n",(int)(TestList.xListEnd.pxPrevious));
  printf("ListItem1->pxPrevious             %#x					\r\n",(int)(ListItem1.pxPrevious));
  printf("/************************结束**************************/\r\n");
  
  SysCtlDelay(2*(SystemCoreClock/3));
  
  //第四步：向列表TestList添加列表项ListItem2，并通过串口打印所有
  //列表项中成员变量pxNext和pxPrevious的值，通过这两个值观察列表
  //项在列表中的连接情况。
  vListInsert(&TestList,&ListItem2);	//插入列表项ListItem2
  printf("/******************添加列表项ListItem2*****************/\r\n");
  printf("项目                              地址				    \r\n");
  printf("TestList->xListEnd->pxNext        %#x					\r\n",(int)(TestList.xListEnd.pxNext));
  printf("ListItem1->pxNext                 %#x					\r\n",(int)(ListItem1.pxNext));
  printf("ListItem2->pxNext                 %#x					\r\n",(int)(ListItem2.pxNext));
  printf("/*******************前后向连接分割线********************/\r\n");
  printf("TestList->xListEnd->pxPrevious    %#x					\r\n",(int)(TestList.xListEnd.pxPrevious));
  printf("ListItem1->pxPrevious             %#x					\r\n",(int)(ListItem1.pxPrevious));
  printf("ListItem2->pxPrevious             %#x					\r\n",(int)(ListItem2.pxPrevious));
  printf("/************************结束**************************/\r\n");
  
  SysCtlDelay(2*(SystemCoreClock/3));
  
  //第五步：向列表TestList添加列表项ListItem3，并通过串口打印所有
  //列表项中成员变量pxNext和pxPrevious的值，通过这两个值观察列表
  //项在列表中的连接情况。
  vListInsert(&TestList,&ListItem3);	//插入列表项ListItem3
  printf("/******************添加列表项ListItem3*****************/\r\n");
  printf("项目                              地址				    \r\n");
  printf("TestList->xListEnd->pxNext        %#x					\r\n",(int)(TestList.xListEnd.pxNext));
  printf("ListItem1->pxNext                 %#x					\r\n",(int)(ListItem1.pxNext));
  printf("ListItem3->pxNext                 %#x					\r\n",(int)(ListItem3.pxNext));
  printf("ListItem2->pxNext                 %#x					\r\n",(int)(ListItem2.pxNext));
  printf("/*******************前后向连接分割线********************/\r\n");
  printf("TestList->xListEnd->pxPrevious    %#x					\r\n",(int)(TestList.xListEnd.pxPrevious));
  printf("ListItem1->pxPrevious             %#x					\r\n",(int)(ListItem1.pxPrevious));
  printf("ListItem3->pxPrevious             %#x					\r\n",(int)(ListItem3.pxPrevious));
  printf("ListItem2->pxPrevious             %#x					\r\n",(int)(ListItem2.pxPrevious));
  printf("/************************结束**************************/\r\n");
  
  SysCtlDelay(2*(SystemCoreClock/3));
  
  //第六步：删除ListItem2，并通过串口打印所有列表项中成员变量pxNext和
  //pxPrevious的值，通过这两个值观察列表项在列表中的连接情况。
  uxListRemove(&ListItem2);						//删除ListItem2
  printf("/******************删除列表项ListItem2*****************/\r\n");
  printf("项目                              地址				    \r\n");
  printf("TestList->xListEnd->pxNext        %#x					\r\n",(int)(TestList.xListEnd.pxNext));
  printf("ListItem1->pxNext                 %#x					\r\n",(int)(ListItem1.pxNext));
  printf("ListItem3->pxNext                 %#x					\r\n",(int)(ListItem3.pxNext));
  printf("/*******************前后向连接分割线********************/\r\n");
  printf("TestList->xListEnd->pxPrevious    %#x					\r\n",(int)(TestList.xListEnd.pxPrevious));
  printf("ListItem1->pxPrevious             %#x					\r\n",(int)(ListItem1.pxPrevious));
  printf("ListItem3->pxPrevious             %#x					\r\n",(int)(ListItem3.pxPrevious));
  printf("/************************结束**************************/\r\n");
  
  SysCtlDelay(2*(SystemCoreClock/3));
  
  //第七步：删除ListItem2，并通过串口打印所有列表项中成员变量pxNext和
  //pxPrevious的值，通过这两个值观察列表项在列表中的连接情况。
  TestList.pxIndex=TestList.pxIndex->pxNext;			//pxIndex向后移一项，这样pxIndex就会指向ListItem1。
  vListInsertEnd(&TestList,&ListItem2);				//列表末尾添加列表项ListItem2
  printf("/***************在末尾添加列表项ListItem2***************/\r\n");
  printf("项目                              地址				    \r\n");
  printf("TestList->pxIndex                 %#x					\r\n",(int)TestList.pxIndex);
  printf("TestList->xListEnd->pxNext        %#x					\r\n",(int)(TestList.xListEnd.pxNext));
  printf("ListItem2->pxNext                 %#x					\r\n",(int)(ListItem2.pxNext));
  printf("ListItem1->pxNext                 %#x					\r\n",(int)(ListItem1.pxNext));
  printf("ListItem3->pxNext                 %#x					\r\n",(int)(ListItem3.pxNext));
  printf("/*******************前后向连接分割线********************/\r\n");
  printf("TestList->xListEnd->pxPrevious    %#x					\r\n",(int)(TestList.xListEnd.pxPrevious));
  printf("ListItem2->pxPrevious             %#x					\r\n",(int)(ListItem2.pxPrevious));
  printf("ListItem1->pxPrevious             %#x					\r\n",(int)(ListItem1.pxPrevious));
  printf("ListItem3->pxPrevious             %#x					\r\n",(int)(ListItem3.pxPrevious));
  printf("/************************结束**************************/\r\n\r\n\r\n");
  while(1)
  {
    vTaskDelay(1000);                           //延时1s，也就是1000个时钟节拍	
  }
}   


//开始任务任务函数
void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();           //进入临界区
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
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
  
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
  
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);
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
