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
#define TASK1_SIZE 		256  
TaskHandle_t Task1_Handler;
void TASK1_entry(void *pvParameters);

/*Task2*/
#define TASK2_PRIO		3
#define TASK2_SIZE 		128  
TaskHandle_t Task2_Handler;
void TASK2_entry(void *pvParameters);


char InfoBuffer[1000];				//保存信息的数组
/*Task1*/
void TASK1_entry(void *pvParameters)
{
  uint32_t TotalRunTime;
  UBaseType_t ArraySize,x;
  TaskStatus_t *StatusArray;
  
  TaskHandle_t TaskHandle;	
  TaskStatus_t TaskStatus;
  
  eTaskState TaskState;
  char TaskInfo[10];
  
  //第一步：函数uxTaskGetSystemState()的使用
  printf("/********第一步：函数uxTaskGetSystemState()的使用**********/\r\n");
  ArraySize=uxTaskGetNumberOfTasks();		//获取系统任务数量
  StatusArray=pvPortMalloc(ArraySize*sizeof(TaskStatus_t));//申请内存
  if(StatusArray!=NULL)					//内存申请成功
  {
    ArraySize=uxTaskGetSystemState((TaskStatus_t* 	)StatusArray, 	//任务信息存储数组
                                   (UBaseType_t		)ArraySize, 	//任务信息存储数组大小
                                   (uint32_t*		)&TotalRunTime);//保存系统总的运行时间
    printf("TaskName\t\tPriority\t\tTaskNumber\t\t\r\n");
    for(x=0;x<ArraySize;x++)
    {
      //通过串口打印出获取到的系统任务的有关信息，比如任务名称、
      //任务优先级和任务编号。
      printf("%s\t\t%d\t\t\t%d\t\t\t\r\n",				
             StatusArray[x].pcTaskName,
             (int)StatusArray[x].uxCurrentPriority,
             (int)StatusArray[x].xTaskNumber);
      
    }
  }
  vPortFree(StatusArray);	//释放内存
  printf("/**************************结束***************************/\r\n");

  
  //第二步：函数vTaskGetInfo()的使用
  printf("/************第二步：函数vTaskGetInfo()的使用**************/\r\n");
  TaskHandle=xTaskGetHandle("TASK2");			//根据任务名获取任务句柄。
  //获取TASK2的任务信息
  vTaskGetInfo((TaskHandle_t	)TaskHandle, 		//任务句柄
               (TaskStatus_t*	)&TaskStatus, 		//任务信息结构体
               (BaseType_t	)pdTRUE,			//允许统计任务堆栈历史最小剩余大小
               (eTaskState	)eInvalid);			//函数自己获取任务运行壮态
  //通过串口打印出指定任务的有关信息。
  printf("任务名:                %s\r\n",TaskStatus.pcTaskName);
  printf("任务编号:              %d\r\n",(int)TaskStatus.xTaskNumber);
  printf("任务壮态:              %d\r\n",TaskStatus.eCurrentState);
  printf("任务当前优先级:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
  printf("任务基优先级:          %d\r\n",(int)TaskStatus.uxBasePriority);
  printf("任务堆栈基地址:        %#x\r\n",(int)TaskStatus.pxStackBase);
  printf("任务堆栈历史剩余最小值:%d\r\n",TaskStatus.usStackHighWaterMark);
  printf("/**************************结束***************************/\r\n");

  
  //第三步：函数eTaskGetState()的使用	
  printf("/***********第三步：函数eTaskGetState()的使用*************/\r\n");
  TaskHandle=xTaskGetHandle("TASK1");		//根据任务名获取任务句柄。
  TaskState=eTaskGetState(TaskHandle);			//获取query_task任务的任务壮态
  memset(TaskInfo,0,10);						
  switch((int)TaskState)
  {
  case 0:
    sprintf(TaskInfo,"Running");
    break;
  case 1:
    sprintf(TaskInfo,"Ready");
    break;
  case 2:
    sprintf(TaskInfo,"Suspend");
    break;
  case 3:
    sprintf(TaskInfo,"Delete");
    break;
  case 4:
    sprintf(TaskInfo,"Invalid");
    break;
  }
  printf("任务壮态值:%d,对应的壮态为:%s\r\n",TaskState,TaskInfo);
  printf("/**************************结束**************************/\r\n");

  
  //第四步：函数vTaskList()的使用	
  printf("/*************第三步：函数vTaskList()的使用*************/\r\n");
  vTaskList(InfoBuffer);							//获取所有任务的信息
  printf("%s\r\n",InfoBuffer);					//通过串口打印所有任务的信息
  printf("/**************************结束**************************/\r\n");
  while(1)
  {
    vTaskDelay(1000);                           //延时1s，也就是1000个时钟节拍	
  }
}   

/*Task2*/
void TASK2_entry(void *pvParameters)
{
  while(1)
  {
    vTaskDelay(800);
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
  //创建TASK2任务
  xTaskCreate((TaskFunction_t )TASK2_entry, //任务函数
              (const char*    )"TASK2",    //任务名称
              (uint16_t       )TASK2_SIZE, //任务堆栈大小
              (void*          )NULL,          //传递给任务函数的参数
              (UBaseType_t    )TASK2_PRIO, //任务优先级
              (TaskHandle_t*  )&Task2_Handler);//任务句柄   
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
