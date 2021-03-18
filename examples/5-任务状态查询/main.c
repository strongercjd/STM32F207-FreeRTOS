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


/*120Mhzʱ��ʱ����ulCountΪ1ʱ��������ʱ3��ʱ�ӣ���ʱ=3*1/120us=1/40us*/
/*
SystemCoreClock=120000000

us����ʱ,��ʱn΢��
SysCtlDelay(n*(SystemCoreClock/3000000));

ms����ʱ,��ʱn����
SysCtlDelay(n*(SystemCoreClock/3000));

m����ʱ,��ʱn��
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
/*��*/
#endif /* __CC_ARM */

/*start_task*/
#define START_TASK_PRIO		1  //�������ȼ�
#define START_STK_SIZE 		128 //ջ��С
TaskHandle_t StartTask_Handler;    //������
void start_task(void *pvParameters);//������

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


char InfoBuffer[1000];				//������Ϣ������
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
  
  //��һ��������uxTaskGetSystemState()��ʹ��
  printf("/********��һ��������uxTaskGetSystemState()��ʹ��**********/\r\n");
  ArraySize=uxTaskGetNumberOfTasks();		//��ȡϵͳ��������
  StatusArray=pvPortMalloc(ArraySize*sizeof(TaskStatus_t));//�����ڴ�
  if(StatusArray!=NULL)					//�ڴ�����ɹ�
  {
    ArraySize=uxTaskGetSystemState((TaskStatus_t* 	)StatusArray, 	//������Ϣ�洢����
                                   (UBaseType_t		)ArraySize, 	//������Ϣ�洢�����С
                                   (uint32_t*		)&TotalRunTime);//����ϵͳ�ܵ�����ʱ��
    printf("TaskName\t\tPriority\t\tTaskNumber\t\t\r\n");
    for(x=0;x<ArraySize;x++)
    {
      //ͨ�����ڴ�ӡ����ȡ����ϵͳ������й���Ϣ�������������ơ�
      //�������ȼ��������š�
      printf("%s\t\t%d\t\t\t%d\t\t\t\r\n",				
             StatusArray[x].pcTaskName,
             (int)StatusArray[x].uxCurrentPriority,
             (int)StatusArray[x].xTaskNumber);
      
    }
  }
  vPortFree(StatusArray);	//�ͷ��ڴ�
  printf("/**************************����***************************/\r\n");

  
  //�ڶ���������vTaskGetInfo()��ʹ��
  printf("/************�ڶ���������vTaskGetInfo()��ʹ��**************/\r\n");
  TaskHandle=xTaskGetHandle("TASK2");			//������������ȡ��������
  //��ȡTASK2��������Ϣ
  vTaskGetInfo((TaskHandle_t	)TaskHandle, 		//������
               (TaskStatus_t*	)&TaskStatus, 		//������Ϣ�ṹ��
               (BaseType_t	)pdTRUE,			//����ͳ�������ջ��ʷ��Сʣ���С
               (eTaskState	)eInvalid);			//�����Լ���ȡ��������׳̬
  //ͨ�����ڴ�ӡ��ָ��������й���Ϣ��
  printf("������:                %s\r\n",TaskStatus.pcTaskName);
  printf("������:              %d\r\n",(int)TaskStatus.xTaskNumber);
  printf("����׳̬:              %d\r\n",TaskStatus.eCurrentState);
  printf("����ǰ���ȼ�:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
  printf("��������ȼ�:          %d\r\n",(int)TaskStatus.uxBasePriority);
  printf("�����ջ����ַ:        %#x\r\n",(int)TaskStatus.pxStackBase);
  printf("�����ջ��ʷʣ����Сֵ:%d\r\n",TaskStatus.usStackHighWaterMark);
  printf("/**************************����***************************/\r\n");

  
  //������������eTaskGetState()��ʹ��	
  printf("/***********������������eTaskGetState()��ʹ��*************/\r\n");
  TaskHandle=xTaskGetHandle("TASK1");		//������������ȡ��������
  TaskState=eTaskGetState(TaskHandle);			//��ȡquery_task���������׳̬
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
  printf("����׳ֵ̬:%d,��Ӧ��׳̬Ϊ:%s\r\n",TaskState,TaskInfo);
  printf("/**************************����**************************/\r\n");

  
  //���Ĳ�������vTaskList()��ʹ��	
  printf("/*************������������vTaskList()��ʹ��*************/\r\n");
  vTaskList(InfoBuffer);							//��ȡ�����������Ϣ
  printf("%s\r\n",InfoBuffer);					//ͨ�����ڴ�ӡ�����������Ϣ
  printf("/**************************����**************************/\r\n");
  while(1)
  {
    vTaskDelay(1000);                           //��ʱ1s��Ҳ����1000��ʱ�ӽ���	
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

//��ʼ����������
void start_task(void *pvParameters)
{
  taskENTER_CRITICAL();           //�����ٽ���
  //����TASK1����
  xTaskCreate((TaskFunction_t )TASK1_entry,//������
              (const char*    )"TASK1",   //��������
              (uint16_t       )TASK1_SIZE,//�����ջ��С
              (void*          )NULL,	     //���ݸ��������Ĳ���
              (UBaseType_t    )TASK1_PRIO,//�������ȼ�
              (TaskHandle_t*  )&Task1_Handler);//������ 
  //����TASK2����
  xTaskCreate((TaskFunction_t )TASK2_entry, //������
              (const char*    )"TASK2",    //��������
              (uint16_t       )TASK2_SIZE, //�����ջ��С
              (void*          )NULL,          //���ݸ��������Ĳ���
              (UBaseType_t    )TASK2_PRIO, //�������ȼ�
              (TaskHandle_t*  )&Task2_Handler);//������   
  vTaskDelete(StartTask_Handler); //ɾ����ʼ����
  taskEXIT_CRITICAL();            //�˳��ٽ���
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
  
  /* ʹ��GPIOE�˿�ʱ�� */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
  
  /* ����LED�ܽ� */
  GPIO_Init_s.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init_s.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init_s.GPIO_OType = GPIO_OType_PP;
  GPIO_Init_s.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init_s.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_Init_s);
  
  GPIO_SetBits(GPIOE,GPIO_Pin_4);//Ϩ��LED��
  
  UART_Init();
  
  
  printf("\r\n======================================================================");
  printf("\r\n=               (C) COPYRIGHT 2021                                   =");
  printf("\r\n=                                                                    =");
  printf("\r\n=                  ST207 FreeRTOS                                    =");
  printf("\r\n=                                                                    =");
  printf("\r\n=                                           By Firefly               =");
  printf("\r\n======================================================================");
  printf("\r\n\r\n");
  
  //������ʼ����
  xTaskCreate((TaskFunction_t )start_task,            //������
              (const char*    )"start_task",          //��������
              (uint16_t       )START_STK_SIZE,        //�����ջ��С
              (void*          )NULL,                  //���ݸ��������Ĳ���
              (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
              (TaskHandle_t*  )&StartTask_Handler);   //������              
  vTaskStartScheduler();          //�����������
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
