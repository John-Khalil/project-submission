#include "main.h"

#include <functional>
#include <vector>
#include <memory>
#include <stdint.h>
#include <algorithm>

#include "consoleLogger.h"
// #pragma pack(1)

UART_HandleTypeDef huart2;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


/*definning the output port from register address*/

volatile uint16_t * _OUTPUT_PORT=(volatile uint16_t *)0x40020014;
volatile uint16_t * _INPUT_PORT=(volatile uint16_t *)0x40020410;

#define OUTPUT_PORT (*_OUTPUT_PORT)
#define INPUT_PORT (*_INPUT_PORT)
// #define INPUT_PORT *((volatile uint16_t *)0x40020410)

#define outputPort() OUTPUT_PORT

#define inputPort() INPUT_PORT
#define inputPIn(_PIN) ((inputPort()>>_PIN)&0x01)

const uint32_t outputPortMask=(((1<<4)-1)<<8);
const uint32_t inputPortMask=(((1<<4)-1)<<12);


/*software delay function that will waste cpu cycles*/

void delayTicks(uint32_t ticks){
    while(ticks--)__asm__("nop");
}

// uint32_t ticks;
// #define delayTicks(_ticks) ticks=_ticks;while(ticks--)__asm__("nop");


unsigned char UNDEFINED[10]="undefined";

/*string counter that will count till the pointer location would be a null*/

unsigned short stringCounter(unsigned char *counted){
	unsigned short counter=0;
	while(*counted){
		counter++;
		counted++;
	}
	return counter;
}

unsigned short CLR_LENGTH=0;									//this value will be reseted to zero after clearing the string/uint_8 pointer
unsigned char * CLR(unsigned char *deletedString,unsigned short _CLR_LENGTH=0){
	CLR_LENGTH=(CLR_LENGTH)?CLR_LENGTH:_CLR_LENGTH;
	unsigned char *returnedString=deletedString;
	while(*deletedString||(CLR_LENGTH-=(CLR_LENGTH!=0))){
		*deletedString=0;
		deletedString++;
	}
	return returnedString;
}

/*creating the function for the base64 encoding and decoding*/

unsigned char inverseBase64Table(unsigned char transBuf){
	return (transBuf-((65*((transBuf<0x5B)&&(transBuf>0x40)))|(71*((transBuf>0x60)&&(transBuf<0x7B)))|(-4*((transBuf>0x2F)&&(transBuf<0x3A)))|(-19*(transBuf==0x2B))|(-16*(transBuf==0x2F))))*(transBuf!=0x3D);
}

unsigned char *base64Decode(unsigned char *base64Text){
	unsigned char *startAddress=base64Text;
	unsigned char *lastAddress=startAddress+stringCounter(startAddress);
	unsigned char *base256Text=base64Text;
	unsigned short base64Counter=0;
	while(base64Text[base64Counter]){
		unsigned char base256Buffer[4]={inverseBase64Table(base64Text[base64Counter++]),inverseBase64Table(base64Text[base64Counter++]),inverseBase64Table(base64Text[base64Counter++]),inverseBase64Table(base64Text[base64Counter++])};
		*base256Text=(base256Buffer[0]<<2)|((base256Buffer[1]&0x30)>>4);
		base256Text++;
		*base256Text=(base256Buffer[1]<<4)|((base256Buffer[2]&0x3C)>>2);
		base256Text++;
		*base256Text=(base256Buffer[2]<<6)|base256Buffer[3];
		base256Text++;
	}
	while(base256Text<lastAddress){
		*base256Text=0;
		base256Text++;
	}
	return startAddress;
}

unsigned char base64Table(unsigned char transBuf){
		return transBuf+((65*(transBuf<26))|(71*((transBuf>25)&&(transBuf<52)))|(-4*((transBuf>51)&&(transBuf<62)))|(-19*(transBuf==62))|(-16*(transBuf==63)));
}

unsigned char *base64Encode(unsigned char *rawData, unsigned char *base64Text=NULL,unsigned short rawDataLength=0) {
	rawDataLength = rawDataLength?rawDataLength:stringCounter(rawData);

  static unsigned char *base64TextBuffer=NULL;
  if(base64TextBuffer!=NULL){
    free(base64TextBuffer);
    base64TextBuffer=NULL;
  }

  base64Text=base64Text?base64Text:(base64TextBuffer=(unsigned char*)calloc((rawDataLength*1.3334f)+8,sizeof(unsigned char)));

	unsigned char paddingCount = rawDataLength % 3;
	rawDataLength -= paddingCount;
	rawDataLength *= 1.3334f;
	unsigned short base64Counter = 0;
	unsigned char *rawData1 = rawData + 1;
	unsigned char *rawData2 = rawData + 2;
	while (base64Counter < rawDataLength) {
		base64Text[base64Counter++] = base64Table((*rawData) >> 2);
		base64Text[base64Counter++] = base64Table((((*rawData) & 0x03) << 4) | ((*rawData1) >> 4));
		base64Text[base64Counter++] = base64Table((((*rawData1) & 0x0F) << 2) | (((*rawData2) & 192) >> 6));
		base64Text[base64Counter++] = base64Table((*rawData2) & 0x3F);
		rawData += 3;
		rawData1 += 3; 
		rawData2 += 3;
	}
	if (paddingCount == 2) {
		base64Text[base64Counter++] = base64Table((*rawData) >> 2);
		base64Text[base64Counter++] = base64Table((((*rawData) & 0x03) << 4) | ((*rawData1) >> 4));
		base64Text[base64Counter++] = base64Table(((*rawData1) & 0x0F) << 2);
		base64Text[base64Counter++] = 0x3D;
	}
	else if (paddingCount == 1) {
		base64Text[base64Counter++] = base64Table((*rawData) >> 2);
		base64Text[base64Counter++] = base64Table(((*rawData) & 0x03) << 4);
		base64Text[base64Counter++] = 0x3D;
		base64Text[base64Counter++] = 0x3D;
	}
	return base64Text;
}

unsigned char equalStrings(unsigned char *stringPointer1,unsigned char *stringPointer2){
	unsigned short diffCounter;
	if((diffCounter=stringCounter(stringPointer1))!=stringCounter(stringPointer2))
		return 0;
	while((stringPointer1[--diffCounter]==stringPointer2[diffCounter])&&diffCounter);
	return (stringPointer1[diffCounter]==stringPointer2[diffCounter]);
}

unsigned char *_CS(unsigned char *bigString,unsigned char *smallString){
	unsigned char *smallStringLocation=bigString+stringCounter(bigString);		// lucky for us c/c++ support pointer arthematic
	while(*smallString){
		*smallStringLocation=*smallString;
		smallString++;
		smallStringLocation++;
	}
	return bigString;
}

/* creating wrapper functions for the default uart library that is is included with the STM cube ide*/

void send(unsigned char *txData){
	HAL_UART_Transmit(&huart2,txData,stringCounter(txData),100);
	return;
}

#define RX_BUFFER_SIZE 2048
unsigned char rxBuffer[RX_BUFFER_SIZE]={};

unsigned char *getData(void){
	CLR_LENGTH=RX_BUFFER_SIZE;
  do{   
	  HAL_UART_Receive (&huart2, CLR(rxBuffer), RX_BUFFER_SIZE-1, 100); // this needs to be changed later for the last parameter (timeout) 
  }while(!stringCounter(rxBuffer));
	return rxBuffer;
}




//^ ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//~ //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//^ /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* creating a shared buffer for the sample and base64 encoding*/

#define sampleBufferSize 2048

union sharedBuffer{
  uint16_t sampleBuffer[sampleBufferSize]={0};
  uint8_t base64Buffer[sampleBufferSize*2];
}signalBuffer;

unsigned short *sampleBuffer=signalBuffer.sampleBuffer;
unsigned char *sampleBufferBase64=signalBuffer.base64Buffer;
uint8_t sampleBase64Buffer[sampleBufferSize*2]={};                  //^ no need to dynamically allocate memory

static uint16_t maxTimeout=-1;
static float calibratingFactor=24.0;


/* creating a function that clear the shared buffer*/

void clearSample(void){
    unsigned long clearCounter=sampleBufferSize;
    while(clearCounter--)
        sampleBuffer[clearCounter]=0;
    return;
}


/* signal sampler*/

unsigned char *getSample(){
    clearSample();
    #define counterDelay() delayTicks(20)

    unsigned short sampleCounter=1;                                           //^ the first address is used for the idle state
    uint16_t pulseCounter;
    const uint16_t limiter=sampleBufferSize*0.75;                             //^ 25% extra is used for the base 64

    
    sampleBuffer[0]=(inputPort()&inputPortMask);

    unsigned short trigPins;
    unsigned short pin=0;
    while(!(trigPins=(sampleBuffer[0]^(inputPort()&inputPortMask))));
    while(trigPins>>pin++); pin-=2;

    // console.log("pin >> ",pin);

    pin=1<<pin;
    #define pinState() (inputPort()&(pin))
    unsigned short currentState=pinState();
    // OUTPUT_PORT|=(1<<8);

    do{
        pulseCounter=maxTimeout;
        
        while((currentState==pinState())&&(--pulseCounter)){            //^ this needs to be super fast
          counterDelay();
        }
        currentState=pinState();
        sampleBuffer[sampleCounter++]=maxTimeout-pulseCounter;
    }while(pulseCounter&&(sampleCounter<(limiter-1)));


    return base64Encode(sampleBufferBase64,CLR(sampleBase64Buffer),sampleCounter*2);
}


/*signal play back*/

void playSample(unsigned char *replaySampleBase64,unsigned char pin=-1){

    clearSample();
    base64Decode(_CS(sampleBufferBase64,replaySampleBase64));
    unsigned short *replaySample=sampleBuffer;


    unsigned short pinValue=(pin==(uint8_t)-1)?outputPortMask:(1<<pin);       //^ from 8 to 11 is the default

    unsigned long idleState=*(replaySample++);                                //~ the first address is used for the idle state
    idleState=idleState>>4;

    outputPort()=(idleState&outputPortMask)|(outputPort()&(~outputPortMask)); //^ setting to the idle state

    // unsigned short *calibratedSample=replaySample;
    // while(*calibratedSample){                                                 //^ calibrating before sample replay, for better accuracy  
    //     (*calibratedSample)*=calibratingFactor;
    //     calibratedSample++;
    // }

    


    while(*replaySample){
        outputPort()^=pinValue;                                               //^ toggle the pin state as we know the very first state
        delayTicks((*replaySample)*calibratingFactor);
        replaySample++;
    }

    outputPort()=(idleState&outputPortMask)|(outputPort()&(~outputPortMask)); //^ setting to the idle state
    return;
}





int main(void){

  /*init hardware*/

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();

  outputPort()=(((inputPort()&inputPortMask)>>4)&outputPortMask)|(outputPort()&(~outputPortMask));

  /*adding a call back for the console object, this might be used for debugging*/

  console.addConsole([&](unsigned char *cosnoleData,unsigned char autoNLCR){
    send(cosnoleData);
    if(autoNLCR)
      send((uint8_t*)"\r\n");
  });

  // console.log("------------app started !!");
  while(1){

    unsigned char *userRequest=getData();
    if(equalStrings(userRequest,(unsigned char *)"!get-key")){
      send(getSample());
    }
    else{
      playSample(userRequest);
    }


  }

}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_USART2_UART_Init(void)
{


  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }


}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
