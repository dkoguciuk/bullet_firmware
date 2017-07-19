#include "switch_lib.h"

void SWITCH_Config(void)
{
	//Ustawiamy potrzebne zegary
	SWITCH_RCCConfig();

	//Ustawiamy GPIO wykorzystywane przez switche
	SWITCH_GPIOConfig();
}

void SWITCH_RCCConfig(void)
{
	//Wlaczenie zegarow portu, do ktorego podlaczono microswitche
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void SWITCH_GPIOConfig(void)
{
	//Definicja struktury GPIO
    GPIO_InitTypeDef  GPIO_InitStructure;

	//Skonfiguruj piny microswitchy
    GPIO_InitStructure.GPIO_Pin = SWITCH1_Pin | SWITCH2_Pin | SWITCH3_Pin
    		                    | SWITCH4_Pin | SWITCH5_Pin | SWITCH6_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  //z podci¹gniêciem do masy
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SWITCH_Port, &GPIO_InitStructure);
}

//Funkcja odczytujaca stan microswitchy jest malo uniwersalna, zaimplementowana
//w sztywny sposób dla pinow 0, 1, 2, 3, 4, 5 GPIOB
//Bedzie sie ona wykonywac duzo szybciej niz gdyby stworzyc ta funkcje
//wykorzystujac makra.
uint8_t SWITCH_ReadPositions(void)
{
	uint16_t port_state = GPIO_ReadInputData(SWITCH_Port);
	uint8_t positions = (uint8_t)(port_state) & 0x3F;
	return positions;
}
