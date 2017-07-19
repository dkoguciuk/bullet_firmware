#include "analog_servo_lib.h"

int width = 0;

void SERVO_Config()
{
	SERVO_RCC_Config();
	SERVO_GPIO_Config();
	SERVO_TIM_BASE_Config();
	SERVO_TIM_PWM_Config();
}

uint8_t SERVO_SetAngle(uint8_t angle)
{
	if (angle>=-45 && angle<=45)
	{
		TIM3->CCR2 =(uint16_t)(280 + (2*angle));
		return 0;
	}
	else
		return -1;
}

uint8_t SERVO_GetAngle(void)
{
	return (uint8_t)((TIM3->CCR2-280)/2);
}

void SERVO_RCC_Config(void)
{
	//wlaczenie taktowania dla timera, portu serwa i funkcji alternatywnych
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
}

void SERVO_GPIO_Config(void)
{
	//definicja struktury GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	//ustawienie pinu serwa jako wyjscie push-pull
	GPIO_InitStructure.GPIO_Pin = SERVO_Pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SERVO_Port, &GPIO_InitStructure);
}

/**
 *do obslugi serwa potrzeba sygnalu o okresie 20ms, ale
 *czestotliwosc TIM3 jest dwukrotnie mniejsza
 *od czestotliwosci zegara systemowego
 */

/**
 * Serwo modelarskie GO-09 porusza sie w zakresie -45 do +45 stopni przy okresie
 * wypelnienia * od okolo 1.9 milisekundy do 3.7 milisekund na 20 milisekund. Z tego
 * wynika, ze dziesiec stopni to okolo 0.0001 milisekundy, czyli 10 mikrosekund - taka
 * ma byc rozdzielczosc timera, czyli clock po podzieleniu przez preskaler, czyli
 * preskaler ma sie rownac:
 * 36000000 * 0,00001 = 360.
 *
 * Dalej prowadzac rozumowanie okres przebiegu ma byc rowny 20ms, czyli period rowny jest:
 * 0,02 / 0,00001 = 2000
 */
void SERVO_TIM_BASE_Config(void)
{
	//deklaracja struktury TIM
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	//ustawienie preskalera wedlug obliczen
	TIM_TimeBaseStructure.TIM_Prescaler = 360-1;

	//wybranie wartosci okresu sygnalu
	TIM_TimeBaseStructure.TIM_Period = 2000-1;

	//brak podzialu zegara
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	//brak licznika powtorzen
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	//zliczanie zboczem narastajacym
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	//wstepna inicjalizacja timera
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
}

/**
 * Teraz zakladajac, ze pozycja neutralna jest dla wypelnienia 2,8ms, mamy wypelnienie:
 * 0,0028 / 0,00001 = 280
 */

void SERVO_TIM_PWM_Config(void)
{
	//deklaracja struktury TIM_OC
	TIM_OCInitTypeDef TIM_OCInitStructure;

	//wlaczenie timera w tryb PWM
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;

	//uruchomienie mozliwosci czytania stanu wyjscia
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	//polaryzacja kanalu wysoka
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//wstepne wypelnienie sygnalu odpowiadajace zerowemu katowi wychylenia
	TIM_OCInitStructure.TIM_Pulse = 280;

	//inicjalizacja kanalu drugiego timera 3
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	//uruchomienie kanalu drugiego timera 3
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//wlaczenie buforowania
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	//uruchomienie TIM3 z PWM
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

