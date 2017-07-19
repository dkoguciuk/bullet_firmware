/**
 *  @file
 *  @author     Bartosz Borawski <sasim.michal@gmail.com>
 *  @date       02.12.2013
 *  @version    1.0
 *
 *  @brief		Biblioteka umozliwia wygodna prace i korzystanie z serwomechanizmu analogowego.
 *              Zostala stworzona na potrzeby projektu Heksapod. W glownym programie nalezy wywolac funkcje
 *              SERVO_Config().
 */

#ifndef ANALOG_SERVO_LIB_H_
#define ANALOG_SERVO_LIB_H_

#include "system.h"
#include "stm32f10x.h"

/**
 * @brief		Zmienna okreslajaca wartosc wypelnienia sygnalu.
 */
extern int width;

/**
 * @brief		Funkcja inicjujaca zegar dla portu i pinu serwa.
 */
void SERVO_RCC_Config(void);

/**
 * @brief		Konfiguracja pinu podlaczenia serwa.
 */
void SERVO_GPIO_Config(void);

/**
 * @brief		Funkcja konfigurujaca timer do pracy z serwem.
 */
void SERVO_TIM_BASE_Config(void);

/**
 * @brief		Konfiguracja PWM do wspolpracy z serwem.
 */
void SERVO_TIM_PWM_Config(void);

/**
 * @brief		Deklaracja funkcji inicjujacej serwo
 */
void SERVO_Config();

/**
 * @brief		Deklaracja funkcji ustawiajacej serwo na dany kat wychylenia
 */
uint8_t SERVO_SetAngle(uint8_t angle);

/**
 * @brief		Deklaracja funkcji czytajacej aktualny stan wychylenia serwa
 */
uint8_t SERVO_GetAngle(void);


#endif /* ANALOG_SERVO_LIB_H_ */
