/**
 *  @file
 *  @author     Daniel Koguciuk <daniel.koguciuk@gmail.com>
 *  @date       26.10.2013
 *  @version    1.3
 *
 *  @brief		Biblioteka obudowuje uzycie peryferium SPI na uzytek projektu Heksapod, by jego uzycie
 *  			bylo wygodniejsze. Z poziomu programu glownego wystarczy wywolac funkcje SPI_Config();
 *  			i juz peryferium SPI bedzie prawidlowo dzialac. Po zaprojektowaniu systemu przerwan nalezy
 *  			ustawic odpowiednie priorytety przerwania od SPI. Poza tym trzeba pamietac, by dostosowac
 *  			zawartosc funkcji SPI_PostProcess() wedlug zaprojektowanego protokolu komunikacyjnego!
 */

#ifndef SPI_H
#define SPI_H

// ======================================================================================
// ===================================== HEADERS ========================================
// ======================================================================================

#include "bullet_firmware_core.h"
#include "AX12A_library.h"

/**
 * 	@brief		Definicja funkcji obslugujacej przerwanie od SPI.
 */
void SPI1_IRQHandler(void);
void TIM3_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);


// ======================================================================================
// ==================================== VARIABLES =======================================
// ======================================================================================


// ======================================================================================
// =============================== PINS&PORTS DEFINES ===================================
// ======================================================================================


/**
 * 	@brief		Definicja numeru peryferium SPI.
 */
#define SPI					SPI1

/**
 * 	@brief		Definicja na ktorym porcie sa wystawione piny SPI.
 */
#define SPI_PORT			GPIOA

/**
 * 	@brief		Definicja numeru pinu SCK.
 */
#define SPI_PIN_SCK			GPIO_Pin_5

/**
 * 	@brief		Definicja numeru pinu MISO.
 */
#define SPI_PIN_MISO		GPIO_Pin_6

/**
 * 	@brief		Definicja numeru pinu MOSI.
 */
#define SPI_PIN_MOSI		GPIO_Pin_7

/**
 *	@brief		Definicja zegara peryferium SPI.
 */
#define SPI_CLK				RCC_APB2Periph_SPI1

/**
 * 	@brief		Definicja zegara portu GPIO dla SPI.
 */
#define SPI_GPIO_CLK		RCC_APB2Periph_GPIOA

/**
 * 	@brief		Definicja przerwania dla SPI.
 */
#define SPI_IRQn          	SPI1_IRQn


#define SPI_TIM				TIM3
#define SPI_TIM_CLOCK		RCC_APB1Periph_TIM3
#define SPI_TIM_IRQ			TIM3_IRQn

// ======================================================================================
// ================================ CONFIG FUNCTIONS ====================================
// ======================================================================================

/**
 * 	@brief		Funkcja inicjujaca dzialanie SPI.
 */
void SPI_Config(void);

#endif /* SPI_H */
