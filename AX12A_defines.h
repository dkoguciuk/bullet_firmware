/**
 *  @file
 *  @author     MichalSasim <sasim.michal@gmail.com>
 *  @date       31.10.2013
 *  @version    1.1
 *
 *  @brief		Biblioteka umozliwia wygodna prace i korzystanie z serwomechanizmow Dynamixel AX12-A.
 *              Zostala stworzona na potrzeby projektu Heksapod. W glownym programie nalezy wywolac funkcje
 *              AX12A_Config() by moc w pelni korzystac z mozliwosci urzadzen.
 *              Nalezy ustawic odpowiednie priorytety przerwan od USART i TIM2 integrujac te biblioteke
 *              w programie wspolpracujacym z roznymi peryferiami.
 *
 *              All EEprom functions take 2ms to perform to make sure status packet form servo is sent.
 */

#ifndef AX12A_DEFINES_H
#define AX12A_DEFINES_H

// ======================================================================================
// ===================================== HEADERS ========================================
// ======================================================================================

#include "bullet_firmware_core.h"

#include "stm_lib/inc/stm32f10x_usart.h"
#include "stm_lib/inc/stm32f10x_gpio.h"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "stm_lib/inc/stm32f10x_tim.h"
#include "stm_lib/inc/misc.h"

// ======================================================================================
// =============================== PINS&PORTS DEFINES ===================================
// ======================================================================================


/**
 * 	@brief		USART periph number.
 */
#define USART_SV				USART2

/**
 * 	@brief		Timer periph numer.
 */
#define TIM_SV					TIM2

/**
 * 	@brief		Port definition for USART communication with PC.
 */
#define USART_SV_PORT			GPIOA

/**
 *	@brief		Transmit/Receive pin definition for USART communication with Servos.
 */
#define USART_SV_PIN_TX			GPIO_Pin_2

/**
 * 	@brief		Clock definition for USART.
 */
#define USART_SV_CLOCK			RCC_APB1Periph_USART2

/**
 * 	@brief		Clock definition for USART port.
 */
#define USART_SV_PORT_CLOCK		RCC_APB2Periph_GPIOA

/**
 * 	@brief 		Clock definition for TIM.
 */
#define TIM_SV_CLOCK			RCC_APB1Periph_TIM2

/**
 * 	@brief		USART IRQN.
 */
#define USART_SV_IRQ			USART2_IRQn

/**
 * 	@brief 		TIM IRQN.
 */
#define TIM_SV_IRQ				TIM2_IRQn

// ======================================================================================
// ================================ LIBRARY DEFINES =====================================
// ======================================================================================


/**
 * 	@brief		Definicja domyslnej wartosci wielkosci pakietu zwrotnego wyrazona w bajtach.
 */
#define DEFAULT_RETURN_PACKET_SIZE     	6

/**
 * 	@brief		Definicja wartosci predkosci transmisji USART.
 */
#define BAUD_RATE      					1000000

/**
 * 	@brief		Definicja adresu rozgloszeniowego.
 */
#define BROADCASTING_ID                0xFE

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego numer modelu serwa (mlodszy bajt).
 */
#define P_MODEL_NUMBER_L          0

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego numer modelu serwa (starszy bajt).
 */
#define P_MODEL_NUMBER_H          1

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego wersje firmware'u serwa.
 */
#define P_VERSION                 2

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego numer identyfikacyjny serwa.
 */
#define P_ID                      3

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego wartosc predkosci transmisji.
 */
#define P_BAUD_RATE               4

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego wartosc czasu po jakim wysylany jest pakiet zwrotny.
 */
#define P_RETURN_DELAY_TIME       5

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego dolna granice zakresu ruchu serwa (mlodszy bajt).
 */
#define P_CW_ANGLE_LIMIT_L        6

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego dolna granice zakresu ruchu serwa (starszy bajt).
 */
#define P_CW_ANGLE_LIMIT_H        7

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego gorna granice zakresu ruchu serwa (mlodszy bajt).
 */
#define P_CCW_ANGLE_LIMIT_L       8

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego gorna granice zakresu ruchu serwa (starszy bajt).
 */
#define P_CCW_ANGLE_LIMIT_H       9

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego maksymalna dopuszczalna temperature serwa.
 */
#define P_LIMIT_TEMPERATURE      11

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego minimalne napiecie zasilajace.
 */
#define P_DOWN_LIMIT_VOLTAGE     12

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego maksymalne napiecie zasilajace.
 */
#define P_UP_LIMIT_VOLTAGE       13

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego maksymalny moment obrotowy (mlodszy bajt) w pamieci RAM.
 */
#define P_MAX_TORQUE_L           14

/**
 * 	@brief		Definicja adresu [EEPROM] przechowujacego maksymalny moment obrotowy (starszy bajt) w pamieci RAM.
 */
#define P_MAX_TORQUE_H           15

/**
 * 	@brief		Definicja adresu [EEPROM] okreslajacego czy pakiet zwrotny ma byc wysylany.
 */
#define P_RETURN_LEVEL           16

/**
 * 	@brief		Definicja adresu [EEPROM] okreslajacego zachowanie sie LED'a w przypadku bledow.
 */
#define P_ALARM_LED              17

/**
 * 	@brief		Definicja adresu [EEPROM] okreslajacego zachowanie sie momentu obrotowego w przypadku napotkania bledow.
 */
#define P_ALARM_SHUTDOWN         18

//#define P_DOWN_CALIBRATION_L     20
//#define P_DOWN_CALIBRATION_H     21
//#define P_UP_CALIBRATION_L       22
//#define P_UP_CALIBRATION_H       23

/**
 * 	@brief		Definicja adresu [RAM] okreslajacego zalaczenie momentu obrotowego serwa.
 */
#define P_TORQUE_ENABLE          24

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego stan LED'a.
 */
#define P_LED                    25

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego wartosc marginesu dolnej odchylki polozenia.
 */
#define P_CW_COMPLIANCE_MARGIN   26

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego wartosc marginesu gornej odchylki polozenia.
 */
#define P_CCW_COMPLIANCE_MARGIN  27

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego wartosc nachylenia dolnej strefy odksztalcalnosci.
 */
#define P_CW_COMPLIANCE_SLOPE    28

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego wartosc nachylenia gornej strefy odksztalcalnosci.
 */
#define P_CCW_COMPLIANCE_SLOPE   29

/**
 * 	@brief		Definicja adresu [RAM] okreslajacego pozycje docelowa serwa (mlodszy bajt).
 */
#define P_GOAL_POSITION_L        30

/**
 * 	@brief		Definicja adresu [RAM] okreslajacego pozycje docelowa serwa (starszy bajt).
 */
#define P_GOAL_POSITION_H        31

/**
 * 	@brief		Definicja adresu [RAM] okreslajacego predkosc ruchu serwa do docelowej pozycji (mlodszy bajt).
 */
#define P_GOAL_SPEED_L           32

/**
 * 	@brief		Definicja adresu [RAM] okreslajacego predkosc ruchu serwa do docelowej pozycji (starszy bajt).
 */
#define P_GOAL_SPEED_H           33

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego maksymalny moment obrotowy (mlodszy bajt).
 */
#define P_TORQUE_LIMIT_L         34

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego maksymalny moment obrotowy (starszy bajt).
 */
#define P_TORQUE_LIMIT_H         35

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego aktualna pozycje serwa (mlodszy bajt).
 */
#define P_PRESENT_POSITION_L     36

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego aktualna pozycje serwa (starszy bajt).
 */
#define P_PRESENT_POSITION_H     37

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego aktualna predkosc serwa (mlodszy bajt).
 */
#define P_PRESENT_SPEED_L        38

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego aktualna predkosc serwa (starszy bajt).
 */
#define P_PRESENT_SPEED_H        39

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego aktualne obciazenie serwa (mlodszy bajt).
 */
#define P_PRESENT_LOAD_L         40
/**
 * 	@brief		Definicja adresu [RAM] przechowujacego aktualne obciazenie serwa (starszy bajt).
 */
#define P_PRESENT_LOAD_H         41

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego aktualna wartosc napiecia zasilajacego.
 */
#define P_PRESENT_VOLTAGE        42

/**
 * 	@brief		Definicja adresu [RAM] przechowujacego aktualna wartosc temperatury.
 */
#define P_PRESENT_TEMPERATURE    43

/**
 * 	@brief		Definicja adresu [RAM] zmiennej wykorzystywanej przy instrukcji REG_WRITE.
 */
#define P_REGISTERED_INSTRUCTION 44

/**
 * 	@brief		Definicja adresu [RAM] okreslajacego czy w danej chwili serwo wykonuje ruch (silnikiem).
 */
#define P_MOVING                 46

/**
 * 	@brief		Definicja adresu [RAM] blokady dostepu do wszyskich adresow z wyjatkiem adresow od 24 do 35.
 */
#define P_LOCK                   47

/**
 * 	@brief		Definicja adresu [RAM] okreslajacego minimalna wartosc pradu zasilajacego podczas pracy serwa (mlodszy bajt).
 */
#define P_PUNCH_L                48

/**
 * 	@brief		Definicja adresu [RAM] okreslajacego minimalna wartosc pradu zasilajacego podczas pracy serwa (starszy bajt).
 */
#define P_PUNCH_H                49

/**
 * 	@brief		Definicja instrukcji PING.
 */
#define INST_PING              	0X01

/**
 * 	@brief		Definicja instrukcji READ.
 */
#define INST_READ              	0X02

/**
 * 	@brief		Definicja instrukcji WRITE.
 */
#define INST_WRITE             	0X03

/**
 * 	@brief		Definicja instrukcji REG_WRITE.
 */
#define INST_REG_WRITE         	0X04

/**
 * 	@brief		Definicja instrukcji ACTION.
 */
#define INST_ACTION            	0X05

/**
 * 	@brief		Definicja instrukcji RESET.
 */
#define INST_RESET             	0X06

/**
 * 	@brief		Definicja instrukcji SYNC_WRITE.
 */
#define INST_SYNC_WRITE        	0X83


#endif /* AX12A_DEFINES_H */
