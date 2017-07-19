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

#ifndef AX12A_LIBRARY_H
#define AX12A_LIBRARY_H

// ======================================================================================
// ===================================== HEADERS ========================================
// ======================================================================================

#include "bullet_firmware_core.h"
#include "AX12A_defines.h"

void USART2_IRQHandler(void);
void TIM2_IRQHandler(void);
void DMA1_Channel6_IRQHandler(void);
void DMA1_Channel7_IRQHandler(void);

// ======================================================================================
// ================================ CONFIG FUNCTIONS ====================================
// ======================================================================================

BulletStatus AX12A_Config(void);

// ======================================================================================
// ================================= PING FUNCTIONS =====================================
// ======================================================================================


BulletStatus  AX12A_Ping(uint8_t servo_id, uint8_t *value);


// ======================================================================================
// ================================ EEPROM FUNCTIONS ====================================
// ======================================================================================


/**
 * 	@brief		Funkcja odczytujaca numer modelu serwa.
 */
BulletStatus AX12A_ReadModelNumber(uint8_t servo_id, uint16_t *value);

/**
 * 	@brief		Funkcja odczytujaca wersje programu firmware serwa.
 */
BulletStatus AX12A_ReadFirmwareVersion(uint8_t servo_id, uint8_t *value);

/**
 * 	@brief		Funkcja zmieniajaca ID serwa (Broadcast ID)
 */
BulletStatus AX12A_WriteID(uint8_t new_ID);

/**
 * 	@brief		Funkcja ustawiajaca predkosc transmisji danych.
 */
BulletStatus AX12A_WriteBaudRate(uint8_t new_baud_rate);

/**
 * 	@brief		Funkcja odczytujaca wartosc czasu odpowiedzi serwa.
 */
BulletStatus AX12A_ReadReturnDelayTime(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca wartosc czasu odpowiedzi serwa.
 */
BulletStatus AX12A_WriteReturnRelayTime(uint8_t servo_number, uint8_t new_return_delay_time);

/**
 * 	@brief		Funkcja do odczytu dolnej granicy zakresu ruchu.
 */
BulletStatus AX12A_ReadLowerAngleLimit(uint8_t servo_number, uint16_t *value);

/**
 * 	@brief		Funkcja do zapisu dolnej granicy zakresu ruchu.
 */
BulletStatus AX12A_WriteLowerAngleLimit(uint8_t servo_number, uint16_t lower_angle_limit);

/**
 * 	@brief		Funkcja do odczytu gornej granicy zakresu ruchu.
 */
BulletStatus AX12A_ReadUpperAngleLimit(uint8_t servo_number, uint16_t *value);

/**
 * 	@brief		Funkcja do zapisu gornej granicy zakresu ruchu.
 */
BulletStatus AX12A_WriteUpperAngleLimit(uint8_t servo_number, uint16_t upper_angle_limit);

/**
 * 	@brief		Funkcja do odczytu limitu temperatury.
 */
BulletStatus AX12A_ReadTemperatureLimit(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja do zapisu limitu temperatury.
 */
BulletStatus AX12A_WriteTemperatureLimit(uint8_t servo_number, uint8_t temperature_limit);

/**
 * 	@brief		Funkcja do odczytu minimalnego napiecia zasilajacego.
 */
BulletStatus AX12A_ReadLowerVolgageLimit(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja do zapisu minimalnego napiecia zasilajacego.
 */
BulletStatus AX12A_WriteLowerVolgageLimit(uint8_t servo_number, uint8_t lower_voltage_limit);

/**
 * 	@brief		Funkcja do odczytu minimalnego napiecia zasilajacego.
 */
BulletStatus AX12A_ReadUpperVolgageLimit(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja do zapisu minimalnego napiecia zasilajacego.
 */
BulletStatus AX12A_WriteUpperVolgageLimit(uint8_t servo_number, uint8_t upper_voltage_limit);

/**
 *  @brief		Funkcja do odczytu maksymalnego momentu obrotowego z pamieci EEPROM.
 */
BulletStatus AX12A_ReadMaxTorqueEEPROM(uint8_t servo_number, uint16_t *value);

/**
 * 	@brief		Funkcja do zapisu maksymalnego momentu obrotowego z pamieci EEPROM.
 */
BulletStatus AX12A_WriteMaxTorqueEEPROM(uint8_t servo_number, uint16_t lower_angle_limit);

/**
 * 	@brief		Funkcja odczytujaca wartosc adresu decydujacego o wysylaniu pakietu zwrotnego.
 */
BulletStatus AX12A_ReadStatusReturnLevel(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca wartosc adresu decydujacego o wysylaniu pakietu zwrotnego.
 */
BulletStatus AX12A_WriteStatusReturnLevel(uint8_t servo_number, uint8_t status);

/**
 * 	@brief		Funkcja odczytujaca swiecenie LEDa podczas okreslonych bledow.
 */
BulletStatus AX12A_ReadAlarmLED(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca swiecenie LEDa podczas okreslonych bledow.
 */
BulletStatus AX12A_WriteAlarmLED(uint8_t servo_number, uint8_t value);

/**
 * 	@brief		Funkcja odczytujaca wylaczenie momentu obrotowego podczas okreslonych bledow.
 */
BulletStatus AX12A_ReadAlarmShutdown(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca wylaczenie momentu obrotowego podczas okreslonych bledow.
 */
BulletStatus AX12A_WriteAlarmShutdown(uint8_t servo_number, uint8_t value);


// ======================================================================================
// =================================== RAM FUNCTIONS ====================================
// ======================================================================================


/**
 * 	@brief		Funkcja odczytujaca wylaczenie momentu obrotowego podczas okreslonych bledow.
 */
BulletStatus AX12A_ReadTorqueEnable(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca wylaczenie momentu obrotowego podczas okreslonych bledow.
 */
BulletStatus AX12A_WriteTorqueEnable(uint8_t servo_number, uint8_t value);

/**
 * 	@brief		Funkcja odczytujaca stan diody LED.
 */
BulletStatus AX12A_ReadLED(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja do ustawienia stanu diody LED.
 */
BulletStatus AX12A_WriteLED(uint8_t servo_number, uint8_t state);

/**
 * 	@brief		Funkcja odczytujaca wartosc marginesu dolnej odchylki polozenia.
 */
BulletStatus AX12A_ReadLowerComplianceMargin(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca wartosc marginesu dolnej odchylki polozenia.
 */
BulletStatus AX12A_WriteLowerComplianceMargin(uint8_t servo_number, uint8_t value);

/**
 * 	@brief		Funkcja odczytujaca wartosc marginesu gornej odchylki polozenia.
 */
BulletStatus AX12A_ReadUpperComplianceMargin(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca wartosc marginesu gornej odchylki polozenia.
 */
BulletStatus AX12A_WriteUpperComplianceMargin(uint8_t servo_number, uint8_t value);

/**
 * 	@brief		Funkcja odczytujaca wartosc nachylenia dolnej strefy odksztalcalnosci.
 */
BulletStatus AX12A_ReadLowerComplianceSlope(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca wartosc nachylenia dolnej strefy odksztalcalnosci.
 */
BulletStatus AX12A_WriteLowerComplianceSlope(uint8_t servo_number, uint8_t value);

/**
 * 	@brief		Funkcja do odczytujaca wartosc nachylenia gornej strefy odksztalcalnosci.
 */
BulletStatus AX12A_ReadUpperComplianceSlope(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja do zapisujaca wartosc nachylenia gornej strefy odksztalcalnosci.
 */
BulletStatus AX12A_WriteUpperComplianceSlope(uint8_t servo_number, uint8_t value);

/**
 *  @brief		Funkcja do odczytu docelowej pozycji serwa.
 */
BulletStatus AX12A_ReadGoalPosition(uint8_t servo_number, uint16_t *value);

/**
 * 	@brief		Funkcja do zapisu docelowej pozycji serwa.
 */
BulletStatus AX12A_WriteGoalPosition(uint8_t servo_number, uint16_t goal_position);

/**
 *  @brief		Funkcja do odczytu predkosci obrotowej serwa.
 */
BulletStatus AX12A_ReadMovingSpeed(uint8_t servo_number, uint16_t *value);

/**
 * 	@brief		Funkcja do zapisu predkosci obrotowej serwa.
 */
BulletStatus AX12A_WriteMovingSpeed(uint8_t servo_number, uint16_t moving_speed);

/**
 *  @brief		Funkcja do odczytu maksymalnego momentu obrotowego z pamieci RAM.
 */
BulletStatus AX12A_ReadMaxTorqueRAM(uint8_t servo_number, uint16_t *value);

/**
 * 	@brief		Funkcja do zapisu maksymalnego momentu obrotowego z pamieci RAM.
 */
BulletStatus AX12A_WriteMaxTorqueRAM(uint8_t servo_number, uint16_t lower_angle_limit);

/**
 *  @brief		Funkcja do odczytu aktualnej pozycji serwa.
 */
BulletStatus AX12A_ReadPresentPosition(uint8_t servo_number, uint16_t *value);

/**
 *  @brief		Funkcja do odczytu aktualnej predkosci obrotowej serwa.
 */
BulletStatus AX12A_ReadPresentMovingSpeed(uint8_t servo_number, uint16_t *value);

/**
 *  @brief		Funkcja do odczytu aktualnego obciazenia serwa.
 */
BulletStatus AX12A_ReadPresentLoad(uint8_t servo_number, uint16_t *value);

/**
 *  @brief		Funkcja do odczytu aktualnej wartosci napiecia zasilajacego.
 */
BulletStatus AX12A_ReadPresentVoltage(uint8_t servo_number, uint8_t *value);

/**
 *  @brief		Funkcja do odczytu aktualnej temperatury.
 */
BulletStatus AX12A_ReadPresentTemperature(uint8_t servo_number, uint8_t *value);

/**
 *  @brief		Funkcja do odczytu aktualnej temperatury.
 */
BulletStatus AX12A_ReadRegisteredInstruction(uint8_t servo_number, uint8_t *value);

/**
 *  @brief		Funkcja pozwalajaca okreslic czy w danej chwili serwo wykonuje ruch (silnikiem).
 */
BulletStatus AX12A_ReadMoving(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja czytajaca stan blokady dostepu do wszyskich adresow z wyjatkiem adresow od 24 do 35.
 */
BulletStatus AX12A_ReadLock(uint8_t servo_number, uint8_t *value);

/**
 * 	@brief		Funkcja zapisujaca stan blokady dostepu do wszyskich adresow z wyjatkiem adresow od 24 do 35.
 */
BulletStatus AX12A_WriteLock(uint8_t servo_number, uint8_t value);

/**
 *  @brief		Funkcja do odczytu minimalnej wartosci pradu zasilajacego podczas pracy serwa (mlodszy bajt).
 */
BulletStatus AX12A_ReadPunch(uint8_t servo_number, uint16_t *value);

/**
 * 	@brief		Funkcja do zapisu minimalnej wartosci pradu zasilajacego podczas pracy serwa (starszy bajt).
 */
BulletStatus AX12A_WritePunch(uint8_t servo_number, uint16_t punch);

// ======================================================================================
// ==================================== MASS METHOD =====================================
// ======================================================================================

/**
 * 	@brief		Funkcja czytaj¹ca pozycjê wszystkich serw i zapisuj¹ca 18 wartoci uint16_t.
 */
BulletStatus AX12A_PingAll(uint8_t *value);

/**
 * 	@brief			Funkcja czytaj¹ca pozycjê wsszystkich serw.
 * 	@param[out]		values - tablica 36 bajtów odczytanych pozycji ka¿dego serwa
 * 	@param[in]		little_endian - parametr specyfikuj¹cy, czy zapisywane dane pozwinny byæ
 * 					typu little endian (1) lub big_endian (0).
 */
BulletStatus AX12A_ReadAllPosition(uint8_t *values, uint8_t little_endian);

/**
 * 	@brief			Funkcja czytaj¹ca pozycjê, prêdkoæ i obci¹¿enie wszystkich serw (6 bajtów na serwo)
 * 	@param[out]		values - tablica 108 bajtów odczytanych pozycji ka¿dego serwa
 * 	@param[in]		little_endian - parametr specyfikuj¹cy, czy zapisywane dane pozwinny byæ
 * 					typu little endian (1) lub big_endian (0).
 */
BulletStatus AX12A_ReadAllPositionSpeedLoad(uint8_t *values, uint8_t little_endian);


/**
 * 	@brief			Funkcja zapisuj¹ca pozycjê wszystkich serw w konkretnej nodze.
 * 	@param[out]		values - tablica 6 bajtów odczytanych pozycji ka¿dego serwa
 * 	@param[in]		little_endian - parametr specyfikuj¹cy, czy zapisywane dane pozwinny byæ
 * 					typu little endian (1) lub big_endian (0).
 */
BulletStatus AX12A_WriteLegGoalPosition(uint8_t leg_number, uint8_t *values, uint8_t little_endian);

/**
 * 	@brief			Funkcja zapisuj¹ca pozycjê wszystkich serw robota.
 * 	@param[out]		values - tablica 6 bajtów odczytanych pozycji ka¿dego serwa
 * 	@param[in]		little_endian - parametr specyfikuj¹cy, czy zapisywane dane pozwinny byæ
 * 					typu little endian (1) lub big_endian (0).
 */
BulletStatus  AX12A_WriteAllGoalPosition(uint8_t *values, uint8_t little_endian);

/**
 * 	@brief			Funkcja w³¹czaj¹ca lub wy³¹czaj¹ca moment si³y przy³o¿ony do osi serwa.
 * 	@param[out]		value - moment jest wy³¹czany w przypadku 0 lub w³¹czany w przypadku ka¿dej
 * 					innej wartoci
 */
BulletStatus  AX12A_WriteAllTorqueEnable(uint8_t value);

// ======================================================================================
// ================================ PRIVATE FUNCTIONS ===================================
// ======================================================================================

BulletStatus  AX12A_ReadByte (uint8_t servo_id, uint8_t control_table_address, uint8_t *value);
BulletStatus  AX12A_ReadWord (uint8_t servo_id, uint8_t control_table_address, uint8_t *value, uint8_t little_endian);
BulletStatus  AX12A_WriteByte(uint8_t servo_id, uint8_t control_table_address, uint8_t value);
BulletStatus  AX12A_WriteWord(uint8_t servo_id, uint8_t control_table_address, uint8_t *values, uint8_t little_endian);

BulletStatus AX12A_ReadBytes(uint8_t servo_id, uint8_t control_table_address, uint8_t bytes[], uint8_t size, uint8_t little_endian);

// ======================================================================================
// ==================================== DEBUGGING =======================================
// ======================================================================================

//FOR DEBUGGING
BulletStatus AX12A_SyncWriteGoalPosition(uint8_t message[]);

#endif /* AX12A_LIBRARY_H */
