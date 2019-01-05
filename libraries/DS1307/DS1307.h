#ifndef DS1307_H
#define DS1307_H

#include <Arduino.h>
#include <Wire.h>

#define DS1307_ADDRESS  0x68

#define ERROR_RTC_STOPPED      100
#define ERROR_ADDRESS_TOO_HIGH 110


/** Adress time register */
const uint8_t DS1307_TIME_REG = 0x00;

/** Adresse et taille de la NVRAM du module RTC DS1307 */
const uint8_t DS1307_NVRAM_BASE = 0x08;
const uint8_t DS1307_NVRAM_SIZE = 56;

typedef struct {
       uint8_t seconds; /**!< Secondes 00 - 59 */
       uint8_t minutes; /**!< Minutes 00 - 59 */
       uint8_t hours;  /**!< Heures 00 - 23 (format 24h), 01 - 12 (format 12h) */
       uint8_t is_pm; /**!< Vaut 1 si l'heure est en format 12h et qu'il est l'aprés midi, sinon 0 */
       uint8_t day_of_week;  /**!< Jour de la semaine 01 - 07, 1 = lundi, 2 = mardi, etc.  */
       uint8_t days; /**!< Jours 01 - 31 */
       uint8_t months;  /**!< Mois 01 - 12 */
       uint8_t year;  /**!< Année au format yy (exemple : 16 = 2016) */
} DateTime_t;

class DS1307Class
{
  public:

    DS1307Class();
    
    uint8_t DS1307_init(void);
    uint8_t DS1307_isRunning(void);
    uint8_t DS1307_adjust_current_datetime(DateTime_t *datetime);
    uint8_t DS1307_read_current_datetime(DateTime_t *datetime);
        
    uint8_t DS1307_read_nvram_memory(uint8_t address);
    uint8_t DS1307_write_nvram_memory(uint8_t address, uint8_t data);
    
    uint8_t DS1307_getStatus(void);
    uint8_t DS1307_getAddress(void);
  
  private:
    uint8_t _address;
    uint8_t _last_status;

    uint8_t DS1307_bcd_to_decimal(uint8_t bcd);
    uint8_t DS1307_decimal_to_bcd(uint8_t decimal);
};

#endif /* DS1307_H */