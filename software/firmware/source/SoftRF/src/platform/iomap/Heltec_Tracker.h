
// GNSS module
#if defined(CONFIG_IDF_TARGET_ESP32S3)

#define SOC_GPIO_PIN_HELTRK_GNSS_RX     33
#define SOC_GPIO_PIN_HELTRK_GNSS_TX     34
#define SOC_GPIO_PIN_HELTRK_GNSS_RST    20 // 35 is LED
#define SOC_GPIO_PIN_HELTRK_GNSS_PPS    19 // 36 is Vext_Ctrl
#define SOC_GPIO_PIN_HELTRK_GNSS_EN     23 /* active LOW */

#else 

#define SOC_GPIO_PIN_HELTRK_GNSS_RX     33
#define SOC_GPIO_PIN_HELTRK_GNSS_TX     17 // 34 is invalid TX for ESP32
#define SOC_GPIO_PIN_HELTRK_GNSS_RST    35
#define SOC_GPIO_PIN_HELTRK_GNSS_PPS    36
#define SOC_GPIO_PIN_HELTRK_GNSS_EN     23 /* active LOW */

#endif

// SX1262
#define SOC_GPIO_PIN_HELTRK_MOSI        10
#define SOC_GPIO_PIN_HELTRK_MISO        11
#define SOC_GPIO_PIN_HELTRK_SCK         9
#define SOC_GPIO_PIN_HELTRK_SS          8
#define SOC_GPIO_PIN_HELTRK_RST         12
#define SOC_GPIO_PIN_HELTRK_BUSY        13
#define SOC_GPIO_PIN_HELTRK_DIO1        14

// TFT
#define SOC_GPIO_PIN_HELTRK_TFT_MOSI    42
#define SOC_GPIO_PIN_HELTRK_TFT_MISO    SOC_UNUSED_PIN
#define SOC_GPIO_PIN_HELTRK_TFT_SCK     41
#define SOC_GPIO_PIN_HELTRK_TFT_SS      38
#define SOC_GPIO_PIN_HELTRK_TFT_DC      40
#define SOC_GPIO_PIN_HELTRK_TFT_RST     39
#define SOC_GPIO_PIN_HELTRK_TFT_BL_V03  45 /* V1.0 PCB marking */
#define SOC_GPIO_PIN_HELTRK_TFT_BL_V05  21 /* V1.1 PCB marking */
#define SOC_GPIO_PIN_HELTRK_TFT_EN      46 /* active LOW */

// OLED
// Hardware pin definitions for Heltec and TTGO-V1 LoRa-32 Boards with OLED SSD1306 I2C Display - Heltec LoRa 32(V2)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define SOC_GPIO_PIN_HELTRK_OLED_RST    21
#define SOC_GPIO_PIN_HELTRK_OLED_SDA    17
#define SOC_GPIO_PIN_HELTRK_OLED_SCL    18
#else
// Hardware pin definitions for Heltec and TTGO-V1 LoRa-32 Boards with OLED SSD1306 I2C Display - Heltec LoRa 32(V3)
#define SOC_GPIO_PIN_HELTRK_OLED_RST    16
#define SOC_GPIO_PIN_HELTRK_OLED_SDA    4
#define SOC_GPIO_PIN_HELTRK_OLED_SCL    15
#endif


// 1st I2C bus
#define SOC_GPIO_PIN_HELTRK_SDA         6
#define SOC_GPIO_PIN_HELTRK_SCL         7

// LED
#if defined(CONFIG_IDF_TARGET_ESP32S3)

#define SOC_GPIO_PIN_HELTRK_LED         SOC_UNUSED_PIN /* 35 /*18 /* white, active HIGH */

#else

#define SOC_GPIO_PIN_HELTRK_LED         SOC_UNUSED_PIN /* 25 /* white, active HIGH */

#endif

// Misc.
#define SOC_GPIO_PIN_HELTRK_VEXT_EN     3 /* V0.3 - active LOW, V0.5 - HIGH */
#define SOC_GPIO_PIN_HELTRK_ADC_EN      2

// V0.5 only: 32768 Hz crystal
#define SOC_GPIO_PIN_HELTRK_XP          15
#define SOC_GPIO_PIN_HELTRK_XN          16


//  Heltec specific
#define DEFAULT_VREF            1100    // Default VREF use if no e-fuse calibration
#define VBATT_SAMPLE            500     // Battery sample rate in ms
#define VBATT_SMOOTH            50      // Number of averages in sample
#define ADC_READ_STABILIZE      5       // in ms (delay from GPIO control and ADC connections times)
#define LO_BATT_SLEEP_TIME      10*60*1000*1000     // How long when low batt to stay in sleep (us)

#if (defined(CONFIG_IDF_TARGET_ESP32S3))
#define VBATT_GPIO              37
#define VOLTAGE_DIVIDER         4.90    // Lora has 390k/100k voltage divider so need to reverse that reduction via (390k+100k)/100k
#else
#define VBATT_GPIO              21      // Heltec GPIO to toggle VBatt read connection ... WARNING!!! This also connects VEXT to VCC=3.3v so be careful what is on header.  Also, take care NOT to have ADC read connection in OPEN DRAIN when GPIO goes HIGH
#define VOLTAGE_DIVIDER         3.20    // Lora has 220k/100k voltage divider so need to reverse that reduction via (220k+100k)/100k on vbat GPIO37 or ADC1_1 (early revs were GPIO13 or ADC2_4 but do NOT use with WiFi.begin())
#endif
