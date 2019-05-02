#define SEESAW_PRIMARY_ADDRESS             0x49 // SeeSaw default

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/

    /** Module Base Addreses
     *  The module base addresses for different seesaw modules.
     */
    enum
    {
        SEESAW_STATUS_BASE = 0x00,
        SEESAW_GPIO_BASE = 0x01,
        SEESAW_SERCOM0_BASE = 0x02,

        SEESAW_TIMER_BASE = 0x08,
        SEESAW_ADC_BASE = 0x09,
        SEESAW_DAC_BASE = 0x0A,
        SEESAW_INTERRUPT_BASE = 0x0B,
        SEESAW_DAP_BASE = 0x0C,
        SEESAW_EEPROM_BASE = 0x0D,
        SEESAW_NEOPIXEL_BASE = 0x0E,
        SEESAW_TOUCH_BASE = 0x0F,
        SEESAW_KEYPAD_BASE = 0x10,
        SEESAW_ENCODER_BASE = 0x11,
    };

    /** GPIO module function addres registers
     */
    enum
    {
        SEESAW_GPIO_DIRSET_BULK = 0x02,
        SEESAW_GPIO_DIRCLR_BULK = 0x03,
        SEESAW_GPIO_BULK = 0x04,
        SEESAW_GPIO_BULK_SET = 0x05,
        SEESAW_GPIO_BULK_CLR = 0x06,
        SEESAW_GPIO_BULK_TOGGLE = 0x07,
        SEESAW_GPIO_INTENSET = 0x08,
        SEESAW_GPIO_INTENCLR = 0x09,
        SEESAW_GPIO_INTFLAG = 0x0A,
        SEESAW_GPIO_PULLENSET = 0x0B,
        SEESAW_GPIO_PULLENCLR = 0x0C,
    };

    /** status module function addres registers
     */
    enum
    {
        SEESAW_STATUS_HW_ID = 0x01,
        SEESAW_STATUS_VERSION = 0x02,
        SEESAW_STATUS_OPTIONS = 0x03,
        SEESAW_STATUS_TEMP = 0x04,
        SEESAW_STATUS_SWRST = 0x7F,
    };

    /** timer module function addres registers
     */
    enum
    {
        SEESAW_TIMER_STATUS = 0x00,
        SEESAW_TIMER_PWM = 0x01,
        SEESAW_TIMER_FREQ = 0x02,
    };
	
    /** ADC module function addres registers
     */
    enum
    {
        SEESAW_ADC_STATUS = 0x00,
        SEESAW_ADC_INTEN = 0x02,
        SEESAW_ADC_INTENCLR = 0x03,
        SEESAW_ADC_WINMODE = 0x04,
        SEESAW_ADC_WINTHRESH = 0x05,
        SEESAW_ADC_CHANNEL_OFFSET = 0x07,
    };

    /** Sercom module function addres registers
     */
    enum
    {
        SEESAW_SERCOM_STATUS = 0x00,
        SEESAW_SERCOM_INTEN = 0x02,
        SEESAW_SERCOM_INTENCLR = 0x03,
        SEESAW_SERCOM_BAUD = 0x04,
        SEESAW_SERCOM_DATA = 0x05,
    };

    /** neopixel module function addres registers
     */
    enum
    {
        SEESAW_NEOPIXEL_STATUS = 0x00,
        SEESAW_NEOPIXEL_PIN = 0x01,
        SEESAW_NEOPIXEL_SPEED = 0x02,
        SEESAW_NEOPIXEL_BUF_LENGTH = 0x03,
        SEESAW_NEOPIXEL_BUF = 0x04,
        SEESAW_NEOPIXEL_SHOW = 0x05,
    };

    /** touch module function addres registers
     */
    enum
    {
        SEESAW_TOUCH_CHANNEL_OFFSET = 0x10,
    };

    /** keypad module function addres registers
     */
    enum
    {
        SEESAW_KEYPAD_STATUS = 0x00,
        SEESAW_KEYPAD_EVENT = 0x01,
        SEESAW_KEYPAD_INTENSET = 0x02,
        SEESAW_KEYPAD_INTENCLR = 0x03,
        SEESAW_KEYPAD_COUNT = 0x04,
        SEESAW_KEYPAD_FIFO = 0x10,
    };

    /** keypad module edge definitions
     */
    enum 
    {
        SEESAW_KEYPAD_EDGE_HIGH = 0,
        SEESAW_KEYPAD_EDGE_LOW,
        SEESAW_KEYPAD_EDGE_FALLING,
        SEESAW_KEYPAD_EDGE_RISING,
    };

    /** encoder module edge definitions
     */
    enum
    {
        SEESAW_ENCODER_STATUS = 0x00,
        SEESAW_ENCODER_INTENSET = 0x02,
        SEESAW_ENCODER_INTENCLR = 0x03,
        SEESAW_ENCODER_POSITION = 0x04,
        SEESAW_ENCODER_DELTA = 0x05,
    };

#define ADC_INPUT_0_PIN 7
#define ADC_INPUT_1_PIN 6
#define ADC_INPUT_2_PIN 5
#define ADC_INPUT_3_PIN 4

#define PWM_0_PIN 16
#define PWM_1_PIN 17
#define PWM_2_PIN 18
#define PWM_3_PIN 19
#define PWM_4_PIN 11
#define PWM_5_PIN 10
#define PWM_6_PIN 9
#define PWM_7_PIN 8

