/*
 * FRDM-K64F: LDR + Temp + External LED + UART to ESP32 + Digital Twin
 *
 * LDR (analog)   -> A0 (PTB2, ADC0_SE12)
 * Green LED      -> A1 (PTB3, GPIO output)
 * LM35DZ Temp    -> A5 (PTC10, ADC1_SE6b)
 * ESP32 TX/RX    -> UART3 (PTC16 = RX, PTC17 = TX) at 9600 baud
 *
 * MODES:
 *   AUTO   - LED controlled by LDR threshold (< 1000 = ON)
 *   MANUAL - LED controlled by dashboard (Digital Twin)
 *
 * UART commands from ESP32:
 *   "CMD:LED_ON\n"   -> manual override, LED ON
 *   "CMD:LED_OFF\n"  -> manual override, LED OFF
 *   "CMD:AUTO\n"     -> return to automatic mode
 */

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_uart.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* ADC0 for LDR */
#define LDR_ADC             ADC0
#define LDR_CHANNEL         12U   /* ADC0_SE12 = PTB2 (A0) */
#define ADC_CHANNEL_GROUP   0U
#define LED_THRESHOLD       1000U

/* ADC1 for LM35DZ temp sensor */
#define TEMP_ADC            ADC1
#define TEMP_CHANNEL        6U    /* ADC1_SE6b = PTC10 (A5) */

/* External green LED on A1 = PTB3 */
#define LED_GPIO            GPIOB
#define LED_PORT            PORTB
#define LED_PIN             3U

/* UART3 for ESP32 */
#define ESP_UART            UART3
#define ESP_UART_CLKSRC     BUS_CLK
#define ESP_UART_BAUDRATE   9600U

/* Mode definitions */
#define MODE_AUTO   0
#define MODE_MANUAL 1

/* LM35DZ: 10mV per degree C, ADC ref = 3.3V, 12-bit = 4096 */
#define ADC_REF_VOLTAGE     3.3f
#define ADC_MAX_VALUE       4096.0f
#define LM35_MV_PER_DEG     10.0f

/*******************************************************************************
 * Variables
 ******************************************************************************/
static int currentMode = MODE_AUTO;
static int manualLedState = 0;
static char uartRxBuf[64];
static int uartRxIdx = 0;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void Init_ADC0(void);
static void Init_ADC1(void);
static uint32_t Read_ADC0(uint32_t channel);
static uint32_t Read_ADC1(uint32_t channel);
static float ReadTemperature(void);
static void Init_UART3(void);
static void UART3_SendString(const char *str);
static void UART3_CheckCommands(void);
static void Init_LED(void);
static void LED_On(void);
static void LED_Off(void);
static void Delay_ms(uint32_t ms);

/*******************************************************************************
 * Code
 ******************************************************************************/

/* ── ADC0 (LDR on A0) ── */
static void Init_ADC0(void)
{
    adc16_config_t adcConfig;

    ADC16_GetDefaultConfig(&adcConfig);
    adcConfig.resolution = kADC16_ResolutionSE12Bit;
    adcConfig.clockDivider = kADC16_ClockDivider4;
    ADC16_Init(LDR_ADC, &adcConfig);
    ADC16_DoAutoCalibration(LDR_ADC);
    ADC16_SetHardwareAverage(LDR_ADC, kADC16_HardwareAverageCount32);
}

static uint32_t Read_ADC0(uint32_t channel)
{
    adc16_channel_config_t chConfig;

    chConfig.channelNumber = channel;
    chConfig.enableInterruptOnConversionCompleted = false;
    chConfig.enableDifferentialConversion = false;

    ADC16_SetChannelConfig(LDR_ADC, ADC_CHANNEL_GROUP, &chConfig);

    while (!(ADC16_GetChannelStatusFlags(LDR_ADC, ADC_CHANNEL_GROUP) &
             kADC16_ChannelConversionDoneFlag))
    {
    }

    return ADC16_GetChannelConversionValue(LDR_ADC, ADC_CHANNEL_GROUP);
}

/* ── ADC1 (LM35DZ on A5) ── */
static void Init_ADC1(void)
{
    adc16_config_t adcConfig;

    ADC16_GetDefaultConfig(&adcConfig);
    adcConfig.resolution = kADC16_ResolutionSE12Bit;
    adcConfig.clockDivider = kADC16_ClockDivider4;
    ADC16_Init(TEMP_ADC, &adcConfig);
    ADC16_DoAutoCalibration(TEMP_ADC);
    ADC16_SetHardwareAverage(TEMP_ADC, kADC16_HardwareAverageCount32);

    /* A5 (PTC10) is on ADC1 mux B channel 6 -> select mux B */
    ADC16_SetChannelMuxMode(TEMP_ADC, kADC16_ChannelMuxB);
}

static uint32_t Read_ADC1(uint32_t channel)
{
    adc16_channel_config_t chConfig;

    chConfig.channelNumber = channel;
    chConfig.enableInterruptOnConversionCompleted = false;
    chConfig.enableDifferentialConversion = false;

    ADC16_SetChannelConfig(TEMP_ADC, ADC_CHANNEL_GROUP, &chConfig);

    while (!(ADC16_GetChannelStatusFlags(TEMP_ADC, ADC_CHANNEL_GROUP) &
             kADC16_ChannelConversionDoneFlag))
    {
    }

    return ADC16_GetChannelConversionValue(TEMP_ADC, ADC_CHANNEL_GROUP);
}

/* ── LM35DZ temperature conversion ── */
static float ReadTemperature(void)
{
    uint32_t adcVal = Read_ADC1(TEMP_CHANNEL);

    /* Convert ADC value to voltage, then to temperature
     * LM35: 10mV per °C
     * voltage = adcVal * (3.3 / 4096)
     * temp_C  = voltage / 0.01 = voltage * 100
     */
    float voltage = ((float)adcVal / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float tempC   = voltage * 100.0f;

    return tempC;
}

/* ── UART3 (ESP32) with interrupt-driven RX ── */

/*
 * K64F UART3 has only a 1-byte hardware RX FIFO.
 * At 9600 baud a new byte arrives every ~1 ms.  The main-loop
 * polling can't keep up, so bytes are lost to overrun.
 *
 * Fix: use the UART3 RX interrupt to capture every byte into a
 * 128-byte software ring buffer.  The main loop then drains the
 * ring buffer at its leisure – no data is ever lost.
 */

#define UART3_RING_SIZE 128
static volatile uint8_t  uart3Ring[UART3_RING_SIZE];
static volatile uint32_t uart3RingHead = 0;   /* ISR writes here  */
static volatile uint32_t uart3RingTail = 0;   /* main loop reads  */

/* ── UART3 RX/TX ISR ── */
void UART3_RX_TX_IRQHandler(void)
{
    uint32_t flags = UART_GetStatusFlags(ESP_UART);

    /* Clear any error flags so the UART doesn't stall */
    if (flags & (kUART_RxOverrunFlag | kUART_NoiseErrorFlag |
                 kUART_FramingErrorFlag | kUART_ParityErrorFlag))
    {
        UART_ClearStatusFlags(ESP_UART,
            kUART_RxOverrunFlag | kUART_NoiseErrorFlag |
            kUART_FramingErrorFlag | kUART_ParityErrorFlag);
    }

    /* Read every available byte into the ring buffer */
    while (kUART_RxDataRegFullFlag & UART_GetStatusFlags(ESP_UART))
    {
        uint8_t ch = UART_ReadByte(ESP_UART);
        uint32_t next = (uart3RingHead + 1U) % UART3_RING_SIZE;
        if (next != uart3RingTail)          /* buffer not full */
        {
            uart3Ring[uart3RingHead] = ch;
            uart3RingHead = next;
        }
    }
}

static void Init_UART3(void)
{
    uart_config_t config;

    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_Uart3);

    PORT_SetPinMux(PORTC, 16U, kPORT_MuxAlt3);
    PORT_SetPinMux(PORTC, 17U, kPORT_MuxAlt3);

    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = ESP_UART_BAUDRATE;
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(ESP_UART, &config, CLOCK_GetFreq(ESP_UART_CLKSRC));

    /* Enable RX data-ready interrupt → UART3_RX_TX_IRQHandler */
    UART_EnableInterrupts(ESP_UART, kUART_RxDataRegFullInterruptEnable |
                                    kUART_RxOverrunInterruptEnable);
    EnableIRQ(UART3_RX_TX_IRQn);
}

static void UART3_SendString(const char *str)
{
    UART_WriteBlocking(ESP_UART, (const uint8_t *)str, strlen(str));
}

/* ── Process commands from the ISR ring buffer ── */
static void UART3_CheckCommands(void)
{
    while (uart3RingTail != uart3RingHead)
    {
        uint8_t ch = uart3Ring[uart3RingTail];
        uart3RingTail = (uart3RingTail + 1U) % UART3_RING_SIZE;

        if (ch == '\n' || ch == '\r')
        {
            if (uartRxIdx > 0)
            {
                uartRxBuf[uartRxIdx] = '\0';

                if (strcmp(uartRxBuf, "CMD:LED_ON") == 0)
                {
                    currentMode = MODE_MANUAL;
                    manualLedState = 1;
                    LED_On();
                    PRINTF("[CMD] Manual LED ON\r\n");
                }
                else if (strcmp(uartRxBuf, "CMD:LED_OFF") == 0)
                {
                    currentMode = MODE_MANUAL;
                    manualLedState = 0;
                    LED_Off();
                    PRINTF("[CMD] Manual LED OFF\r\n");
                }
                else if (strcmp(uartRxBuf, "CMD:AUTO") == 0)
                {
                    currentMode = MODE_AUTO;
                    PRINTF("[CMD] Switched to AUTO mode\r\n");
                }

                uartRxIdx = 0;
            }
        }
        else
        {
            if (uartRxIdx < (int)(sizeof(uartRxBuf) - 1))
            {
                uartRxBuf[uartRxIdx++] = (char)ch;
            }
        }
    }
}

/* ── External Green LED on A1 (PTB3) ── */
static void Init_LED(void)
{
    gpio_pin_config_t ledConfig = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };

    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(LED_PORT, LED_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(LED_GPIO, LED_PIN, &ledConfig);
}

static void LED_On(void)
{
    GPIO_PortSet(LED_GPIO, 1U << LED_PIN);
}

static void LED_Off(void)
{
    GPIO_PortClear(LED_GPIO, 1U << LED_PIN);
}

/* ── Simple delay ── */
static void Delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    for (; ms > 0; ms--)
        for (i = 0; i < 7000; i++)
            ;
}

/* ── main ── */
int main(void)
{
    char buf[160];
    uint32_t ldr_val;
    float temp_val;
    int led_state;
    int tempInt, tempFrac;

    /* Board init */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    /* Peripheral init */
    Init_ADC0();
    Init_ADC1();
    Init_UART3();
    Init_LED();

    PRINTF("\r\n=== FRDM-K64F Sensor Monitor + Digital Twin ===\r\n");
    UART3_SendString("START\r\n");

    while (1)
    {
        /* Check for commands from ESP32 (Digital Twin) */
        UART3_CheckCommands();

        /* Read sensors */
        ldr_val  = Read_ADC0(LDR_CHANNEL);
        temp_val = ReadTemperature();

        /* Split float for printing (PRINTF may not support %f) */
        tempInt  = (int)temp_val;
        tempFrac = (int)((temp_val - (float)tempInt) * 100.0f);
        if (tempFrac < 0) tempFrac = -tempFrac;

        /* LED control depends on mode */
        if (currentMode == MODE_AUTO)
        {
            if (ldr_val < LED_THRESHOLD)
            {
                LED_On();
                led_state = 1;
            }
            else
            {
                LED_Off();
                led_state = 0;
            }
        }
        else
        {
            led_state = manualLedState;
        }

        /* Print to debug terminal */
        PRINTF("LDR: %u | Temp: %d.%02d C | LED: %s | Mode: %s\r\n",
               ldr_val, tempInt, tempFrac,
               led_state ? "ON" : "OFF",
               currentMode == MODE_AUTO ? "AUTO" : "MANUAL");

        /* Drain any pending commands before we tie up the TX line */
        UART3_CheckCommands();

        /* Send to ESP32 via UART3 */
        sprintf(buf, "LDR:%u,TEMP:%d.%02d,LED:%s,MODE:%s\r\n",
                ldr_val, tempInt, tempFrac,
                led_state ? "ON" : "OFF",
                currentMode == MODE_AUTO ? "AUTO" : "MANUAL");
        UART3_SendString(buf);

        /* Check again right after TX in case bytes arrived during send */
        UART3_CheckCommands();

        /* Wait 1 second, polling for UART commands every 10ms */
        {
            int i;
            for (i = 0; i < 100; i++)
            {
                Delay_ms(10);
                UART3_CheckCommands();
            }
        }
    }
}

/* Note: This project was developed with the assistance of AI coding tools. */
