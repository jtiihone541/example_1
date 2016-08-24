/*
 * measure_task.c
 *
 * Created: 7.8.2016 17:10:46
 *  Author: Juha_Normal
 */ 
#include <asf.h>
#include <core_cm3.h>
#include <string.h>
#include <semphr.h>
#include <FreeRTOS.h>
#define PIO_PORT PIO_PA2

#define AM2031_MAX_SAMPLES 42
#define AM2031_PIN_CHANGE_INTERRUPT 0
#define AM2031_FALLING_EDGE_INTERRUPT 1

/* All below are system tics, i.e. 1ms */
#define INTRA_DIGIT_DELAY 400
#define INTER_DIGIT_DELAY 800
#define INTER_PARAM_DELAY 1600
#define ERROR_DISPLAY_DELAY 10

typedef struct
{
    uint8_t bit_counter;
    uint8_t interrupt_type;
    uint32_t zero_bit_limit;
    uint32_t bit_time_table[AM2031_MAX_SAMPLES];
    uint32_t abs_time_table[AM2031_MAX_SAMPLES];
    uint32_t databytes[AM2031_MAX_SAMPLES];
    
} am2031_interrupt_data_t;

typedef struct
{
    uint32_t    humidity_int;
    uint32_t    temp_int;
    float       humidity_float;
    float       temp_float;
    uint8_t     data_valid;
} am2301_processed_data_t;

am2031_interrupt_data_t am2031_interrupt_data;
am2301_processed_data_t am2301_data;
xSemaphoreHandle    semaphore;
volatile uint32_t        current_systick;

void init_am2031_interrupt_data(void)
{
    am2031_interrupt_data.bit_counter = 0;
    am2031_interrupt_data.interrupt_type = AM2031_FALLING_EDGE_INTERRUPT;
    
    /* By default, Atmel ASF configures Arduino due to use 84MHz clock, which is used as direcly */
    /* by Systick (no prescaler used), meaning one systick counter step equals 12ns */
    /* "zero bit limit" 8000 equals to 95us, shorter pulses from AM2301 represents "0", longer "1" */
    am2031_interrupt_data.zero_bit_limit = 8000;
    memset(am2031_interrupt_data.bit_time_table, 0, AM2031_MAX_SAMPLES);
    memset(am2031_interrupt_data.databytes, 0, AM2031_MAX_SAMPLES);
    return;
}

void convent_am2301_data(void)
{
    uint32_t    conversion, i, calculated_parity;
    
    memset((void *)&am2301_data, sizeof(am2301_data), 0);
    am2301_data.data_valid = 0; /* By default, it is false */
    
    conversion = 0;
    for(i = 0; i < 16; i++)
    {
        /* Convert humidity. Bits are from MSB->LSB order */
        conversion <<= 1;
        conversion |= am2031_interrupt_data.databytes[i] & 1;
    }
    am2301_data.humidity_int = conversion;
    
    conversion = 0;
    for(i = 0; i < 16; i++)
    {
        /* Convert temperature. Bits are from MSB->LSB order */
        conversion <<= 1;
        conversion |= am2031_interrupt_data.databytes[i+16] & 1;
    }
    am2301_data.temp_int = conversion;
    
    /* Check parity */
    
    conversion = 0;
    for(i = 0; i < 8; i++)
    {
        conversion <<= 1;
        conversion |= am2031_interrupt_data.databytes[i+32] & 1;
    }
    calculated_parity = (am2301_data.humidity_int & 0xff) + ((am2301_data.humidity_int >> 8) & 0xff) + (am2301_data.temp_int & 0xff) + ((am2301_data.temp_int >> 8) & 0xff);
    calculated_parity &= 0xff;
    
    if (calculated_parity == conversion)
    {
        /* Parity correct! */
        am2301_data.data_valid = 1;
    }

    return;
}

void start_measurement(void)
{
    /* This is called when AM2031 measurement is being started, by pulling low the output pin*/
    return;
}

void stop_measurement(void)
{
    /* This is called by handle_am2031_bit when the last bit has been received */
    //init_pin_signal();
    //NVIC_DisableIRQ(PIOA_IRQn);
    //send_message(1, AM2301_DATA_RECEIVED, 0);
    //return;
}

void handle_am2031_bit(uint32_t bit_time)
{
    uint8_t bit_value = 0;
    static uint32_t previous_bit_time;
    uint32_t actual_bit_time;

    am2031_interrupt_data.bit_counter++;
    if (am2031_interrupt_data.bit_counter < 3)
    {
        /* First 2 bits "falling edges" are the "start of acknowledge and start of first databit -> ignore those */
        previous_bit_time = bit_time;
        return;
    }
    /* bits 2-42 are the actual data bits */
    if (previous_bit_time < bit_time)
    {
        /* SysTick has wrapped */
        actual_bit_time = (SysTick->LOAD - bit_time) + previous_bit_time;
    }
    else
    {
        actual_bit_time = previous_bit_time-bit_time;
    }
    previous_bit_time=bit_time;
    if (actual_bit_time > am2031_interrupt_data.zero_bit_limit)
    {
        /* bit time is longer than shorter zero bit time -> it must be binary '1' */
        bit_value = 1;
    }
    else
    {
        bit_value = 0;
    }
    /* Store the bit */
    am2031_interrupt_data.bit_time_table[am2031_interrupt_data.bit_counter - 3] = actual_bit_time;
    am2031_interrupt_data.databytes[am2031_interrupt_data.bit_counter - 3] = bit_value;
    am2031_interrupt_data.abs_time_table[am2031_interrupt_data.bit_counter - 3] = bit_time;
    if (am2031_interrupt_data.bit_counter == AM2031_MAX_SAMPLES)
    {
        stop_measurement();
    }
    return;
}
void measure_data_handler(const uint32_t id, const uint32_t index)
{
    uint32_t timestamp = 0;
    if ((id == ID_PIOA) && (index == PIO_PORT))
    {
        /* Pin change occured in correct port */
        if ((PIOA->PIO_PDSR & PIO_PORT) == 0)
        {
            /* Pin change is falling edge, because port databit is zero - as a result of falling edge... */
            current_systick = SysTick->VAL;
            xSemaphoreGiveFromISR(semaphore, NULL);
        }        
    }
    return;
}

void blink_led(uint32_t times)
{
    uint8_t j;
    
    for(j = 0; j < times; j++)
    {
        /* LED on */
        pio_set_output(PIOB, PIO_PB27, HIGH, DISABLE, DISABLE);
        vTaskDelay(INTRA_DIGIT_DELAY);
        pio_set_output(PIOB, PIO_PB27, LOW, DISABLE, DISABLE);
        vTaskDelay(INTRA_DIGIT_DELAY);
    }
    return;
}

void blink_error(void)
{
    uint8_t i;
    
    for(i = 0; i < 100; i++)
    {
        pio_set_output(PIOB, PIO_PB27, HIGH, DISABLE, DISABLE);
        vTaskDelay(ERROR_DISPLAY_DELAY);
        pio_set_output(PIOB, PIO_PB27, LOW, DISABLE, DISABLE);
        vTaskDelay(ERROR_DISPLAY_DELAY);
    }
}
void output_am2301_data()
{
    uint32_t temp_lo, temp_hi, hum_lo, hum_hi;
    
    temp_hi = am2301_data.temp_int / 100;
    temp_lo = (am2301_data.temp_int - 100 * temp_hi) / 10;
    
    hum_hi = am2301_data.humidity_int / 100;
    hum_lo = (am2301_data.humidity_int - 100 * hum_hi) / 10;
    switch (am2301_data.data_valid)
    {
        case 1:
            blink_led(temp_hi);
            vTaskDelay(INTER_DIGIT_DELAY);
            blink_led(temp_lo);
            vTaskDelay(INTRA_DIGIT_DELAY);
            blink_led(hum_hi);
            vTaskDelay(INTER_DIGIT_DELAY);
            blink_led(hum_lo);
            break;
            
        default:
            /* Data is not valid, blink an error sequence */
            blink_error();
            break;
    }
    
}
void measure_task(void *parameters)
{
    uint32_t i;
    
    pmc_enable_periph_clk(ID_PIOA);
    pio_set_output(PIOB, PIO_PB27, LOW, DISABLE, DISABLE);
    pio_set_output(PIOA, PIO_PORT, HIGH, DISABLE, ENABLE);
    vTaskDelay(100);
    
    /* Do a dummy read sequence first to awake AM2301 */
    pio_set_output(PIOA, PIO_PORT, LOW, DISABLE, ENABLE);
    vTaskDelay(5);  /* Keep port pin in low state for 5 ticks */
    pio_set_input(PIOA, PIO_PORT, PIO_PULLUP|PIO_DEGLITCH);
    vTaskDelay(2000);
    pio_set_output(PIOA, PIO_PORT, HIGH, DISABLE, ENABLE);
    vTaskDelay(100);
    vSemaphoreCreateBinary(semaphore);
    while(1)
    {
        /* Main loop of task */
        init_am2031_interrupt_data();

        /* Give a 5ms low signal for AM2301 to start measurement */
        pio_set_output(PIOA, PIO_PORT, LOW, DISABLE, ENABLE);
        vTaskDelay(5);  /* Keep port pin in low state for 5 ticks (5ms) */
        
        /* Now change it to input mode, to get response from AM2301 */
        pio_set_input(PIOA, PIO_PORT, PIO_PULLUP|PIO_DEGLITCH);
        pio_handler_set(PIOA, ID_PIOA, PIO_PORT, PIO_IT_FALL_EDGE, measure_data_handler);
        pio_enable_interrupt(PIOA, PIO_PORT);
        for(i = 0; i < 1000; i++);
        NVIC_ClearPendingIRQ(PIOA_IRQn);
        i = PIOA->PIO_PSR;
        NVIC_EnableIRQ(PIOA_IRQn);
        for (i = 0; i < 43; i++)
        {
            /* Bit loop for data sent by AM2301 */
            /* Port A ISR gives semaphore and have also set the "current_systick" from SysTick timer */
            xSemaphoreTake(semaphore,10);
            
            handle_am2031_bit(current_systick);
        }
        NVIC_DisableIRQ(PIOA_IRQn);
        pio_disable_interrupt(PIOA, PIO_PORT);
        pio_set_output(PIOA, PIO_PORT, HIGH, DISABLE, ENABLE);
        convent_am2301_data();
       
        
        /* Output temp & humidity via LED */
        output_am2301_data();
        vTaskDelay(10000);
    }
    return;
}