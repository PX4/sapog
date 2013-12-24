/*
 * Pavel Kirienko <pavel.kirienko@gmail.com>
 *
 * Arduino Nano v3 connections:
 *   A7 - 3DR PowerBrick Voltage
 *   A6 - 3DR PowerBrick Current
 *   A0 - Vishay BPW24R (cathode) (anode to GND)
 *
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

// --------------------------

// Layout: Header [1], Checksum [1], Tach [2], Voltage [2], Current [2]
#define SERIAL_FRAME_HEADER     0xFA
#define SERIAL_FRAME_SIZE       (1 + 1 + 2 + 2 + 2)

struct serial_tx_frame
{
    uint8_t data[SERIAL_FRAME_SIZE];
    uint8_t next_index;
};

struct serial_tx_frame serial_tx_frame;

void serial_init(void)
{
    UCSR0A = 0;
    UCSR0B = (1 << 7) | (1 << 6) | (1 << 4) | (1 << 3);
    UCSR0C = (1 << 2) | (1 << 1);
    UBRR0 = 8;      // 115200 3.7%

    serial_tx_frame.next_index = SERIAL_FRAME_SIZE;
}

bool serial_send(uint16_t tach, uint16_t voltage, uint16_t current)
{
    if (serial_tx_frame.next_index < SERIAL_FRAME_SIZE)
        return false;

    serial_tx_frame.data[0] = SERIAL_FRAME_HEADER;

    memcpy(serial_tx_frame.data + 2, &tach, 2);
    memcpy(serial_tx_frame.data + 4, &voltage, 2);
    memcpy(serial_tx_frame.data + 6, &current, 2);
    
    serial_tx_frame.data[1] = 0;
    for (int i = 2; i < SERIAL_FRAME_SIZE; ++i)
        serial_tx_frame.data[1] += serial_tx_frame.data[i];

    serial_tx_frame.next_index = 0;

    return true;
}

void serial_poll(void)
{
    if (serial_tx_frame.next_index >= SERIAL_FRAME_SIZE)
        return;
    if (!(UCSR0A & (1 << 5)))
        return;
    UDR0 = serial_tx_frame.data[serial_tx_frame.next_index];
    serial_tx_frame.next_index++;
}

void serial_print(const char* str)
{
    while (*str) {
        while (!(UCSR0A & (1 << 5))) {}
        UDR0 = *str++;
    }
}

// --------------------------

enum adc_channels
{
    ADC_CHAN_OPTO = 0,
    ADC_CHAN_CURR = 6,
    ADC_CHAN_VOLT = 7
};

#define adc_is_ready() (ADCSRA & (1 << 4))
#define adc_read()     (ADCH)
#define adc_read16()   (ADC >> 6)

void adc_init(void)
{
    DIDR0 = 0x3F;
    ADMUX = (1 << 6) | (1 << 5);
    ADCSRB = 0;
    ADCSRA = (1 << 7) | (1 << 6) | (1 << 4) | (1 << 2) | (1 << 1);
    while (!adc_is_ready()) {}
}

void adc_select_channel(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF0) | channel;
}

void adc_start(void)
{
    ADCSRA |= (1 << 6) | (1 << 4);
}

// --------------------------

#if F_CPU != 16000000
#   error "Expected core clock - 16MHz"
#endif

void timer_init(void)
{
    TIMSK1 = 0;
    TCCR1A = 0;
    TCCR1B = (1 << 2); // 250 kHz clock
}

#define timer_stamp()   (TCNT1)

// --------------------------

void gpio_init(void)
{
    DDRB  = 1 << 5;  // LED on PB5, Sensor input on PB0
    PORTB = 0xFF;    // All pull-ups, LED on

    DDRC  = 1 << 1;  // Photodiode cathode on PC1
    PORTC = 1;       // Photodiode anode pulled up on PC0, rest are inputs (disabled by DIDR)

    DDRD  = 1 << 2;  // Testpoint on PD2
    PORTD = 0xFF;
}

#define testpoint_set() PORTD = 0xFF
#define testpoint_clr() PORTD = 0xFB

void led_set(bool on)
{
    if (on) PORTB |= 1 << 5;
    else    PORTB &= ~(1 << 5);
}

// --------------------------

#define OPTO_THRESHOLD      30
#define OPTO_DC_HISTORY_LEN 1024

uint8_t opto_update_dc_signal(uint8_t sample)
{
    static uint8_t hist[OPTO_DC_HISTORY_LEN];
    static uint16_t num_samples;
    static uint16_t index;
    static uint32_t sum;

    if (num_samples == OPTO_DC_HISTORY_LEN) {
        sum -= hist[index];
        hist[index] = sample;
        sum += sample;
        index += 1;
        if (index >= OPTO_DC_HISTORY_LEN)
            index = 0;
        return sum / OPTO_DC_HISTORY_LEN;
    } else {
        hist[num_samples++] = sample;
        sum += sample;
        return sum / num_samples;
    }
}

bool opto_detect_edge(int16_t sample)
{
    static bool in_peak = false;

    const int16_t dc = opto_update_dc_signal(sample);
    const int16_t ac = sample - dc;

    if (in_peak) {
        if (ac < (OPTO_THRESHOLD / 4))
            in_peak = false;
        return false;
    } else {
        if (ac > OPTO_THRESHOLD) {
            in_peak = true;
            return true;
        }
        return false;
    }
}

// --------------------------

#define wait_adc_poll_serial() while (!adc_is_ready()) { serial_poll(); }

int main(void)
{
    gpio_init();
    serial_init();
    adc_init();
    timer_init();

    adc_select_channel(ADC_CHAN_OPTO);
    adc_start();

    uint16_t prev_opto_timestamp = timer_stamp();
    bool opto_timed_out = true;

    while (1) {
        //testpoint_clr();

        wait_adc_poll_serial();
        adc_start();
        const uint8_t sample = adc_read();

        //testpoint_set();

        const uint16_t timestamp = timer_stamp();

        bool need_publish = false;
        uint16_t tach = 0;

        if (opto_detect_edge(sample)) {
            if (!opto_timed_out) {
                tach = timestamp - prev_opto_timestamp;
                need_publish = true;
            } else {
                opto_timed_out = false;
            }
            prev_opto_timestamp = timestamp;
        } else if (timestamp - prev_opto_timestamp > 50000) {
            opto_timed_out = true;
            prev_opto_timestamp = timestamp;
            need_publish = true;
        }

        if (need_publish) {
            wait_adc_poll_serial();

            adc_select_channel(ADC_CHAN_VOLT);
            adc_start();
            wait_adc_poll_serial();
            const uint16_t voltage = adc_read16();

            adc_select_channel(ADC_CHAN_CURR);
            adc_start();
            wait_adc_poll_serial();
            const uint16_t current = adc_read16();

            adc_select_channel(ADC_CHAN_OPTO);
            adc_start();

            const bool success = serial_send(tach, voltage, current);
            static bool failure_latch = false;
            if (!success)
                failure_latch = true;
            led_set(failure_latch);
        }
    }
}
