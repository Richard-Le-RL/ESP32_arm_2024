#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#define F_CPU 16000000UL
#define BAUD 9600
#define MYUBRR (F_CPU/16/BAUD-1)

#define PCA9685_ADDR 0x40
#define MODE1       0x00
#define PRESCALE    0xFE
#define LED0_ON_L   0x06
 
// ---------- USART ---------- 
// Used to send stuff to serial monitor
void USART_Init(unsigned int ubrr) {
    UBRR0H = (unsigned char)(ubrr>>8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1<<TXEN0);
    UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

void USART_Transmit(unsigned char data) {
    while (!( UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

void USART_Print(const char* str) {
    while (*str) USART_Transmit(*str++);
}

// ---------- TWI/I2C ----------
void TWI_Init(void) {
    TWSR = 0x00;
    TWBR = 0x48;
    TWCR = (1 << TWEN);
}

void TWI_Start(void) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void TWI_Stop(void) {
    TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void TWI_Write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
}

void PCA9685_Write(uint8_t reg, uint8_t data) {
    TWI_Start();
    TWI_Write((PCA9685_ADDR << 1));
    TWI_Write(reg);
    TWI_Write(data);
    TWI_Stop();
}

void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t reg = LED0_ON_L + 4 * channel;
    TWI_Start();
    TWI_Write(PCA9685_ADDR << 1);
    TWI_Write(reg);
    TWI_Write(on & 0xFF);
    TWI_Write((on >> 8) & 0xFF);
    TWI_Write(off & 0xFF);
    TWI_Write((off >> 8) & 0xFF);
    TWI_Stop();
}

void PCA9685_Init(void) {
    PCA9685_Write(MODE1, 0x00);
    _delay_ms(1);
    PCA9685_Write(MODE1, 0x10);
    PCA9685_Write(PRESCALE, 121); // 50Hz
    PCA9685_Write(MODE1, 0x00);
    _delay_ms(1);
    PCA9685_Write(MODE1, 0xA1);
}

// ---------- ADC ----------
void ADC_Init(void) {
    ADMUX = (1 << REFS0) | 1; // ADC1
    ADCSRA = (1 << ADEN)  | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_Read(void) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

uint16_t ADC_ReadChannel(uint8_t channel) {
    ADMUX = (1 << REFS0) | (channel & 0x07); // AVcc, ADCx
    // _delay_us(5); // Small delay for MUX to settle
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}
// ---------- MAIN ----------
int main(void) {
    char buffer[64];
    USART_Init(MYUBRR);
    TWI_Init();
    ADC_Init();
    PCA9685_Init();

    while (1) {
        for (uint8_t i = 0; i < 5; i++) {
            uint16_t adc = ADC_ReadChannel(i); // Read ADC0–ADC4
            uint16_t angle = (adc * 180UL) / 1023;
            uint16_t pwm = 102 + ((uint32_t)adc * (512 - 102)) / 1023;

            PCA9685_SetPWM(i, 0, pwm); // Channel 0–4

            snprintf(buffer, sizeof(buffer), "Servo %d | ADC: %d | Pulse: %u | Angle: %d\r\n", i, adc, pwm, angle);
            USART_Print(buffer);
        }
 
        _delay_ms(5); // Update rate
    }
}