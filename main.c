#include "stm32f10x.h"

void delay(uint32_t d)
{
    while(d--);
}

void LCD_Command(uint8_t c)
{
    GPIOB->BRR = (1<<0);
    GPIOB->ODR = (GPIOB->ODR & 0x00FF) | (c<<8);
    GPIOB->BSRR = (1<<1);
    delay(2000);
    GPIOB->BRR = (1<<1);
}

void LCD_Data(uint8_t d)
{
    GPIOB->BSRR = (1<<0);
    GPIOB->ODR = (GPIOB->ODR & 0x00FF) | (d<<8);
    GPIOB->BSRR = (1<<1);
    delay(2000);
    GPIOB->BRR = (1<<1);
}

void LCD_Init(void)
{
    delay(50000);
    LCD_Command(0x38);
    LCD_Command(0x0C);
    LCD_Command(0x01);
}

void ADC_Init(void)
{
    RCC->APB2ENR |= (1<<2) | (1<<9);

    GPIOA->CRL &= ~(0xF<<0);   // PA0 analog

    ADC1->SMPR2 = 7;
    ADC1->CR2 |= 1;
    delay(10000);
}

uint16_t ADC_Read(void)
{
    ADC1->SQR3 = 0;
    ADC1->CR2 |= (1<<22);
    while(!(ADC1->SR & (1<<1)));
    return ADC1->DR;
}

int main(void)
{
    uint16_t val;

    RCC->APB2ENR |= (1<<3);
    GPIOB->CRL = 0x00000033;
    GPIOB->CRH = 0x33333333;

    LCD_Init();
    ADC_Init();

    while(1)
    {
        val = ADC_Read();

        LCD_Command(0x80);
        LCD_Data((val/1000)%10 + '0');
        LCD_Data((val/100)%10  + '0');
        LCD_Data((val/10)%10   + '0');
        LCD_Data(val%10 + '0');

        delay(300000);
    }
}
