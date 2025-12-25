#include <stdint.h>

/*
 * Example definitions
 * (Change these according to your microcontroller)
 */
#define LED_PIN     5

#define GPIO_DIR    (*(volatile uint32_t *)0x50000000) // Direction register
#define GPIO_SET    (*(volatile uint32_t *)0x50000004) // Output set register
#define GPIO_CLR    (*(volatile uint32_t *)0x50000008) // Output clear register

/* Software delay using polling (busy wait) */
void delay_ms(uint32_t ms)
{
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++)
    {
        for (j = 0; j < 1200; j++)
        {
            ;   // Busy wait
        }
    }
}

int main(void)
{
    /* Configure LED pin as OUTPUT */
    GPIO_DIR |= (1 << LED_PIN);

    while (1)
    {
        /* Turn LED ON */
        GPIO_SET |= (1 << LED_PIN);
        delay_ms(1000);

        /* Turn LED OFF */
        GPIO_CLR |= (1 << LED_PIN);
        delay_ms(1000);
    }
}
