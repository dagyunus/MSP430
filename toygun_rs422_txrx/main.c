#include <msp430.h>
#include <stdint.h>

#define RS422_RE_PIN    BIT6    // P9.6 -> RS422_RE#
#define RS422_DE_PIN    BIT7    // P9.7 -> RS422_DE

/*
 * Full-duplex RS422:
 * DE  = 1 -> TX aktif
 * RE# = 0 -> RX aktif
 */
#define RS422_TX_RX_ENABLE()   do {                  \
                                  P9OUT |=  RS422_DE_PIN;   \
                                  P9OUT &= ~RS422_RE_PIN;   \
                              } while(0)

static void Clock_Init(void)
{
    UCSCTL3 = SELREF_2;

    /*
     * ACLK  = REFO
     * SMCLK = DCOCLKDIV
     * MCLK  = DCOCLKDIV
     */
    UCSCTL4 = SELA_2 | SELS_4 | SELM_4;

    __bis_SR_register(SCG0);

    UCSCTL0 = 0x0000;
    UCSCTL1 = DCORSEL_2;
    UCSCTL2 = FLLD_1 | 30;     // yaklaşık 1 MHz

    __bic_SR_register(SCG0);

    __delay_cycles(250000);
}

static void RS422_UART_Init(void)
{
    /*
     * P9.4 -> UCA2TXD / RS422_TX
     * P9.5 -> UCA2RXD / RS422_RX
     */
    P9SEL |= BIT4 | BIT5;

    P9DIR |= BIT4;       // TX output
    P9DIR &= ~BIT5;      // RX input

    /*
     * P9.6 -> RE#
     * P9.7 -> DE
     */
    P9SEL &= ~(RS422_RE_PIN | RS422_DE_PIN);
    P9DIR |= RS422_RE_PIN | RS422_DE_PIN;

    /*
     * TX ve RX aktif
     */
    RS422_TX_RX_ENABLE();

    /*
     * UCA2 UART ayarı
     */
    UCA2CTL1 = UCSWRST;
    UCA2CTL0 = 0x00;             // 8-bit, no parity, 1 stop bit
    UCA2CTL1 |= UCSSEL_2;        // SMCLK

    /*
     * 115200 baud @ yaklaşık 1 MHz
     */
    UCA2BR0 = 8;
    UCA2BR1 = 0;
    UCA2MCTL = UCBRS_6;

    UCA2CTL1 &= ~UCSWRST;
}

static void RS422_SendByte(uint8_t data)
{
    while (!(UCA2IFG & UCTXIFG));

    UCA2TXBUF = data;

    while (UCA2STAT & UCBUSY);
}

static uint8_t RS422_ReceiveAvailable(void)
{
    return (UCA2IFG & UCRXIFG) ? 1u : 0u;
}

static uint8_t RS422_ReadByte(void)
{
    return UCA2RXBUF;
}

int main(void)
{
    uint8_t rx_data;

    WDTCTL = WDTPW | WDTHOLD;

    Clock_Init();
    RS422_UART_Init();

    /*
     * Açılışta sadece bir kez gönder.
     * Bunu görüyorsan kartın TX tarafı çalışıyor.
     */
    RS422_SendByte(0xA5);

    while (1)
    {
        if (RS422_ReceiveAvailable())
        {
            rx_data = RS422_ReadByte();

            /*
             * Veri geldiğini anlamak için önce F1 gönderiyoruz.
             * Sonra gelen veriyi aynen geri gönderiyoruz.
             */
            RS422_SendByte(0xF1);
            RS422_SendByte(rx_data);
        }
    }
}