/******************************************************************************
  * @file    ADC.c
  * @author
  * @version
  * @date
  * @brief
  *****************************************************************************/

#include <msp430.h>
#include <stdint.h>
#include "ADC.h"
#include "main.h"

volatile unsigned int ADC_Result[16];                // 10-bit ADC conversion result array

volatile unsigned char HR_current_ADC_ready = 0;
volatile unsigned char BR_current_ADC_ready = 0;

void ADC_init(void)
{
    // Configure ADC
    P1SEL1 |= BR_CURRENT;               /* P1.0, A0 */
    P1SEL0 |= BR_CURRENT;

    P1SEL1 |= HR_CURRENT;               /* P1.1, A1 */
    P1SEL0 |= HR_CURRENT;

    P2SEL1 |= BR_FORWARD_SW;            /* P2.3, A6 */
    P2SEL0 |= BR_FORWARD_SW;

    P2SEL1 |= BR_REVERSE_SW;            /* P2.4, A7 */
    P2SEL0 |= BR_REVERSE_SW;

    P3SEL1 |= HR_REVERSE_SW;            /* P3.0, A12 */
    P3SEL0 |= HR_REVERSE_SW;

    P3SEL1 |= HR_FORWARD_SW;            /* P3.0, A13 */
    P3SEL0 |= HR_FORWARD_SW;

//    ADC10CTL0 |= ADC10SHT_2 + ADC10ON;        // ADC10ON, S&H=16 ADC clks
//    ADC10CTL1 |= ADC10SHP;                    // ADCCLK = MODOSC; sampling timer
//    ADC10CTL2 |= ADC10RES;                    // 10-bit conversion results
//
//    /*******************Analog PIN **********************/
//    ADC10MCTL0 |= ADC10INCH_12;                // A1 ADC input select; Vref=AVCC
//
//    ADC10IE |= ADC10IE0;                      // Enable ADC conv complete interrupt
//    __delay_cycles(100);                        // Wait for ADC ref to settle

//    ADC10CTL0 = ADC10SHT_4 + ADC10ON + ADC10MSC;    // S&H - 64 cycles; ADC ON; Multiple Conversions
//    ADC10CTL1 = ADC10SHP + ADC10CONSEQ_1;           // Sample-Hold Pulse selection; Sequence of Channels
//    ADC10CTL2 = ADC10RES;                           // 10-Bit Resolution
//    ADC10IE = ADC10IE0;                             // Enable Conversion Complete Interrupt
//
//    ADC10MCTL0 = ADC10INCH_15;        // We need A0, A1(Port 1), A6, A7(Port 2), A12, A13(Port 3)

    // Configure ADC10
      ADC10CTL0 = ADC10SHT_4 + ADC10MSC + ADC10ON;// 16ADCclks, MSC, ADC ON
      ADC10CTL1 = ADC10SHP + ADC10CONSEQ_1;     // sampling timer, s/w trig.,single sequence
      ADC10CTL2 = ADC10RES;                   // 8-bit resolution
      ADC10MCTL0 = ADC10INCH_15;                 // A0,A1,A2(EoS), AVCC reference

      // Configure DMA0 (ADC10IFG trigger)
      DMACTL0 = DMA0TSEL__ADC10IFG;             // ADC10IFG trigger
      __data20_write_long((uintptr_t) &DMA0SA,(uintptr_t) &ADC10MEM0);
                                                // Source single address
      __data20_write_long((uintptr_t) &DMA0DA,(uintptr_t) &ADC_Result[15]);
                                                // Destination array address
      DMA0SZ = 0x10;                            // 16 conversions
      //DMA0CTL = DMADT_4 + DMADSTINCR_2 + DMASRCBYTE + DMADSTBYTE + DMAEN + DMAIE;
      DMA0CTL = DMADT_4 + DMADSTINCR_2 + DMAEN + DMAIE;
                                                // Rpt Single Transfer, decrement dest, Source unchanged, byte access,
                                                // enable int after seq of convs


    __delay_cycles(1000);                        // Wait for ADC ref to settle
}

void ADC_convert (void)
{
    ADC10CTL0 |= ADC10ENC + ADC10SC;         // Sampling and conversion start
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=DMA_VECTOR
__interrupt void DMA0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(DMA_VECTOR))) DMA0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(DMAIV,16))
  {
    case  0: break;                          // No interrupt
    case  2:
      // sequence of conversions complete
        BR_current_ADC_ready = 1;
        HR_current_ADC_ready = 1;
      break;                                 // DMA0IFG
    case  4: break;                          // DMA1IFG
    case  6: break;                          // DMA2IFG
    case  8: break;                          // Reserved
    case 10: break;                          // Reserved
    case 12: break;                          // Reserved
    case 14: break;                          // Reserved
    case 16: break;                          // Reserved
    default: break;
  }
}

//// ADC10 interrupt service routine
//#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
//#pragma vector=ADC10_VECTOR
//__interrupt void ADC10_ISR(void)
//#elif defined(__GNUC__)
//void __attribute__ ((interrupt(ADC10_VECTOR))) ADC10_ISR (void)
//#else
//#error Compiler not supported!
//#endif
//{
//
//  switch(__even_in_range(ADC10IV,12))
//  {
//    case  0: break;                          // No interrupt
//    case  2: break;                          // conversion result overflow
//    case  4: break;                          // conversion time overflow
//    case  6: break;                          // ADC10HI
//    case  8: break;                          // ADC10LO
//    case 10: break;                          // ADC10IN
//    case 12:
//        ADC_Result[count] = ADC10MEM0;
//        if (count == 0)
//        {
//            BR_current_ADC_ready = 1;
//            count = 15;
//        }
//        else if (count == 1)
//        {
//            count--;
//            HR_current_ADC_ready = 1;
//        }
//        else
//            count--;
//        break;                          // Clear CPUOFF bit from 0(SR)
//    default: break;
//  }
//}
