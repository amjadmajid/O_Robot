#include "msp.h"
#include "UART0.h"
#include "clock.h"
#include "interruptHandler.h"

uint8_t read_buttons(){
    // P1->IN works in a negative logic fashion
    return (~(P1->IN) & BIT1 && ~(P1->IN) & BIT4 );
}

void press_buttons_to_go(void){
    P1->SEL0 &= ~(BIT1+BIT4);
    P1->SEL1 &= ~(BIT1+BIT4);
    P1->DIR &= ~(BIT1+BIT4);
    P1->REN |=  (BIT1+BIT4);
    P1->OUT |=  (BIT1+BIT4); // pull-up resistors

    while( read_buttons() == 0x00 ) ; // wait for press
    while( read_buttons() != 0x00 ) ; // wait for release
}

void initialize(){
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    disableInterrupts();

    clock_init_48MHz();
    UART0_Init();
    press_buttons_to_go();
    enableInterrupts();
}
