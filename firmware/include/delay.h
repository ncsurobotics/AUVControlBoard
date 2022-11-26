#pragma once


void delay_init(void);

/**
 * Blocking delay for given duration in us
 */
void delay_us(unsigned int us);

/**
 * Blocking delay for given duration in ms
 */
void delay_ms(unsigned int ms);
