#ifndef FUNCTION_H
#define FUNCTION_H
#include "main.h"

extern uint16_t del_val;

void led_control(void);
uint16_t crc16_traker(uint8_t *packet, uint8_t size_packet);
void command_shell(void);
#endif //FUNCTION_H
