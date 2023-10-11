#ifndef ICS2_RX13T_H
#define ICS2_RX13T_H

#include <stdint.h>

#define   ICS_SCI1_PD3_PD5    (0x10U)
#define   ICS_SCI1_PB6_PB7    (0x11U)
#define   ICS_SCI5_PB6_PB7    (0x50U)
#define   ICS_SCI5_PB2_PB1    (0x51U)
#define   ICS_SCI5_P23_P24    (0x52U)
#define   ICS_SCI12_PB0_P94   (0xC0U)


void     ics2_init(void * addr, uint8_t port, uint8_t level, uint8_t speed, uint8_t mode);
void     ics2_watchpoint(void);
uint32_t ics2_version(void);

void ics_int_sci_eri(void);
void ics_int_sci_rxi(void);

#endif /* ICS2_RX13T_H */
