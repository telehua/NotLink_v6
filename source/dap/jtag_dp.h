#ifndef __JTAG_DP_H__
#define __JTAG_DP_H__

#if (DAP_JTAG != 0)
extern void JTAG_Sequence(uint32_t info, const uint8_t *tdi, uint8_t *tdo);
extern void JTAG_IR(uint32_t ir);
extern uint32_t JTAG_ReadIDCode(void);
extern void JTAG_WriteAbort(uint32_t data);
extern uint8_t JTAG_Transfer(uint32_t request, uint32_t *data);
#endif

#endif /* __JTAG_DP_H__ */
