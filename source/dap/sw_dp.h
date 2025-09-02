#ifndef __SW_DP_H__
#define __SW_DP_H__

void SWJ_Pins(const uint8_t *request, uint8_t *response);
void SWJ_Sequence(uint32_t count, const uint8_t *data);

#if (DAP_SWD != 0)
void SWD_Sequence(uint32_t info, const uint8_t *swdo, uint8_t *swdi);
uint8_t SWD_Transfer(uint32_t request, uint32_t *data);
#endif

#endif /* __SW_DP_H__ */
