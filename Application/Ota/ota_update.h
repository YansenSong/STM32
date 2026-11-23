#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H
#include <stdint.h>
#define OTA_UPDATE_PORT 5003u
void OtaUpdate_Init(void);
void OtaUpdate_Receive(const uint8_t *data, uint16_t length);
void OtaUpdate_SendReply(void);
uint8_t OtaUpdate_InMaintenance(void);
#endif
