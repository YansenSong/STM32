#ifndef UPPER_CMD_H
#define UPPER_CMD_H

#include <stdint.h>

extern volatile uint32_t dbg_cmd_found_cnt;
extern volatile uint32_t dbg_cmd_ok_cnt;
extern volatile uint32_t dbg_cmd_checksum_err_cnt;
extern volatile uint32_t dbg_cmd_header_drop_cnt;
extern volatile uint8_t  dbg_last_cmd[26];
extern volatile uint32_t dbg_last_checksum_calc;
extern volatile uint32_t dbg_last_checksum_recv;

void UpperCmd_Init(void);
void UpperCmd_Process(void);

#endif
