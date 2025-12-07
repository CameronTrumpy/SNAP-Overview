/*
 * la_os_commands.h
 *
 *  Created on: Nov 15, 2025
 *      Author: Andre
 */

#ifndef INC_LA_OS_COMMANDS_H_
#define INC_LA_OS_COMMANDS_H_

#include "snap_master.h"

int la_stream_init(void);

int command_get_chunk(uint32_t num_chunks, uint8_t cdc_port, CommandCode command, volatile uint8_t *stop_flag);

// LA COMMANDS
int command_la_start_continuous(void);

int command_la_stop_continuous(void);

int command_la_config(uint32_t freq);

int command_la_get_chunk(uint32_t num_chunks, uint8_t cdc_port);
// END LA

// OS COMMANDS
int command_os_start_continuous(void);

int command_os_stop_continuous(void);

int command_os_config(uint32_t freq);

int command_os_get_chunk(uint32_t num_chunks, uint8_t cdc_port);
// END OS


#endif /* INC_LA_OS_COMMANDS_H_ */
