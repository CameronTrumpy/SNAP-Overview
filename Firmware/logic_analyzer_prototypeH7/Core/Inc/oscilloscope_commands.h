/*
 * oscilloscope_commands.h
 *
 *  Created on: Nov 14, 2025
 *      Author: Andre
 */

#ifndef INC_OSCILLOSCOPE_COMMANDS_H_
#define INC_OSCILLOSCOPE_COMMANDS_H_

#include "snap_module.h"
#include "oscilloscope_ll.h"
#include "stdbool.h"
#include <stdint.h> // Required for UINT16_MAX

// Command handlers
void commandOSStartContinuous(struct CommandMessage *cmd, struct CommandResponse *resp);
void commandOSStopContinuous(struct CommandMessage *cmd, struct CommandResponse *resp);
void commandOSGetChunk(struct CommandMessage *cmd, struct CommandResponse *resp);
void commandOSConfig(struct CommandMessage *cmd, struct CommandResponse *resp);

#endif /* INC_OSCILLOSCOPE_COMMANDS_H_ */
