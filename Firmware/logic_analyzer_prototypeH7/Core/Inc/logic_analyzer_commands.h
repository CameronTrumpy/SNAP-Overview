/*
 * logic_analyzer_commands.h
 *
 *  Created on: Oct 20, 2025
 *      Author: Andre
 */

#ifndef LOGIC_ANALYZER_COMMANDS_H_
#define LOGIC_ANALYZER_COMMANDS_H_

#include "snap_module.h"
#include "logic_analyzer_ll.h"
#include "stdbool.h"
#include <stdint.h> // Required for UINT16_MAX

// Command handlers
void commandLAStartContinuous(struct CommandMessage *cmd, struct CommandResponse *resp);
void commandLAStopContinuous(struct CommandMessage *cmd, struct CommandResponse *resp);
void commandLAGetChunk(struct CommandMessage *cmd, struct CommandResponse *resp);
void commandLAConfig(struct CommandMessage *cmd, struct CommandResponse *resp);

#endif /* LOGIC_ANALYZER_COMMANDS_H_ */
