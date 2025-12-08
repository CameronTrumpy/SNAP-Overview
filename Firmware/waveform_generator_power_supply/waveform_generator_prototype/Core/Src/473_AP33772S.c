/*
 * 473_AP33772S.c
 *
 *  Created on: Oct 7, 2025
 *      Author: camer
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "473_AP33772S.h"
#include "stm32g4xx_hal_i2c.h"

extern I2C_HandleTypeDef hi2c1;

/* -------------------------------------------------------------------------- */
/*                                API Functions                               */
/* -------------------------------------------------------------------------- */

void AP33772S_Init(AP33772S_t *dev, I2C_HandleTypeDef *hi2c)
{
    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;

    //set to negative value s.t. we know when it has been properly updated by the MapPPSAVSInfo function
    dev->indexAVSUser = -1;
    dev->indexPPSUser = -1;
}


bool AP33772S_Begin(AP33772S_t *dev)
{
    dev->tgtAddr = CMD_SRCPDO;
    uint8_t len = 26;
    if (AP33772S_i2c_read(dev, len, I2C_TIMEOUT) != HAL_OK){
    	return false;
    }

    //populate raw PDO data
    for (int i = 0; i < 26; i += 2)
    {
        int idx = i / 2;
        dev->SRC_SPRandEPRpdoArray[idx].byte0 = dev->readBuf[i];
        dev->SRC_SPRandEPRpdoArray[idx].byte1 = dev->readBuf[i + 1];
    }

    //map data to PPS indices
    AP33772S_MapPPSAVSInfo(dev);
    AP33772S_SetTRCurve(dev);

    return true;
}


void AP33772S_MapPPSAVSInfo(AP33772S_t *dev)
{
  for(int i = 1; i<=13; i++)
  {
    if(i < 8 && dev->SRC_SPRandEPRpdoArray[i-1].pps.type == 1)
    {
      printf("Found PPS profile (Slot %d)\r\n", i);
      dev->indexPPSUser = i;
    }
    else if(i >= 8 && dev->SRC_SPRandEPRpdoArray[i-1].avs.type == 1)
    {
        printf("Found AVS profile (Slot %d)\r\n", i);
      dev->indexAVSUser = i;
    }
  }
}

// CheckAlive w/ either OPMODE or SYSTEM registers
uint8_t AP33772S_CheckAlive(AP33772S_t *dev){
    dev->tgtAddr = CMD_SYSTEM;
    if(AP33772S_i2c_read(dev, 1, I2C_TIMEOUT)!= HAL_OK)
        return AP_DEAD;

    return AP_ALIVE;
}

void AP33772S_ReadPDOs(AP33772S_t *dev)
{
	dev->tgtAddr = CMD_SRCPDO;
    uint8_t len = 26;

    // Request PDO objects from PSU

    AP33772S_i2c_read(dev, len, I2C_TIMEOUT);

    printf("Detected Source PDOs\r\n");

    for (int i = 0; i < 13; i++)
    {
        uint16_t raw = (dev->readBuf[i * 2 + 1] << 8) | dev->readBuf[i * 2];
        SRC_SPRandEPR_PDO_Fields pdo;
        pdo.byte0 = dev->readBuf[i * 2];
        pdo.byte1 = dev->readBuf[i * 2 + 1];

        // Skip unused PDOs
        if (raw == 0x0000){
        	continue;
        }else{
        	dev->validPDOs++;
        }

        //Printing for debug info
        printf("PDO %2d:\r\n", i + 1);
        printf("  Raw Value   : 0x%04X\r\n", raw);
        printf("  Type        : %s\r\n", pdo.fixed.type ? "Programmable (PPS/AVS)" : "Fixed Supply");
        printf("  Detect      : %s\r\n", pdo.fixed.detect ? "Valid" : "Not detected");
        float voltage = 0.0f;
        float current = 0.0f;
        if (pdo.fixed.type == PDO_FIXED) {
            // Fixed PDO
            voltage = (float)pdo.fixed.voltage_max * 0.05f;
            current = (float)pdo.fixed.current_max * 0.25f;  // example: 250mA per LSB
            printf("  Voltage Max : %.2f V\r\n", voltage);
            printf("  Current Max : %.2f A\r\n", current);
            printf("  Peak Current: %u\r\n", pdo.fixed.peak_current);
        } else {
            // PPS or AVS PDO
            voltage = (float)pdo.pps.voltage_max * 0.1f;
            current = AP33772S_currentMapRev(pdo.pps.current_max);
            printf("  Voltage Max : %.2f V\r\n", voltage);
            printf("  Voltage Min : %.2f V\r\n", (float)pdo.pps.voltage_min * 0.1f);
            printf("  Current Max : %.2f A\r\n", current);
        }

        printf("\r\n");
    }
}


bool AP33772S_SetFixedPDO(AP33772S_t *dev, int pdoIndex)
{
  RDO_DATA_T rdoData;

  // Sanity check include, check if the value is in EPR range (index < 8) and also AVS mode
  if(pdoIndex < 8 && dev->SRC_SPRandEPRpdoArray[pdoIndex-1].pps.type == 0)
  {
    printf("Type is Fixed.\r\n"); // DEBUG

    rdoData.REQMSG_Fields.PDO_INDEX = pdoIndex;
    rdoData.REQMSG_Fields.VOLTAGE_SEL = dev->SRC_SPRandEPRpdoArray[pdoIndex-1].fixed.voltage_max;  // Output Voltage in 200mV units
    rdoData.REQMSG_Fields.CURRENT_SEL = dev->SRC_SPRandEPRpdoArray[pdoIndex-1].fixed.current_max;

    dev->tgtAddr = CMD_PD_REQMSG;
    dev->writeBuf[1] = rdoData.byte0;  // Store the lower 8 bits (voltage)
    dev->writeBuf[2] = rdoData.byte1 & 0x0F;  // Store the upper 4 bits (current sel)
    dev->writeBuf[2] |= (pdoIndex &0x0F) << 4; //set PDO slot to use
    if(AP33772S_i2c_write(dev, 3, I2C_TIMEOUT)!= HAL_OK){
    	return false;
    }
  }else{
	  return false;
  }
  return true;
}



bool AP33772S_SetPPSPDO(AP33772S_t *dev, int pdoIndex, int target_mV, int max_mA)
{
  RDO_DATA_T rdoData;

  int voltage_min_decoded;
  // Sanity check include, check if the value is in EPR range (index < 8) and also AVS mode
  if(pdoIndex < 8 && dev->SRC_SPRandEPRpdoArray[pdoIndex-1].pps.type == 1)
  {
    printf("Type is PPS.\r\n"); // DEBUG
    // Now that we are in PPS mode

    rdoData.REQMSG_Fields.PDO_INDEX = pdoIndex;

    if(AP33772S_currentMap(max_mA) > dev->SRC_SPRandEPRpdoArray[pdoIndex-1].pps.current_max)
    {
      printf("PPS Current not in range.\r\n"); // DEBUG
      return false; // Check if current setting is in range
    }

    //Decode voltage_min
    if(dev->SRC_SPRandEPRpdoArray[pdoIndex-1].pps.voltage_min > 0) voltage_min_decoded = 3300;

    if(target_mV < voltage_min_decoded ||
    		target_mV > dev->SRC_SPRandEPRpdoArray[pdoIndex-1].pps.voltage_max*100 )
    {
    	printf("PPS Voltage not in range.\r\n"); // DEBUG
      return false; // Check if current setting is in range
    }

    rdoData.REQMSG_Fields.VOLTAGE_SEL = target_mV/100;  // Output Voltage in 200mV units
    rdoData.REQMSG_Fields.CURRENT_SEL = AP33772S_currentMap(max_mA);

    dev->tgtAddr = CMD_PD_REQMSG;
    dev->writeBuf[1] = rdoData.byte0;  // Store the lower 8 bits (voltage)
    dev->writeBuf[2] = rdoData.byte1 & 0x0F;  // Store the upper 4 bits (current sel)
    dev->writeBuf[2] |= (pdoIndex &0x0F) << 4; //set PDO slot to use
    if(AP33772S_i2c_write(dev, 3, I2C_TIMEOUT)!= HAL_OK){
    	return false;
    }
  }else{
	  return false;
  }
  return true;
}



HAL_StatusTypeDef AP33772S_SetAVSPDO(AP33772S_t *dev, int voltage_mV, int current_mA)
{

    dev->tgtAddr = CMD_PD_REQMSG;
    dev->writeBuf[1] = voltage_mV / 20; // 20mV units
    dev->writeBuf[2] = current_mA / 50; // 50mA units
    dev->writeBuf[3] = 0x00;

    printf("[AP33772S] fix AVS: %d mV, %d mA\r\n", voltage_mV, current_mA);
    return AP33772S_i2c_write(dev, 4, I2C_TIMEOUT);
}


// Set NTC thermistor parameters
void AP33772S_SetTRCurve(AP33772S_t *dev){
    // For the Murata NCG18XH103F0SRB
    // https://pim.murata.com/asset/pim4/ntcForTemperatureSensor/NTHCG270_TXT_NTCFORTEMPERATURESENSOR?lastModifiedDatetime=20250707192731
    // TR25 = 10,000
    // TR50 =  4,160
    // TR75	=  1,924
    // TR100 =  973
    dev->tgtAddr = CMD_TR25;
    dev->writeBuf[1] = (10000 & 0xFF);
    dev->writeBuf[2] = (10000 >>8) & 0xFF;
    AP33772S_i2c_write(dev, 2, I2C_TIMEOUT);

    dev->tgtAddr = CMD_TR50;
    dev->writeBuf[1] = (4160 & 0xFF);
    dev->writeBuf[2] = (4160 >>8) & 0xFF;
    AP33772S_i2c_write(dev, 2, I2C_TIMEOUT);

	dev->tgtAddr = CMD_TR75;
    dev->writeBuf[1] = (1924 & 0xFF);
    dev->writeBuf[2] = (1924 >>8) & 0xFF;
    AP33772S_i2c_write(dev, 2, I2C_TIMEOUT);

	dev->tgtAddr = CMD_TR100;
    dev->writeBuf[1] = (973 & 0xFF);
    dev->writeBuf[2] = (973 >>8) & 0xFF;
    AP33772S_i2c_write(dev, 2, I2C_TIMEOUT);
}

bool AP33772S_SetSYSTEM(AP33772S_t *dev, int VOUTCTL)
{
    dev->tgtAddr = CMD_SYSTEM;
    if(VOUTCTL == 0){
        dev->writeBuf[1] = 0b00010001; //turn off
    }else{
    	dev->writeBuf[1] = 0b00010010; //turn on
    }

//    dev->writeBuf[1] = sysreg_val | (~VOUTCTL & 0b1);  // 0 = Auto VOUT control, 1 = VOUT force OFF
    if(AP33772S_i2c_write(dev, 2, I2C_TIMEOUT)!= HAL_OK){
    	return false;
    }
    return true;
}


float AP33772S_readREQVoltage(AP33772S_t *dev)
{
	dev->tgtAddr = CMD_VREQ;
    float ret = -1;
    if(AP33772S_i2c_read(dev, 2, I2C_TIMEOUT) == HAL_OK){
        ret = 0.05f * (float)((dev->readBuf[1] << 8) | dev->readBuf[0]);
    }
    return ret;
}

float AP33772S_readREQCurrent(AP33772S_t *dev)
{
	dev->tgtAddr = CMD_IREQ;
    float ret = -1;
    if(AP33772S_i2c_read(dev, 2, I2C_TIMEOUT)  == HAL_OK){
    	ret = 0.010f * (float)((dev->readBuf[1] << 8) | dev->readBuf[0]); // LSB 10mA
    }
    return ret;
}

/* -------------------------------------------------------------------------- */
/*                          Performance Parameter Monitoring				  */
/* -------------------------------------------------------------------------- */

float AP33772S_readVoltage(AP33772S_t *dev)
{
	dev->tgtAddr = CMD_VOLTAGE;
    float ret = -1;
    if(AP33772S_i2c_read(dev, 2, I2C_TIMEOUT) == HAL_OK){
		ret = 0.08f * (float)((dev->readBuf[1] << 8) | dev->readBuf[0]);
    }
    return ret;
}

float AP33772S_readCurrent(AP33772S_t *dev)
{
	dev->tgtAddr = CMD_CURRENT;
    float ret = -1;
    if(AP33772S_i2c_read(dev, 2, I2C_TIMEOUT) == HAL_OK){
    	ret = 0.024f * 0.001f * (float)((dev->readBuf[1] << 8) | dev->readBuf[0]);
    }
    return ret;
}

uint8_t AP33772S_readTemp(AP33772S_t *dev)
{
	dev->tgtAddr = CMD_TEMP;
    uint8_t ret = 255;
    if(AP33772S_i2c_read(dev, 2, I2C_TIMEOUT) == HAL_OK){
		ret = (dev->readBuf[1] << 8) | dev->readBuf[0];
    }
    return ret;
}

/* -------------------------------------------------------------------------- */
/*                               Local Helpers                                */
/* -------------------------------------------------------------------------- */
int AP33772S_currentMap(int current)
{
  // Check if the value is out of bounds
  if (current < 0 || current > 5000) {
      return -1; // Return -1 for invalid inputs
  }

  // If value is below 1250, return 0
  if (current < 1250) {
      return 0;
  }

  // Calculate the result for ranges above 1250
  return ((current - 1250) / 250) + 1;
}

float AP33772S_currentMapRev(int currentIdx)
{
	  switch (currentIdx) {
	    case 0:
	      return 1.24f;
	      break;
	    case 1:
		  return 1.49f;
	      break;
	    case 2:
		  return 1.74f;
	      break;
	    case 3:
	    	return 1.99f;
	      break;
	    case 4:
	    	return 2.24f;
	      break;
	    case 5:
	    	return 2.49f;
	      break;
	    case 6:
	    	return 2.74f;
	      break;
	    case 7:
	    	return 2.99f;
	      break;
	    case 8:
	    	return 3.24f;
	      break;
	    case 9:
	    	return 3.49f;
	      break;
	    case 10:
	    	return 3.74f;
	      break;
	    case 11:
	    	return 3.99f;
	      break;
	    case 12:
	    	return 4.24f;
	      break;
	    case 13:
	    	return 4.49f;
	      break;
	    case 14:
	    	return 4.99f;
	      break;
	    case 15:
	    	return 5.0f;
	      break;
	    default:
	      return -1.0f;
	      break;
	  }
	}


HAL_StatusTypeDef AP33772S_i2c_read(AP33772S_t *dev, uint16_t rdLen, uint32_t timeout){
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(dev->hi2c, (AP33772S_ADDRESS), &dev->tgtAddr, 1, timeout);
    if (ret != HAL_OK) //Early return
    	return ret;

    ret = HAL_I2C_Master_Receive(dev->hi2c, (AP33772S_ADDRESS), dev->readBuf, rdLen, timeout);
    return ret;
}

HAL_StatusTypeDef AP33772S_i2c_write(AP33772S_t *dev, uint16_t wrLen, uint32_t timeout)
{
	dev->writeBuf[0] = dev->tgtAddr;
    return HAL_I2C_Master_Transmit(dev->hi2c, (AP33772S_ADDRESS), dev->writeBuf, wrLen, timeout);
}


