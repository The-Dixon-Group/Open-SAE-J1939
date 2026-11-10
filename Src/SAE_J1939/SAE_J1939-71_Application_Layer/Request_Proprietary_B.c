/*
 * Request_Proprietary_B.c
 *
 *  Created on: 3 November 2024
 *      Author: Guillermo Rodríguez
 */

#include "Application_Layer.h"

 /* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"
#include "../../Hardware/Hardware.h"
#include "comms.h"
#include "definitions.h"
/*
 * Request Proprietary B to another ECU
 * PGN: 0x00FF00 <-> 0x00FFFF
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_Proprietary_B(J1939* j1939, uint8_t DA, uint32_t PGN) {
	return SAE_J1939_Send_Request(j1939, DA, PGN);
}

 // IMPORTANT: This function can return NULL if given PGN is not expected in this ECU
struct Proprietary_B * Get_Proprietary_B_By_PGN(struct Proprietary * proprietary, uint32_t PGN)
{
	for (int i = 0; i < MAX_PROPRIETARY_B_PGNS; ++i)
	{
		if (proprietary->proprietary_B[i].pgn == PGN)
		{
			return &proprietary->proprietary_B[i];
		}
	}
	return NULL;
}

/*
 * Response the request Proprietary B about this ECU
 * PGN: 0x00FF00 <-> 0x00FFFF
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_Proprietary_B(J1939* j1939, uint8_t DA, uint32_t PGN, bool * is_supported) {
	struct Proprietary_B * proprietary_B = Get_Proprietary_B_By_PGN(&j1939->this_proprietary, PGN);
	if (proprietary_B == NULL) /* This PGN is not implemented on this ECU, can't send reply */
	{
		*is_supported = false;
		return STATUS_SEND_ERROR;
	}
	*is_supported = true;

	/* Find the length of the array fields */
	uint16_t length_of_each_field = proprietary_B->total_bytes;
	if (length_of_each_field < 9) {
		/* If each field have the length 8 or less, then we can send Proprietary B as it was a normal message */
		uint32_t ID = (PGN << 8) | j1939->information_this_ECU.this_ECU_address;
		uint8_t data[8];
		memcpy(data, proprietary_B->data, length_of_each_field);
		return CAN_Send_Message(ID, data);
	}
	else {
		/* Multiple messages - Load data */
		j1939->this_ecu_tp_cm.total_message_size_being_transmitted = length_of_each_field;
		memcpy(j1939->this_ecu_tp_dt.data, proprietary_B->data, length_of_each_field);
		
		/* Send TP CM */
		j1939->this_ecu_tp_cm.number_of_packages_being_transmitted = j1939->this_ecu_tp_cm.total_message_size_being_transmitted % 8 > 0 ? j1939->this_ecu_tp_cm.total_message_size_being_transmitted / 8 + 1 : j1939->this_ecu_tp_cm.total_message_size_being_transmitted / 8; /* Rounding up */
		j1939->this_ecu_tp_cm.PGN_of_the_packeted_message = PGN;
		j1939->this_ecu_tp_cm.control_byte = DA == 0xFF ? CONTROL_BYTE_TP_CM_BAM : CONTROL_BYTE_TP_CM_RTS; /* If broadcast, then use BAM control byte */
		ENUM_J1939_STATUS_CODES status = SAE_J1939_Send_Transport_Protocol_Connection_Management(j1939, DA);
		if (status != STATUS_SEND_OK) {
			return status;
		}

		/* Check if we are going to send it directly (BAM) */
		if (j1939->this_ecu_tp_cm.control_byte == CONTROL_BYTE_TP_CM_BAM) {
			j1939->from_other_ecu_tp_cm.control_byte = j1939->this_ecu_tp_cm.control_byte;
			return SAE_J1939_Send_Transport_Protocol_Data_Transfer(j1939, DA);
		}
		return status;
	}
}

/*
 * Store the Proprietary B about other ECU
 * PGN: 0x00FF00 <-> 0x00FFFF
 */
void SAE_J1939_Read_Response_Request_Proprietary_B(J1939* j1939, uint8_t SA, uint32_t PGN, uint8_t data[]) {
	struct Proprietary_B * proprietary_B = Get_Proprietary_B_By_PGN(&j1939->from_other_ecu_proprietary, PGN);

	if (proprietary_B == NULL) /* Proprietary B is not expected, don't fill the data anywhere */
	{
		return;
	}

	uint16_t total_bytes = proprietary_B->total_bytes;
	memcpy(proprietary_B->data, data, total_bytes);
	proprietary_B->from_ecu_address = SA;
    
    /*
     * Handle specific Proprietary B PGNs here.
     */
    switch (PGN)
    {
#if PRODUCT == CHECKMATEII
        
#elif PRODUCT == DIGITAL_PRODUCT_INDICATOR
        case SET_PRODUCT_INDEX_PGN:
        {
            /* The first data byte identifies the compartment the broadcast
            * message is intended for. Only process the message when it matches
            * this ECU's current compartment position. The second data byte
            * contains the product index to be applied to that display.
            */
            uint8_t broadcastcompartmentPosition = data[0];
            uint8_t productIndex = data[1];
            
            
            if (broadcastcompartmentPosition == compartmentPosition){
                SetProdIndex(productIndex);
            }
            break;
        }
        
        case REQUEST_PRODUCT_LIST_PGN:
        {
            uint8_t broadcastcompartmentPosition = data[0];
            if (broadcastcompartmentPosition == compartmentPosition){
                //Send Over Product List if Compartment Matches
                SAE_J1939_Send_Product_List(j1939,J1939_BROADCAST_ADDRESS);
            }
            break;
        }
#elif PRODUCT == CHECKMATEII_TESTER
        case TEST_FIRMWARE_PGN:
        {
            /* The first data byte identifies the UUT command the broadcast
            * message is intended for.
            */
            uint8_t uut_command = data[0];
            float bank_voltage = 0;
            switch(uut_command){
                case CMD_00:
                    // Handle command 0x00 (UUT Reset)
                    SAE_J1939_Send_UUT_Response(j1939,ACK_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    NVIC_SystemReset();
                    break;
                
                case CMD_01:
                    // Handle command 0x01 (non-is relay de-energize)
                    DeEnergizeNONISRelay();
                    SAE_J1939_Send_UUT_Response(j1939,ACK_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    break;

                case CMD_02:
                    // Handle command 0x02 (non-is relay energize)
                    EnergizeNONISRelay();
                    SAE_J1939_Send_UUT_Response(j1939,ACK_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    break;

                case CMD_03:
                    // Handle command 0x03 (Read Aux input 1)
                    if(GetAuxStatus(AUX1_INPUT)){
                        SAE_J1939_Send_UUT_Response(j1939,HIGH_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);    
                    }
                    else{
                        SAE_J1939_Send_UUT_Response(j1939,LOW_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    }
                    break;
                    
                case CMD_04:
                    // Handle command 0x04 (Read Aux input 2)
                    if(GetAuxStatus(AUX2_INPUT)){
                        SAE_J1939_Send_UUT_Response(j1939,HIGH_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);    
                    }
                    else{
                        SAE_J1939_Send_UUT_Response(j1939,LOW_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    }
                    break;
                case CMD_05:
                    // Handle command 0x05 (Read Aux input 3)
                    if(GetAuxStatus(AUX3_INPUT)){
                        SAE_J1939_Send_UUT_Response(j1939,HIGH_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);    
                    }
                    else{
                        SAE_J1939_Send_UUT_Response(j1939,LOW_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    }
                    break;
                
                case CMD_06:
                    //Handle command 6 (Read retain reset input)
                    if(GetRetainInputStatus()){
                       SAE_J1939_Send_UUT_Response(j1939,HIGH_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);  
                    }
                    else{
                       SAE_J1939_Send_UUT_Response(j1939,LOW_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    }
                    break;
                    
                case CMD_07:
                {
                    // BANK1 channel 1 -> LED 4
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][0].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(4, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(4, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }

                case CMD_08:
                {
                    // BANK1 channel 2 -> LED 5
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][1].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(5, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(5, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }

                case CMD_09:
                {
                    // BANK1 channel 3 -> LED 6
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][2].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(6, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(6, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }

                case CMD_0A:
                {
                    // BANK1 channel 4 -> LED 7
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][3].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(7, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(7, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }

                case CMD_0B:
                {
                    // BANK1 channel 5 -> LED 8 (Low Gain))
                    
                    //Set Gain to Low
                    Set5WDiagGain(LOW_GAIN);
                    
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][4].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(8, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(8, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }

                case CMD_0C:
                {
                    // BANK1 channel 5 -> LED 8 (High Gain))
                    
                    //Set Gain to HIGH
                    Set5WDiagGain(HIGH_GAIN);
                    
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][4].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(8, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(8, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }

                case CMD_0D:
                {
                    // BANK1 channel 6 -> LED 9
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][5].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(9, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(9, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }

                case CMD_0E:
                {
                    // BANK1 channel 7 -> LED 10
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][6].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(10, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(10, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }

                case CMD_0F:
                {
                    // BANK1 channel 8 -> LED 11
                    GetSensorBankStatus(BANK1);

                    bank_voltage =
                        ((float)sensorData.CH_M[BANK1][7].MAX_VOLTAGE / 255.0f)
                        * 3.30f / 0.2740f;

                    if (SENSOR_VOLTAGE_MIN <= bank_voltage &&
                        SENSOR_VOLTAGE_MAX >= bank_voltage)
                    {
                        __NOP();
                        SetLEDColor(11, (COLOR_t){DEFAULT_BRIGHTNESS, 0, 0}, 0);
                    }
                    else
                    {
                        __NOP();
                        SetLEDColor(11, (COLOR_t){0, DEFAULT_BRIGHTNESS, 0}, 0);
                    }

                    break;
                }
                
                case CMD_10:
                    // Handle command 0x10 (set optic out)
                    DAC_DataWrite(Voltage2DACCounts(12.0));
                    
                    //Send ACK
                    SAE_J1939_Send_UUT_Response(j1939,ACK_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    
                    break;
                    
                 case CMD_11:
                    // Handle command 0x11 (clear optic out)
                    DAC_DataWrite(Voltage2DACCounts(12.0));
                    
                    //Send ACK
                    SAE_J1939_Send_UUT_Response(j1939,ACK_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    
                    break;      
                    
                case CMD_2E:
                    // Handle command 0x2E (Turn All LEDs Red)
                    BankClearLED((uint32_t)0xFFFF<<4);
                    BankSetLEDRed((uint32_t)0xFFFF<<4);
                    SAE_J1939_Send_UUT_Response(j1939,ACK_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    break;     
                case CMD_2F:
                    // Handle command 0x2F (Turn All LEDs Green)
                    BankClearLED((uint32_t)0xFFFF<<4);
                    BankSetLEDGreen((uint32_t)0xFFFF<<4);
                    SAE_J1939_Send_UUT_Response(j1939,ACK_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    
                    break;       
                case CMD_30:
                    // Handle command 0x30 (Turn All LEDs Blue)
                    BankClearLED((uint32_t)0xFFFF<<4);
                    BankSetLEDBlue((uint32_t)0xFFFF<<4);
                    SAE_J1939_Send_UUT_Response(j1939,ACK_RESPONSE,SYSTEM_STATUS_DEFAULT_PRIORITY);
                    break;

                default:
                    // Unknown or unsupported command
                    break;
                }
        }
#endif
        /*
         * Add more PGNs here as needed.
         *
         * case 0xFF10:
         * {
         *     ...
         *     break;
         * }
         */

        default:
            break;
    }
    }
