/*
 * DM3.c
 *
 *  Created on: 14 juli 2021
 *      Author: Daniel Mårtensson
 */

#include "Diagnostics_Layer.h"

/* Layers */
#include "../SAE_J1939-21_Transport_Layer/Transport_Layer.h"

#ifdef J1939_MASTER
/*
 * Request DM3 from another ECU
 * PGN: 0x00FECC (65228)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Send_Request_DM3(J1939 *j1939, uint8_t DA) {
	return SAE_J1939_Send_Request(j1939, DA, PGN_DM3);
}
#endif

/*
 * Response the request of DM3 (clear DM2, which is previously active errors from DM1 codes) to other ECU about this ECU
 * PGN: 0x00FECC (65228)
 */
ENUM_J1939_STATUS_CODES SAE_J1939_Response_Request_DM3(J1939* j1939, uint8_t DA) {
	
    for(uint8_t i = 0; i < MAX_DM_FIELD; i++)
    {
        j1939->this_dm.DTC[i].active = 0;
        j1939->this_dm.DTC[i].occurance_count = 0;
    }
	return SAE_J1939_Response_Request_DM2(j1939, DA);					/* Send DM2 codes to the ECU who send the request */
}
