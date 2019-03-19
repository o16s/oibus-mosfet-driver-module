#ifndef __OI_UAVCAN_H
#define __OI_UAVCAN_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Application constants
 */
#define APP_VERSION_MAJOR                                           1
#define APP_VERSION_MINOR                                           0
#define APP_NODE_NAME                                               "ch.octanis.oibus.mosfet-driver-module"


/*
 * Some useful constants defined by the UAVCAN specification.
 * Data type signature values can be easily obtained with the script show_data_type_info.py
 */
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID                      1
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE               0x0b2a812620a11d40
#define UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC         400000UL
#define UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC         600000UL

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      ((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1

#define UAVCAN_PROTOCOL_PARAM_GETSET_MAX_SIZE 			    ((2967+ 7) / 8)
#define UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE			    0xa7b622f939d1a4d5
#define UAVCAN_PROTOCOL_PARAM_GETSET_ID				    11



#define UNIQUE_ID_LENGTH_BYTES 16

#define CANARD_SPIN_PERIOD   1000

#define FIRMWARE_GIT_HASH					    0
#define UAVCAN_NODE_ID						    111

#define ARRAY_SIZE(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))



void oi_uavcan_sendCanard(void);
void oi_uavcan_receiveCanard(void);
void oi_uavcan_spinCanard(uint32_t time);
void oi_uavcan_init(void);



#ifdef __cplusplus
}
#endif
#endif  // __OI_UAVCAN_H
