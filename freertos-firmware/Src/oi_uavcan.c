#include "oi_uavcan.h"
#include <canard.h>
#include <canard_stm32.h>
#include <cmsis_os.h>
#include "stm32f3xx_hal.h"
#include "gpio.h"

#include <string.h>
#include <stdint.h>


/**********
 private variables
 **********/
static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode   = UAVCAN_NODE_MODE_INITIALIZATION;

static CanardInstance g_canard;             // The library instance
static uint8_t g_canard_memory_pool[1024];  // Arena for memory allocation, used by the library
static uint32_t g_uptime = 0;

static param_t parameters[] =
{
	    {"drivers", 0, 0b0000000000, 0b1111111111, 0b0000000000}
};



/**********
 private functions
 **********/

//helpers
static void readUniqueID(uint8_t* out_uid)
{
   for (uint8_t i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++)
   {
       out_uid[i] = i;
   }
}


static void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    const uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
    const uint8_t node_mode   = UAVCAN_NODE_MODE_OPERATIONAL;
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
    const uint32_t uptime_sec = g_uptime / 1000;
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}

static uint16_t makeNodeInfoMessage(uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE])
{
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    makeNodeStatusMessage(buffer);

    buffer[7] = APP_VERSION_MAJOR;
    buffer[8] = APP_VERSION_MINOR;
    buffer[9] = 1;  // Optional field flags, VCS commit is set
    const uint32_t git_hash = FIRMWARE_GIT_HASH;
    canardEncodeScalar(buffer, 80, 32, &git_hash);

    readUniqueID(&buffer[24]);
    const size_t name_len = strlen(APP_NODE_NAME);
    memcpy(&buffer[41], APP_NODE_NAME, name_len);
    return 41 + name_len ;
}

static void getNodeInfoHandleCanard(CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
    memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);
    const uint16_t len = makeNodeInfoMessage(buffer);
    int result = canardRequestOrRespond(&g_canard,
                                        transfer->source_node_id,
                                        UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
                                        UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
                                        &transfer->transfer_id,
                                        transfer->priority,
                                        CanardResponse,
                                        &buffer[0],
                                        (uint16_t)len);
    if (result < 0)
    {
        // TODO: handle the error
    }
}


static uint16_t encodeParamCanard(param_t * p, uint8_t * buffer)
{
    uint8_t n     = 0;
    int offset    = 0;
    uint8_t tag   = 1;

    if (p == NULL)
    {
        tag = 0;
        canardEncodeScalar(buffer, offset, 5, &n);
        offset += 5;
        canardEncodeScalar(buffer, offset,3, &tag);
        offset += 3;

        canardEncodeScalar(buffer, offset, 6, &n);
        offset += 6;
        canardEncodeScalar(buffer, offset,2, &tag);
        offset += 2;

        canardEncodeScalar(buffer, offset, 6, &n);
        offset += 6;
        canardEncodeScalar(buffer, offset, 2, &tag);
        offset += 2;
        buffer[offset / 8] = 0;
        return ( offset / 8 + 1 );
    }

    canardEncodeScalar(buffer, offset, 5,&n);
    offset += 5;
    canardEncodeScalar(buffer, offset, 3, &tag);
    offset += 3;
    canardEncodeScalar(buffer, offset, 64, &p->val);
    offset += 64;

    canardEncodeScalar(buffer, offset, 5, &n);
    offset += 5;
    canardEncodeScalar(buffer, offset, 3, &tag);
    offset += 3;
    canardEncodeScalar(buffer, offset, 64, &p->defval);
    offset += 64;

    canardEncodeScalar(buffer, offset, 6, &n);
    offset += 6;
    canardEncodeScalar(buffer, offset, 2, &tag);
    offset += 2;
    canardEncodeScalar(buffer, offset, 64, &p->max);
    offset += 64;

    canardEncodeScalar(buffer, offset, 6, &n);
    offset += 6;
    canardEncodeScalar(buffer, offset,2,&tag);
    offset += 2;
    canardEncodeScalar(buffer, offset,64,&p->min);
    offset += 64;

    memcpy(&buffer[offset / 8], p->name, strlen((char const*)p->name));
    /* See important note below */
    return  (offset/8 + strlen((char const*)p->name));

}

static void getsetHandleCanard(CanardRxTransfer* transfer){

    uint16_t index = 0xFFFF;
    uint8_t tag    = 0;
    int offset     = 0;
    int64_t val    = 0;

    canardDecodeScalar(transfer, offset,  13, false, &index);
    offset += 13;
    canardDecodeScalar(transfer, offset, 3, false, &tag);
    offset += 3;

    if (tag == 1)
    {
        canardDecodeScalar(transfer, offset, 64, false, &val);
        offset += 64;
    }

    uint16_t n = transfer->payload_len - offset / 8 ;
    uint8_t name[16] = "";
    for (int i = 0; i < n; i++)
    {
        canardDecodeScalar(transfer, offset, 8, false, &name[i]);
        offset += 8;
    }

    param_t* p = NULL;

    if (strlen((char const*)name))
    {
        p = oi_uavcan_getParamByName(name);
    }
    else
    {
        p = oi_uavcan_getParamByIndex(index);
    }

    if ((p)&&(tag == 1))
    {
        p->val = val;
    }

    uint8_t  buffer[64] = "";
    uint16_t len = encodeParamCanard(p, buffer);
    int result = canardRequestOrRespond(&g_canard,
                                        transfer->source_node_id,
                                        UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                                        UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                                        &transfer->transfer_id,
                                        transfer->priority,
                                        CanardResponse,
                                        &buffer[0],
                                        (uint16_t)len);





}


//main functions
static bool shouldAcceptTransfer(const CanardInstance* ins,
                         uint64_t* out_data_type_signature,
                         uint16_t data_type_id,
                         CanardTransferType transfer_type,
                         uint8_t source_node_id)
{
   if ((transfer_type == CanardTransferTypeRequest) &&
       (data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
   {
       *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
       return true;
   }

   if ((transfer_type == CanardTransferTypeRequest) &&
       (data_type_id ==  UAVCAN_PROTOCOL_PARAM_GETSET_ID))
   {
       *out_data_type_signature =  UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
       return true;
   }

   return false;
}


static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
   if ((transfer->transfer_type == CanardTransferTypeRequest) &&
   (transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
   {
	    getNodeInfoHandleCanard(transfer);
   }



   if ((transfer->transfer_type == CanardTransferTypeRequest) &&
   (transfer->data_type_id == UAVCAN_PROTOCOL_PARAM_GETSET_ID))
   {
      getsetHandleCanard(transfer);
   }


}






/**********
 public
 **********/

param_t* oi_uavcan_getParamByIndex(uint16_t index)
{
   if (index >= ARRAY_SIZE(parameters))
   {
       return NULL;
   }
   return &parameters[index];
}

param_t* oi_uavcan_getParamByName(uint8_t * name)
{
   for (uint16_t i = 0; i < ARRAY_SIZE(parameters); i++)
   {
       if (strncmp(name, parameters[i].name, strlen(parameters[i].name)) == 0)
       {
             return &parameters[i];
       }
   }
   return NULL;
}

void oi_uavcan_init(void)
{
  //turn on TCAN330
  HAL_GPIO_WritePin(GPIOB, CAN_S_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, CAN_SHTD_Pin, GPIO_PIN_RESET);


  CanardSTM32CANTimings timings;
  int result = canardSTM32ComputeCANTimings(HAL_RCC_GetPCLK1Freq(), CANSPEED, &timings);
  if (result)
  {
      //__ASM volatile("BKPT #01");
  }

  canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);
  result = canardSTM32Init(&timings, CanardSTM32IfaceModeNormal);

  canardInit(&g_canard,                         // Uninitialized library instance
             g_canard_memory_pool,              // Raw memory chunk used for dynamic allocation
             sizeof(g_canard_memory_pool),      // Size of the above, in bytes
             onTransferReceived,                // Callback, see CanardOnTransferReception
             shouldAcceptTransfer,              // Callback, see CanardShouldAcceptTransfer
             NULL);

  canardSetLocalNodeID(&g_canard, UAVCAN_NODE_ID);
}

void oi_uavcan_spinCanard(uint32_t time)
{
  g_uptime = time;

	static uint32_t spin_time = 0;
  if (time < spin_time + CANARD_SPIN_PERIOD) {
	   return;
  }else{

      spin_time = time;

      uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
      static uint8_t transfer_id = 0;                          // This variable MUST BE STATIC; refer to the libcanard documentation for the background

      makeNodeStatusMessage(buffer);
      canardBroadcast(&g_canard,
                        UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                        UAVCAN_NODE_STATUS_DATA_TYPE_ID,
                        &transfer_id,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        UAVCAN_NODE_STATUS_MESSAGE_SIZE);

  }  // rate limiting

}


void oi_uavcan_sendCanard(void)
{
   const CanardCANFrame* txf = canardPeekTxQueue(&g_canard);
   while(txf)
   {
       const int tx_res = canardSTM32Transmit(txf);
       if(tx_res > 0){
           canardPopTxQueue(&g_canard);
       }

       txf = canardPeekTxQueue(&g_canard);
   }
}


void oi_uavcan_receiveCanard(void)
{
   CanardCANFrame rx_frame;
   int res = canardSTM32Receive(&rx_frame);
   if(res){
     canardHandleRxFrame(&g_canard, &rx_frame, g_uptime * 1000);
   }
}


void oi_uavcan_publish_keyVal(char * key[3], float val){

  uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MESSAGE_SIZE];
  memset(buffer,0x00,UAVCAN_PROTOCOL_DEBUG_KEYVALUE_MESSAGE_SIZE);
  static uint8_t transfer_id = 0;                          // This variable MUST BE STATIC; refer to the libcanard documentation for the background
  canardEncodeScalar(buffer, 0, 32, &val);
  memcpy(&buffer[4], key, 3);
  canardBroadcast(&g_canard,
                UAVCAN_PROTOCOL_DEBUG_KEYVALUE_SIGNATURE,
                UAVCAN_PROTOCOL_DEBUG_KEYVALUE_ID,
                &transfer_id,
                CANARD_TRANSFER_PRIORITY_LOW,
                &buffer[0],
                7);
}
