

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "Service_ECGMonitor.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */




/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// ECG Monitor service
CONST uint8 ECGMonitorServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECGMONITOR_SERV_UUID), HI_UINT16(ECGMONITOR_SERV_UUID)
};

// ECG Signal characteristic
CONST uint8 ECGSignalUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECGMONITOR_ECGSIGNAL_UUID), HI_UINT16(ECGMONITOR_ECGSIGNAL_UUID)
};

// 1mV Signal characteristic
CONST uint8 signal1mVUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECGMONITOR_1MVSIGNAL_UUID), HI_UINT16(ECGMONITOR_1MVSIGNAL_UUID)
};

// Sampling rate characteristic
CONST uint8 sampleRateUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECGMONITOR_SAMPLERATE_UUID), HI_UINT16(ECGMONITOR_SAMPLERATE_UUID)
};

// Lead Type characteristic
CONST uint8 leadTypeUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(ECGMONITOR_LEAD_TYPE_UUID), HI_UINT16(ECGMONITOR_LEAD_TYPE_UUID)
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
//��������Ӧ�ò�Ļص�����
static ECGServiceAppCB_t ECGServiceAppCB;

/*********************************************************************
 * Profile Attributes - variables
 */

// ECG Monitor Service attribute��Service��Attribute����
static CONST gattAttrType_t ECGService = { ATT_BT_UUID_SIZE, ECGMonitorServUUID };

// ECG Signal Characteristic
static uint8 ECGSignalProps = GATT_PROP_NOTIFY;
static int ECGSignal = 0;
static gattCharCfg_t ECGSignalConfig[GATT_MAX_NUM_CONN];  //ÿ�������϶�Ӧ��һ������ֵ

//1mV Signal Characteristic
static uint8 signal1mVProps = GATT_PROP_READ;
static int signal1mV = 0;

//sampling rate characteristic
static uint8 sampleRateProps = GATT_PROP_READ;
static int sampleRate = 250;      //Hz

// Lead Type Characteristic
static uint8 leadTypeProps = GATT_PROP_READ;
static uint8 leadType  = 0x00;  //Lead I



/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t ECGMonitorAttrTbl[] = 
{
  // 0. ECG Monitor Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&ECGService                      /* pValue */
  },
      
    // ECG Signal
    // 1. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &ECGSignalProps 
    },

    // 2. Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, ECGSignalUUID },
      0, 
      0, 
      (uint8*)&ECGSignal 
    },

    // 3. Characteristic Configuration 
    { 
      { ATT_BT_UUID_SIZE, clientCharCfgUUID },
      GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
      0, 
      (uint8 *)&ECGSignalConfig
    },
 
    // 1mV Signal   
    // 4. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &signal1mVProps 
    },

    // 5. Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, signal1mVUUID },
      GATT_PERMIT_READ, 
      0, 
      (uint8*)&signal1mV 
    },
    
    // sampling rate   
    // 6. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &sampleRateProps 
    },

    // 7. Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, sampleRateUUID },
      GATT_PERMIT_READ, 
      0, 
      (uint8*)&sampleRate 
    },    
    
    // lead type   
    // 8. Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &leadTypeProps 
    },

    // 9. Characteristic Value
    { 
      { ATT_BT_UUID_SIZE, leadTypeUUID },
      GATT_PERMIT_READ, 
      0, 
      (uint8*)&leadType 
    },        

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 ECGMonitor_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t ECGMonitor_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void ECGMonitor_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// ����Service��GATT�Ļص�����Ҫ��Ӧ�ò�Ļص������
CONST gattServiceCBs_t ECGMonitorCBs =
{
  ECGMonitor_ReadAttrCB,  // Read callback function pointer
  ECGMonitor_WriteAttrCB, // Write callback function pointer
  NULL                     // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
bStatus_t CMTechECGMonitor_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  // ָ��INVALID_CONNHANDLE����ָ���е�����
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, ECGSignalConfig );

  // Register with Link DB to receive link status change callback
  // �Ǽ�����״̬�ı�Ļص�����
  VOID linkDB_Register( ECGMonitor_HandleConnStatusCB );  
  
  if ( services & ECGMONITOR_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    // �Ǽ�GATT���Ա��Service��GATT�Ļص�����
    status = GATTServApp_RegisterService( ECGMonitorAttrTbl, 
                                          GATT_NUM_ATTRS( ECGMonitorAttrTbl ),
                                          &ECGMonitorCBs );
  }

  return ( status );
}

// �Ǽ�Ӧ�ò�ص�����
extern void CMTechECGMonitor_Register( ECGServiceAppCB_t pfnServiceCB )
{
  ECGServiceAppCB = pfnServiceCB;
}

// ��Ӧ�ò���ã����÷�������ֵ
bStatus_t CMTechECGMonitor_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {      
    case ECGMONITOR_ECGSIGNAL_CHAR_CFG:      
      // Need connection handle
      //thermometerTempConfig.value = *((uint8*)value);
      break;  
      
    case ECGMONITOR_1MVSIGNAL:
      if ( len == sizeof(int) ) 
      {
        VOID osal_memcpy( &signal1mV, value, sizeof(int) );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;  
 
    case ECGMONITOR_SAMPLERATE:
      if ( len == sizeof(int) ) 
      {
        VOID osal_memcpy( &sampleRate, value, sizeof(int) );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break; 
      
    case ECGMONITOR_LEAD_TYPE:
      if ( len == sizeof(uint8) ) 
      {
        leadType = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;        
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

// ��Ӧ�ò���ã���ȡ��������ֵ
bStatus_t CMTechECGMonitor_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {     
    case ECGMONITOR_ECGSIGNAL_CHAR_CFG:
        // Need connection handle
        //*((uint16*)value) = thermometerTempConfig.value;
        break;  
        
    case ECGMONITOR_1MVSIGNAL:
      *((int*)value) = signal1mV;
      break;          
      
    case ECGMONITOR_SAMPLERATE:
      *((int*)value) = sampleRate;
      break;              
        
    case ECGMONITOR_LEAD_TYPE:
      *((uint8*)value) = leadType;
      break;        
      
    default:
        ret = INVALIDPARAMETER;
        break;
  }
  
  return ( ret );
}

//����һ��ECG�ź�ֵ��Notification
bStatus_t CMTechECGMonitor_ECGSignalNotify( uint16 connHandle, attHandleValueNoti_t *pNoti )
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, ECGSignalConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    // Set the handle
    pNoti->handle = ECGMonitorAttrTbl[ECGSIGNAL_VALUE_POS].handle;
  
    // Send the Notification
    return GATT_Notification( connHandle, pNoti, FALSE );
  }

  return bleIncorrectMode;
}

// ��pAttr�ж�һ��Attribute��ֵ�ŵ�pValue��ȥ
static uint8 ECGMonitor_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  //blob��Binary Large Object, �����ƴ����ݶ���
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {      
      case ECGMONITOR_1MVSIGNAL_UUID:   
        *pLen = sizeof(int);
        VOID osal_memcpy( pValue, pAttr->pValue, sizeof(int) );
        break;

      case ECGMONITOR_SAMPLERATE_UUID:   
        *pLen = sizeof(int);
        VOID osal_memcpy( pValue, pAttr->pValue, sizeof(int) );
        break; 
        
      case ECGMONITOR_LEAD_TYPE_UUID:   
        *pLen = 1;
        *pValue = *((uint8*)pAttr->pValue);
        break;         
        
      default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  
  return ( status );
}

// дһ��Attribute����pValueд��pAttr��ȥ
static bStatus_t ECGMonitor_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                          uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;

  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  switch ( uuid )
  {
    case  GATT_CLIENT_CHAR_CFG_UUID:  //�ͻ����޸���������CCC
      //����GATT����
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY );  
      //����ɹ�����֪ͨӦ�ò�
      if ( status == SUCCESS )
      {
        uint16 value = BUILD_UINT16( pValue[0], pValue[1] );      
      
        //֪ͨӦ�ò�ECG Signal CCC�����Ѿ��޸���
        (*ECGServiceAppCB)( (value == GATT_CFG_NO_OPERATION) ? 
                                 ECGMONITOR_SIGNAL_NOTI_DISABLED :
                                 ECGMONITOR_SIGNAL_NOTI_ENABLED );    
      }
      break;
    
    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }
  
  return ( status );
}

// ����״̬�ı�ص�����
static void ECGMonitor_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, ECGSignalConfig );
    }
  }
}


/*********************************************************************
*********************************************************************/
