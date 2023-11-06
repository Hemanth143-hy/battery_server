/* Bluetooth  info */
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_uuid.h"
#include "wiced_gki.h"
#include "wiced_bt_app_common.h"
#include "wiced_result.h"
#include "wiced_hal_platform.h"
#include "wiced_memory.h"
#include "wiced_transport.h"
#include "gatt_server_db.h"
#include "wiced_hal_nvram.h"
#include "wiced_timer.h"
#include "wiced_hal_puart.h"
#include "string.h"
#include "wiced_bt_stack.h"

#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif

/******************************************************************************
 *                             External Definitions
 ******************************************************************************/
extern const wiced_bt_cfg_settings_t wiced_app_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[];


/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define BATTERY_SERVICE_GATTS_MAX_CONN                1
#define MAX_BATTERY_LEVEL                             100
#define BATTERY_SERVICE_TIMER_DURATION_IN_MS          3000
#define MAX_BATTERY_POWER                             100
#define DISCHARGE_STATE                               00
#define CHARGE_STATE                                  01
#define LEVEL_POWER                                   0x03
#define BATTERY_SERVICE_VS_ID                     WICED_NVRAM_VSID_START
#define BATTERY_SERVICE_LOCAL_KEYS_VS_ID          ( BATTERY_SERVICE_VS_ID + 1 )
#define BATTERY_SERVICE_PAIRED_KEYS_VS_ID         BATTERY_SERVICE_LOCAL_KEYS_VS_ID + 1

/******************************************************************************
 *                                Structures
 ******************************************************************************/
typedef struct
{
    BD_ADDR   remote_addr;              // remote peer device address
    uint16_t  conn_id;                  // connection ID referenced by the stack
    uint8_t   flag_notify_sent;         // flag to check whether notifiation was sent
    uint8_t   battery_level;                        /* Battery level */

}battery_service_app_state_t;

#pragma pack(1)


typedef struct {
        uint8_t battery_level;       // Assign a value between 0 and 100
        uint8_t battery_power_info;  // Assign based on your application's requirement
        uint8_t discharging_state;   // 0 for not discharging, 1 for discharging
        uint8_t charging_state;      // 0 for not charging, 1 for charging
        uint8_t level;              // connection ID referenced by the stack

} battery_level_state_t;

#pragma pack(1)

typedef struct {
                   // Assign a value between 0 and 100
            uint8_t battery_power_info;  // Assign based on your application's requirement
            uint8_t discharging_state;   // 0 for not discharging, 1 for discharging
            uint8_t charging_state;      // 0 for not charging, 1 for charging
            uint8_t level;              // connection ID referenced by the stack

} battery_power_state_t;

#pragma pack(1)

typedef struct {
    uint8_t temperature;    // Battery temperature in degrees Celsius
    uint16_t voltage;       // Battery voltage in millivolts
    uint16_t current;       // Battery current in milliamps
    uint8_t state_of_charge; // Battery state of charge in percentage
    // ... other relevant fields for battery health information
} battery_health_info_t;

#pragma pack(1)

/* Host information saved in  NVRAM */
typedef PACKED struct
{
    BD_ADDR  bdaddr;                               /* BD address of the bonded host */
    uint16_t characteristic_client_configuration;  /* Current value of the client configuration descriptor */
} host_info_t;

#pragma pack()

typedef struct
{
    uint16_t handle;
    uint16_t attr_len;
    void     *p_attr;
} attribute_t;

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/
/* transport configuration */
const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    { WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_DEFAULT_BAUD },
    { 0, 0},
    NULL,
    NULL,
    NULL
};

/* Holds global state of the App */
battery_service_app_state_t battery_service_app_state;
//size_t battery_level_state_size = sizeof(battery_level_state_t);


battery_power_state_t power_state;

battery_health_info_t health_info;
uint16_t global_conn_id = 0;

wiced_timer_t battery_notification_timer;


battery_level_state_t battery_state;






/* Holds the host info saved in the NVRAM */
host_info_t battery_service_hostinfo;

uint8_t battery_service_device_name[]          = "Battery_info";                     //GAP Service characteristic Device Name
uint8_t battery_service_appearance_name[2]     = { BIT16_TO_8(APPEARANCE_GENERIC_TAG) };

/* Attribute list of the battery service */
attribute_t gatt_user_attributes[] =
{
    { HANDLE_BS_GAP_SERVICE_CHAR_DEV_NAME_VAL,             sizeof( battery_service_device_name ),   battery_service_device_name },
    { HANDLE_BS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,       sizeof(battery_service_appearance_name), battery_service_appearance_name },
    { HANDLE_BATTERY_SERVICE_CHAR_LEVEL_VAL ,              1,                                       (void *)&battery_service_app_state.battery_level },
    { HANDLE_BATERY_SERVICE_BATTERY_LEVEL_CHAR_CFG_DESC,   2,                                       (void *)&battery_service_hostinfo.characteristic_client_configuration },
    { HANDLE_BATTERY_SERVICE_CHAR_LEVEL_STATE_VALUE ,       5,                                     (void *)&battery_state},
    { HANDLE_BATERY_SERVICE_BATTERY_LEVEL_STATE_CHAR_CFG_DESC,   2,                                       (void *)&battery_service_hostinfo.characteristic_client_configuration },
    { HANDLE_BATTERY_SERVICE_CHAR_POWER_STATE_VALUE ,       4,                                     (void *)&power_state },
    { HANDLE_BATERY_SERVICE_BATTERY_POWER_STATE_CHAR_CFG_DESC,   2,                                       (void *)&battery_service_hostinfo.characteristic_client_configuration },
    { HANDLE_BATTERY_SERVICE_CHAR_HEALTH_INFORMATION_VALUE ,       3,                                     (void *)&health_info },
    { HANDLE_BATERY_SERVICE_BATTERY_HEALTH_INFORMATION_CHAR_CFG_DESC,   2,                                       (void *)&battery_service_hostinfo.characteristic_client_configuration },
};


wiced_timer_t battery_service_timer;

/*****************************************************************************
 *                           Function Prototypes
 *****************************************************************************/
static wiced_result_t             battery_service_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                       battery_service_application_init();
static wiced_bt_gatt_status_t     battery_service_gatt_cback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );
static void                       battery_service_load_keys_for_address_resolution( void );
static void                       battery_service_set_advertisement_data();
static wiced_bt_gatt_status_t     battery_service_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t     battery_service_gatts_connection_up(wiced_bt_gatt_connection_status_t *p_status);
static wiced_bt_gatt_status_t     battery_service_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status );
static wiced_bt_gatt_status_t     battery_service_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data );
static wiced_bt_gatt_status_t     battery_service_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );
static wiced_bt_gatt_status_t     battery_service_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
static void                       battery_service_smp_bond_result( uint8_t result );
static void                       battery_service_encryption_changed( wiced_result_t result, uint8_t* bd_addr );
static void                       battery_service_send_message();
static void                       battery_service_timer_expiry_handler(  uint32_t param );
static void                       battery_service_notify_state_level();
static void                       battery_service_power_state();
static void                       battery_service_health();

void store_connection_id(uint16_t conn_id);


#ifdef ENABLE_HCI_TRACE
static void                       battery_service_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
#endif

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
#ifndef CYW20735B0
APPLICATION_START( )
#else
void application_start( void )
#endif
{
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    //wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif
    WICED_BT_TRACE( " Battery Service Server Start\n" );

    // Register call back and configuration with stack
    wiced_bt_stack_init( battery_service_management_cback ,
                    &wiced_app_cfg_settings, wiced_app_cfg_buf_pools );

    memset(&battery_state, 0, sizeof(battery_state));
   // memset(&power_state, 0, sizeof(power_state));

    battery_service_app_state.battery_level = MAX_BATTERY_LEVEL;
    battery_service_hostinfo.characteristic_client_configuration = 0;

    battery_state.battery_level = MAX_BATTERY_LEVEL;
    battery_state.battery_power_info =  0x0F;
    battery_state.discharging_state = 0x01;
    battery_state.charging_state   = 0x01;
    battery_state.level = MAX_BATTERY_LEVEL;
    battery_service_hostinfo.characteristic_client_configuration = 0;


       power_state.battery_power_info =  0x0F;
       power_state.discharging_state = 0x01;
       power_state.charging_state   = 0x01;
       power_state.level = MAX_BATTERY_LEVEL;
    battery_service_hostinfo.characteristic_client_configuration = 0;

    health_info.temperature = 25;
    health_info.voltage     = 3600;
    health_info.current     = 500;
    health_info.state_of_charge = MAX_BATTERY_LEVEL;
    battery_service_hostinfo.characteristic_client_configuration = 0;





}

static void battery_service_application_init()
{
    wiced_bt_gatt_status_t gatt_status;
#ifndef CYW20735B1
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif
    /* Register for gatt event notifications */
    gatt_status = wiced_bt_gatt_register(&battery_service_gatt_cback );

    WICED_BT_TRACE( "wiced_bt_gatt_register: %d\n", gatt_status );
    /* Initialize GATT database */
    gatt_status = wiced_bt_gatt_db_init((uint8_t *)gatt_db, gatt_db_size );

    WICED_BT_TRACE("wiced_bt_gatt_db_init %d\n", gatt_status);

#ifdef ENABLE_HCI_TRACE
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( battery_service_hci_trace_cback );
#endif

#ifdef CYW20706A2
    /* Enable privacy to advertise with RPA */
    wiced_bt_ble_enable_privacy ( WICED_TRUE );
#endif
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);


    /* Load the address resolution DB with the keys stored in the NVRAM */
    battery_service_load_keys_for_address_resolution();

    battery_service_set_advertisement_data();

    /* start LE advertising */
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    WICED_BT_TRACE("Waiting for Battery Service to connect...\n");

    wiced_init_timer( &battery_service_timer, battery_service_timer_expiry_handler, 0,
            WICED_MILLI_SECONDS_TIMER );
}

static void battery_service_set_advertisement_data()
{
    wiced_result_t              result;
    wiced_bt_ble_advert_elem_t  adv_elem[3];
    uint8_t ble_advertisement_flag_value        = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem                            = 0;
    uint16_t battery_service_uuid    = UUID_SERVICE_BATTERY;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len          = 1;
    adv_elem[num_elem].p_data       = &ble_advertisement_flag_value;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    adv_elem[num_elem].len          = LEN_UUID_16;
    adv_elem[num_elem].p_data       = (uint8_t *)&battery_service_uuid;
    num_elem ++;

    adv_elem[num_elem].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len          = strlen((const char *)wiced_app_cfg_settings.device_name);
    adv_elem[num_elem].p_data       = (uint8_t *)wiced_app_cfg_settings.device_name;

    num_elem++;

    result = wiced_bt_ble_set_raw_advertisement_data(num_elem,adv_elem);

    WICED_BT_TRACE("wiced_bt_ble_set_advertisement_data %d\n",result);
}

static void battery_service_timer_expiry_handler(  uint32_t param )
{
    if( battery_service_app_state.battery_level <= 0 )
    {
        battery_service_app_state.battery_level = MAX_BATTERY_POWER;
    }

    battery_service_send_message();
    battery_service_notify_state_level();
    battery_service_power_state();
    battery_service_health();
    wiced_start_timer( &battery_service_timer, BATTERY_SERVICE_TIMER_DURATION_IN_MS );
}

/* GATT event handler */
static wiced_bt_gatt_status_t battery_service_gatt_cback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    switch ( event )
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = battery_service_gatts_conn_status_cb( &p_data->connection_status );
            uint16_t conn_id = p_data->connection_status.conn_id;
            store_connection_id(conn_id);
            WICED_BT_TRACE("Connection ID: %d\n", conn_id);

            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = battery_service_gatts_req_cb( &p_data->attribute_request );
            break;

        default:
            break;
    }
    return result;
}

static wiced_bt_gatt_status_t battery_service_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if ( p_status->connected )
    {
        return battery_service_gatts_connection_up( p_status );

    }

    return battery_service_gatts_connection_down( p_status );
}

/* This function is invoked when connection is established */
static wiced_bt_gatt_status_t battery_service_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;
    uint8_t        bytes_written = 0;

    WICED_BT_TRACE( "battery_service_conn_up %B id:%d\n:", p_status->bd_addr, p_status->conn_id);

    /* Update the connection handler.  Save address of the connected device. */
    battery_service_app_state.conn_id = p_status->conn_id;
    memcpy(battery_service_app_state.remote_addr, p_status->bd_addr, sizeof(BD_ADDR));

    /* Stop advertising */
    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );

    WICED_BT_TRACE( "Stopping Advertisements%d\n", result );

    /* Updating the bd address in the  host info in NVRAM  */
    memcpy( battery_service_hostinfo.bdaddr, p_status->bd_addr, sizeof( BD_ADDR ) );

    /* Save the  host info in NVRAM  */
    bytes_written = wiced_hal_write_nvram( BATTERY_SERVICE_VS_ID, sizeof(battery_service_hostinfo), (uint8_t*)&battery_service_hostinfo, &result );
    WICED_BT_TRACE("NVRAM write %d\n", bytes_written);

    return WICED_BT_GATT_SUCCESS;
}

static wiced_bt_gatt_status_t battery_service_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    wiced_result_t result;

    WICED_BT_TRACE( "connection_down %B conn_id:%d reason:%d\n", battery_service_app_state.remote_addr, p_status->conn_id, p_status->reason );

    /* Resetting the device info */
      battery_service_app_state.conn_id = 0;
    wiced_stop_timer( &battery_service_timer );

    result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
    WICED_BT_TRACE( "wiced_bt_start_advertisements %d\n", result );

    return WICED_BT_SUCCESS;
}

static wiced_result_t battery_service_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    uint8_t                          *p_keys;

    WICED_BT_TRACE("battery_service_management_cback: %x\n", event );

    switch( event )
    {
    /* Bluetooth stack enabled */
    case BTM_ENABLED_EVT:
        battery_service_application_init();
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        WICED_BT_TRACE("numeric_value: %d \n", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
        wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap   = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data       = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req       = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size   = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys      = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys      = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
        WICED_BT_TRACE( "Pairing Complete: %d",p_info->reason);
        battery_service_smp_bond_result( p_info->reason );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->paired_device_link_keys_update;
        wiced_hal_write_nvram ( BATTERY_SERVICE_PAIRED_KEYS_VS_ID, sizeof( wiced_bt_device_link_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("keys save to NVRAM %B result: %d \n", p_keys, result);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->paired_device_link_keys_request;
        wiced_hal_read_nvram( BATTERY_SERVICE_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p_keys, &result );
        WICED_BT_TRACE("keys read from NVRAM %B result: %d \n", p_keys, result);
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
        wiced_hal_write_nvram ( BATTERY_SERVICE_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
        WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
        break;


    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( BATTERY_SERVICE_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
        WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_status = &p_event_data->encryption_status;
        WICED_BT_TRACE("Encryption Status Event: bd ( %B ) res %d \n", p_status->bd_addr, p_status->result);
        battery_service_encryption_changed( p_status->result, p_status->bd_addr );
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        p_mode = &p_event_data->ble_advert_state_changed;

        WICED_BT_TRACE("Advertisement State Change: %d\n", *p_mode);
        if ( *p_mode == BTM_BLE_ADVERT_OFF )
        {
            if ( !battery_service_app_state.conn_id )
            {
                result =  wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL );
                WICED_BT_TRACE("wiced_bt_start_advertisements: %d\n", result );
            }
            else
            {
                WICED_BT_TRACE("ADV stop\n");
            }
        }

        break;

    default:
        break;
    }

    return result;
}

static void battery_service_load_keys_for_address_resolution( void )
{
    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t              result;
    uint8_t                     *p;

    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
    p = (uint8_t*)&link_keys;
    wiced_hal_read_nvram( BATTERY_SERVICE_PAIRED_KEYS_VS_ID, sizeof(wiced_bt_device_link_keys_t), p, &result);

    if(result == WICED_BT_SUCCESS)
    {
#ifdef CYW20706A2
        result = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys, link_keys.key_data.ble_addr_type );
#else
        result = wiced_bt_dev_add_device_to_address_resolution_db(&link_keys);
#endif
    }
    WICED_BT_TRACE("battery_service_load_keys_for_address_resolution %B result:%d \n", p, result );
}

static wiced_bt_gatt_status_t battery_service_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_result_t result = WICED_BT_GATT_INVALID_PDU;

    WICED_BT_TRACE( "battery_service_gatts_req_cb. conn %d, type %d\n", p_data->conn_id, p_data->request_type );

    switch ( p_data->request_type )
    {
    case GATTS_REQ_TYPE_READ:
        result = battery_service_gatts_req_read_handler( p_data->conn_id, &(p_data->data.read_req) );
        break;

    case GATTS_REQ_TYPE_WRITE:
        result = battery_service_gatts_req_write_handler( p_data->conn_id, &(p_data->data.write_req) );
        break;

    case GATTS_REQ_TYPE_WRITE_EXEC:
        break;

    case GATTS_REQ_TYPE_MTU:
        break;

    case GATTS_REQ_TYPE_CONF:
        break;

   default:
        break;
    }

    return result;
}

/*
 * Find attribute description by handle
 */
attribute_t * battery_service_get_attribute( uint16_t handle )
{
    int i;
    for ( i = 0; i <  sizeof( gatt_user_attributes ) / sizeof( gatt_user_attributes[0] ); i++ )
    {
        if ( gatt_user_attributes[i].handle == handle )
        {
            return ( &gatt_user_attributes[i] );
        }
    }
    WICED_BT_TRACE( "attr not found:%x\n", handle );
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
static wiced_bt_gatt_status_t battery_service_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    attribute_t *puAttribute;
    int          attr_len_to_copy;

    if ( ( puAttribute = battery_service_get_attribute(p_read_data->handle) ) == NULL)
    {
        WICED_BT_TRACE("read_hndlr attr not found hdl:%x\n", p_read_data->handle );
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    attr_len_to_copy = puAttribute->attr_len;

    WICED_BT_TRACE("read_hndlr conn_id:%d hdl:%x offset:%d len:%d\n", conn_id, p_read_data->handle, p_read_data->offset, attr_len_to_copy );

    if ( p_read_data->offset >= puAttribute->attr_len )
    {
        attr_len_to_copy = 0;
    }

    if ( attr_len_to_copy != 0 )
    {
        uint8_t *from;
        int      to_copy = attr_len_to_copy - p_read_data->offset;


        if ( to_copy > *p_read_data->p_val_len )
        {
            to_copy = *p_read_data->p_val_len;
        }

        from = ((uint8_t *)puAttribute->p_attr) + p_read_data->offset;
        *p_read_data->p_val_len = to_copy;

        memcpy( p_read_data->p_val, from, to_copy);
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process write request or write command from peer device
 */
static wiced_bt_gatt_status_t battery_service_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t result    = WICED_BT_GATT_SUCCESS;
    uint8_t                *p_attr   = p_data->p_val;
    uint8_t                nv_update = WICED_FALSE;

    WICED_BT_TRACE("write_handler: conn_id:%d hdl:0x%x prep:%d offset:%d len:%d\n ", conn_id, p_data->handle, p_data->is_prep, p_data->offset, p_data->val_len );

    switch ( p_data->handle )
    {
        case HANDLE_BATERY_SERVICE_BATTERY_LEVEL_CHAR_CFG_DESC:
            if ( p_data->val_len != 2 )
            {
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            battery_service_hostinfo.characteristic_client_configuration = p_attr[0] | ( p_attr[1] << 8 );

            if( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
            {
                wiced_start_timer( &battery_service_timer, BATTERY_SERVICE_TIMER_DURATION_IN_MS );
            }
            else
            {
                wiced_stop_timer( &battery_service_timer );
            }

            nv_update = WICED_TRUE;
            break;
        case HANDLE_BATERY_SERVICE_BATTERY_LEVEL_STATE_CHAR_CFG_DESC :
                   if ( p_data->val_len != 2 )
                   {
                       return WICED_BT_GATT_INVALID_ATTR_LEN;
                   }
                   battery_service_hostinfo.characteristic_client_configuration = p_attr[2] | ( p_attr[3] << 8 );

                   if( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
                   {
                       wiced_start_timer( &battery_service_timer, BATTERY_SERVICE_TIMER_DURATION_IN_MS );
                   }
                   else
                   {
                       wiced_stop_timer( &battery_service_timer );
                   }

                   nv_update = WICED_TRUE;
                   break;
        case HANDLE_BATERY_SERVICE_BATTERY_POWER_STATE_CHAR_CFG_DESC:
                           if ( p_data->val_len != 2 )
                           {
                               return WICED_BT_GATT_INVALID_ATTR_LEN;
                           }
                           battery_service_hostinfo.characteristic_client_configuration = p_attr[4] | ( p_attr[5] << 8 );

                           if( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
                           {
                               wiced_start_timer( &battery_service_timer, BATTERY_SERVICE_TIMER_DURATION_IN_MS );
                           }
                           else
                           {
                               wiced_stop_timer( &battery_service_timer );
                           }

                           nv_update = WICED_TRUE;
                           break;
        case HANDLE_BATERY_SERVICE_BATTERY_HEALTH_INFORMATION_CHAR_CFG_DESC:
                           if ( p_data->val_len != 2 )
                                   {
                                       return WICED_BT_GATT_INVALID_ATTR_LEN;
                                   }
                                   battery_service_hostinfo.characteristic_client_configuration = p_attr[6] | ( p_attr[7] << 8 );

                                   if( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
                                   {
                                       wiced_start_timer( &battery_service_timer, BATTERY_SERVICE_TIMER_DURATION_IN_MS );
                                   }
                                   else
                                   {
                                       wiced_stop_timer( &battery_service_timer );
                                   }

                                   nv_update = WICED_TRUE;
                                   break;
        default:
            result = WICED_BT_GATT_INVALID_HANDLE;
            break;
    }

    if ( nv_update )
    {
        wiced_result_t rc;
        int bytes_written = wiced_hal_write_nvram( BATTERY_SERVICE_VS_ID, sizeof(battery_service_hostinfo), (uint8_t*)&battery_service_hostinfo, &rc );
        WICED_BT_TRACE("NVRAM write:%d rc:%d", bytes_written, rc);
    }

    return result;
}

/*
 * Process SMP bonding result. If we successfully paired with the
 * central device, save its BDADDR in the NVRAM and initialize
 * associated data
 */

static void battery_service_smp_bond_result( uint8_t result )
{
    wiced_result_t status;
    uint8_t written_byte = 0;
    WICED_BT_TRACE( "battery_info, bond result: %d\n", result );
}

/*
 * Process notification from stack that encryption has been set. If connected
 * client is registered for notification or indication, it is a good time to
 * send it out
 */
static void battery_service_encryption_changed( wiced_result_t result, uint8_t* bd_addr )
{
    WICED_BT_TRACE( "encryption change bd ( %B ) res: %d \n", battery_service_hostinfo.bdaddr,  result);

    /* Connection has been encrypted meaning that we have correct/paired device
     * restore values in the database
     */
   if( result == WICED_SUCCESS )
   {
        wiced_hal_read_nvram( BATTERY_SERVICE_VS_ID, sizeof(battery_service_hostinfo), (uint8_t*)&battery_service_hostinfo, &result );

        if( result == WICED_SUCCESS )
        {
            if ( (memcmp(battery_service_hostinfo.bdaddr, battery_service_app_state.remote_addr, BD_ADDR_LEN) == 0) )
            {
                if( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
                {
                    /* Ideally, after encryption server should comes to know that client had registered for notifications and
                     * application has to check whether the current battery level is different than the previous notified battery level
                     * and then the client need to be notified. So essentially only one notification should be resulted.
                     * But with our current dummy battery level implementation, it is not possible to get current battery level,
                     *  so battery level should be notified with value less than 1 level compared to last notification.
                     */
                    battery_service_app_state.battery_level;
                    battery_service_send_message();
                    wiced_start_timer( &battery_service_timer, BATTERY_SERVICE_TIMER_DURATION_IN_MS );
                }
            }
        }
   }
}

static void battery_service_send_message()
{
    wiced_bt_gatt_status_t result;
    wiced_result_t status;
    uint8_t written_byte = 1;
    WICED_BT_TRACE( "battery_service_send_message: CCC:%d\n", battery_service_hostinfo.characteristic_client_configuration );

    /* If client has not registered for indication or notification, no action */
    if ( battery_service_hostinfo.characteristic_client_configuration == 0 )
    {
        return;
    }
    else if ( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
    {
        uint8_t p_attr = battery_service_app_state.battery_level;
        //uint8_t q_attr = battery_state.level;

        battery_service_app_state.flag_notify_sent = TRUE;
        WICED_BT_TRACE("battery--------------------- : %d\n",battery_service_app_state.conn_id);
       result = wiced_bt_gatt_send_notification( battery_service_app_state.conn_id, HANDLE_BATTERY_SERVICE_CHAR_LEVEL_VAL, sizeof(p_attr), &p_attr );
        WICED_BT_TRACE("battery notification olddddddddddddddd : %d\n",result);

    }
}
void store_connection_id(uint16_t conn_id) {
    global_conn_id = conn_id;
}
static void  battery_service_notify_state_level() {
        wiced_bt_gatt_status_t result1;


       // uint16_t get_connection_id() {
        //}
        WICED_BT_TRACE( "battery_service_send_message: CCC:%d\n", battery_service_hostinfo.characteristic_client_configuration );

        if ( battery_service_hostinfo.characteristic_client_configuration == 0 )
           {
               return;
           }
           else if ( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
           {

               result1 = wiced_bt_gatt_send_notification(global_conn_id, HANDLE_BATTERY_SERVICE_CHAR_LEVEL_STATE_VALUE,
                                        sizeof(battery_state), (uint8_t *)&battery_state);

           }

    }

static void   battery_service_power_state() {

    wiced_bt_gatt_status_t resultt;
    WICED_BT_TRACE( "battery_service_send_message: CCC:%d\n", battery_service_hostinfo.characteristic_client_configuration );
    if ( battery_service_hostinfo.characteristic_client_configuration == 0 )
               {
                   return;
               }
               else if ( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
               {

                   resultt = wiced_bt_gatt_send_notification(global_conn_id, HANDLE_BATTERY_SERVICE_CHAR_POWER_STATE_VALUE,
                                            sizeof(power_state), (uint8_t *)&power_state);

               }

}
static void battery_service_health()
{
    wiced_bt_gatt_status_t result;
        WICED_BT_TRACE( "battery_service_send_message: CCC:%d\n", battery_service_hostinfo.characteristic_client_configuration );
        if ( battery_service_hostinfo.characteristic_client_configuration == 0 )
                   {
                       return;
                   }
                   else if ( battery_service_hostinfo.characteristic_client_configuration & GATT_CLIENT_CONFIG_NOTIFICATION )
                   {

                       result = wiced_bt_gatt_send_notification(global_conn_id, HANDLE_BATTERY_SERVICE_CHAR_HEALTH_INFORMATION_VALUE,
                                                sizeof(health_info), (uint8_t *)&health_info);

                   }

    }




#ifdef ENABLE_HCI_TRACE
void battery_service_hci_trace_cback( wiced_bt_hci_trace_type_t type, uintt length, uint8_t* p_data )
{
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}
#endif
