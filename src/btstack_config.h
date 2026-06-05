// https://github.com/rafaelvaloto/Pico_W-Dualsense/blob/main/btstack_config.h

// c
#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

#if ENABLE_NS2PRO
#ifndef ENABLE_BLE
#define ENABLE_BLE
#endif
#ifndef ENABLE_LE_CENTRAL
#define ENABLE_LE_CENTRAL
#endif
#ifndef ENABLE_LE_PERIPHERAL
#define ENABLE_LE_PERIPHERAL
#endif
#ifndef ENABLE_GATT_CLIENT
#define ENABLE_GATT_CLIENT
#endif
#ifndef ENABLE_LE_SECURE_CONNECTIONS
#define ENABLE_LE_SECURE_CONNECTIONS
#endif
#ifndef ENABLE_LE_PROACTIVE_AUTHENTICATION
#define ENABLE_LE_PROACTIVE_AUTHENTICATION
#endif
#else
#ifndef ENABLE_CLASSIC
#define ENABLE_CLASSIC
#endif
#endif


// CYW43 HCI Transport requires pre-buffer space for packet header

// Se estiver 1 ou 2, o 0x31 do DualSense causa estouro
#define MAX_NR_HCI_ACL_PACKETS 4

#define MAX_NR_HCI_CONNECTIONS 1
#if ENABLE_NS2PRO
#define MAX_NR_L2CAP_CHANNELS  4
#define MAX_NR_L2CAP_SERVICES  0
#define MAX_NR_GATT_CLIENTS    1
#define MAX_NR_SM_LOOKUP_ENTRIES 4
#define MAX_NR_LE_DEVICE_DB_ENTRIES 4
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_ATT_DB_SIZE 512
#else
#define MAX_NR_L2CAP_CHANNELS  2
#define MAX_NR_L2CAP_SERVICES  3 // GDP + CONTROL + INTERRUPT
#endif
//
#define HCI_ACL_PAYLOAD_SIZE 1021
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4


#define MAX_NR_RFCOMM_MULTIPLEXERS 0
#define MAX_NR_RFCOMM_SERVICES 0
#define MAX_NR_RFCOMM_CHANNELS 0

// CYW43 específico - necessário para o transport layer

#define NVM_NUM_LINK_KEYS 4
#define NVM_NUM_DEVICE_DB_ENTRIES 4
#define HAVE_EMBEDDED_TIME_MS

// Logging
#define ENABLE_PRINTF_HEXDUMP
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR

#endif
