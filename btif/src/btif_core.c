/******************************************************************************
 *
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/************************************************************************************
 *
 *  Filename:      btif_core.c
 *
 *  Description:   Contains core functionality related to interfacing between
 *                 Bluetooth HAL and BTE core stack.
 *
 ***********************************************************************************/

#include <stdlib.h>
#include <hardware/bluetooth.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <cutils/properties.h>

#define LOG_TAG "BTIF_CORE"
#include "btif_api.h"
#include "bt_utils.h"
#include "bta_api.h"
#include "gki.h"
#include "btu.h"
#include "bte.h"
#include "bd.h"
#include "btif_av.h"
#include "btif_storage.h"
#include "btif_util.h"
#include "btif_sock.h"
#include "btif_pan.h"
#include "btif_profile_queue.h"
#include "btif_config.h"
/************************************************************************************
**  Constants & Macros
************************************************************************************/

#ifndef BTIF_TASK_STACK_SIZE
#define BTIF_TASK_STACK_SIZE       0x2000         /* In bytes */
#endif

#ifndef BTE_DID_CONF_FILE
#define BTE_DID_CONF_FILE "/etc/bluetooth/bt_did.conf"
#endif

#define BTIF_TASK_STR        ((INT8 *) "BTIF")

/************************************************************************************
**  Local type definitions
************************************************************************************/

/* These type definitions are used when passing data from the HAL to BTIF context
*  in the downstream path for the adapter and remote_device property APIs */

typedef struct {
  bt_bdaddr_t bd_addr;
  bt_property_type_t type;
} btif_storage_read_t;

typedef struct {
  bt_bdaddr_t bd_addr;
  bt_property_t prop;
} btif_storage_write_t;

typedef union {
  btif_storage_read_t read_req;
  btif_storage_write_t write_req;
} btif_storage_req_t;

typedef enum {
    BTIF_CORE_STATE_DISABLED = 0,
    BTIF_CORE_STATE_ENABLING,
    BTIF_CORE_STATE_ENABLED,
    BTIF_CORE_STATE_DISABLING
} btif_core_state_t;

/************************************************************************************
**  Static variables
************************************************************************************/

bt_bdaddr_t btif_local_bd_addr;

static UINT32 btif_task_stack[(BTIF_TASK_STACK_SIZE + 3) / 4];

/* holds main adapter state */
static btif_core_state_t btif_core_state = BTIF_CORE_STATE_DISABLED;

static int btif_shutdown_pending = 0;
static tBTA_SERVICE_MASK btif_enabled_services = 0;

/*
* This variable should be set to 1, if the Bluedroid+BTIF libraries are to
* function in DUT mode.
*
* To set this, the btif_init_bluetooth needs to be called with argument as 1
*/
static UINT8 btif_dut_mode = 0;

/************************************************************************************
**  Static functions
************************************************************************************/
static bt_status_t btif_associate_evt(void);
static bt_status_t btif_disassociate_evt(void);

/* sends message to btif task */
static void btif_sendmsg(void *p_msg);

/************************************************************************************
**  Externs
************************************************************************************/
extern void bte_load_did_conf(const char *p_path);

/** TODO: Move these to _common.h */
void bte_main_boot_entry(void);
void bte_main_enable();
void bte_main_disable(void);
void bte_main_shutdown(void);
#if (defined(HCILP_INCLUDED) && HCILP_INCLUDED == TRUE)
void bte_main_enable_lpm(BOOLEAN enable);
#endif
void bte_main_postload_cfg(void);
void btif_dm_execute_service_request(UINT16 event, char *p_param);
#ifdef BTIF_DM_OOB_TEST
void btif_dm_load_local_oob(void);
#endif
void bte_main_config_hci_logging(BOOLEAN enable, BOOLEAN bt_disabled);

/************************************************************************************
**  Functions
************************************************************************************/
static bt_status_t btif_in_get_adapter_properties(void);
/*MOCKAIC*/static void btd_to_btif(BT_HDR *msg);

/*****************************************************************************
**   Context switching functions
*****************************************************************************/


/*******************************************************************************
**
** Function         btif_context_switched
**
** Description      Callback used to execute transferred context callback
**
**                  p_msg : message to be executed in btif context
**
** Returns          void
**
*******************************************************************************/

static void btif_context_switched(void *p_msg)
{
    tBTIF_CONTEXT_SWITCH_CBACK *p;

    BTIF_TRACE_ERROR0("btif_context_switched");

    p = (tBTIF_CONTEXT_SWITCH_CBACK *) p_msg;

    /* each callback knows how to parse the data */
    if (p->p_cb)
        p->p_cb(p->event, p->p_param);
}


/*******************************************************************************
**
** Function         btif_transfer_context
**
** Description      This function switches context to btif task
**
**                  p_cback   : callback used to process message in btif context
**                  event     : event id of message
**                  p_params  : parameter area passed to callback (copied)
**                  param_len : length of parameter area
**                  p_copy_cback : If set this function will be invoked for deep copy
**
** Returns          void
**
*******************************************************************************/

bt_status_t btif_transfer_context (tBTIF_CBACK *p_cback, UINT16 event, char* p_params, int param_len, tBTIF_COPY_CBACK *p_copy_cback)
{
    tBTIF_CONTEXT_SWITCH_CBACK *p_msg;

    BTIF_TRACE_ERROR2("btif_transfer_context event %d, len %d", event, param_len);

    /* allocate and send message that will be executed in btif context */
    if ((p_msg = (tBTIF_CONTEXT_SWITCH_CBACK *) GKI_getbuf(sizeof(tBTIF_CONTEXT_SWITCH_CBACK) + param_len)) != NULL)
    {
        p_msg->hdr.event = BT_EVT_CONTEXT_SWITCH_EVT; /* internal event */
        p_msg->p_cb = p_cback;

        p_msg->event = event;                         /* callback event */

        /* check if caller has provided a copy callback to do the deep copy */
        if (p_copy_cback)
        {
            p_copy_cback(event, p_msg->p_param, p_params);
        }
        else if (p_params)
        {
            memcpy(p_msg->p_param, p_params, param_len);  /* callback parameter data */
        }

        btif_sendmsg(p_msg);
        return BT_STATUS_SUCCESS;
    }
    else
    {
        /* let caller deal with a failed allocation */
        return BT_STATUS_NOMEM;
    }
}

/*******************************************************************************
**
** Function         btif_is_dut_mode
**
** Description      checks if BTIF is currently in DUT mode
**
** Returns          1 if test mode, otherwize 0
**
*******************************************************************************/

UINT8 btif_is_dut_mode(void)
{
    return (btif_dut_mode == 1);
}

/*******************************************************************************
**
** Function         btif_is_enabled
**
** Description      checks if main adapter is fully enabled
**
** Returns          1 if fully enabled, otherwize 0
**
*******************************************************************************/

int btif_is_enabled(void)
{
    return 1;/*MOCKAIC*///((!btif_is_dut_mode()) && (btif_core_state == BTIF_CORE_STATE_ENABLED));
}

/*******************************************************************************
**
** Function         btif_task
**
** Description      BTIF task handler managing all messages being passed
**                  Bluetooth HAL and BTA.
**
** Returns          void
**
*******************************************************************************/

static void btif_task(UINT32 params)
{
    UINT16   event;
    BT_HDR   *p_msg;

    BTIF_TRACE_ERROR0("btif_task: starting");

    btif_associate_evt();

    for(;;)
    {
        /* wait for specified events */
        event = GKI_wait(0xFFFF, 0);

        /*
         * Wait for the trigger to init chip and stack. This trigger will
         * be received by btu_task once the UART is opened and ready
         */
        if (event == BT_EVT_TRIGGER_STACK_INIT)
        {
            BTIF_TRACE_ERROR0("btif_task: received trigger stack init event");
            #if (BLE_INCLUDED == TRUE)
            btif_dm_load_ble_local_keys();
            #endif
            BTA_EnableBluetooth(bte_dm_evt);
        }

        /*
         * Failed to initialize controller hardware, reset state and bring
         * down all threads
         */
        if (event == BT_EVT_HARDWARE_INIT_FAIL)
        {
            BTIF_TRACE_ERROR0("btif_task: hardware init failed");
            bte_main_disable();
            btif_queue_release();
            GKI_task_self_cleanup(BTIF_TASK);
            bte_main_shutdown();
            btif_dut_mode = 0;
            btif_core_state = BTIF_CORE_STATE_DISABLED;
            HAL_CBACK(bt_hal_cbacks,adapter_state_changed_cb,BT_STATE_OFF);
            break;
        }

        if (event & EVENT_MASK(GKI_SHUTDOWN_EVT))
        {
            BTIF_TRACE_ERROR0("btif_task: GKI_SHUTDOWN_EVT");
            break;
        }

        if(event & TASK_MBOX_1_EVT_MASK)
        {
            BTIF_TRACE_ERROR0("btif_task:  fetched event A");
            while((p_msg = GKI_read_mbox(BTU_BTIF_MBOX)) != NULL)
            {
                BTIF_TRACE_ERROR0("btif_task:  fetched event B");
                switch (p_msg->event)
                {
                    case BT_EVT_CONTEXT_SWITCH_EVT:
                        BTIF_TRACE_ERROR0("btif_task:  fetched event BT_EVT_CONTEXT_SWITCH_EVT");
                        btif_context_switched(p_msg);
                        break;
                    case BT_EVT_CONTEXT_AICSET_EVT:
                        BTIF_TRACE_ERROR0("btif_task:  fetched event BT_EVT_CONTEXT_AICSET_EVT");
                        btd_to_btif(p_msg);
                        break;
                    default:
                        BTIF_TRACE_ERROR0("btif_task:  fetched event default");
                        break;
             }
                GKI_freebuf(p_msg);
            }
        }

    }

    btif_disassociate_evt();

    BTIF_TRACE_ERROR0("btif_task: btif task exiting");
}


/*******************************************************************************
**
** Function         btif_sendmsg
**
** Description      Sends msg to BTIF task
**
** Returns          void
**
*******************************************************************************/

void btif_sendmsg(void *p_msg)
{
    GKI_send_msg(BTIF_TASK, BTU_BTIF_MBOX, p_msg);
}

static void btif_fetch_local_bdaddr(bt_bdaddr_t *local_addr)
{
    char val[256];
    uint8_t valid_bda = FALSE;
    int val_size = 0;
    const uint8_t null_bdaddr[BD_ADDR_LEN] = {0,0,0,0,0,0};

    /* Get local bdaddr storage path from property */
    if (property_get(PROPERTY_BT_BDADDR_PATH, val, NULL))
    {
        int addr_fd;

        BTIF_TRACE_ERROR1("local bdaddr is stored in %s", val);

        if ((addr_fd = open(val, O_RDONLY)) != -1)
        {
            memset(val, 0, sizeof(val));
            read(addr_fd, val, FACTORY_BT_BDADDR_STORAGE_LEN);
            str2bd(val, local_addr);
            /* If this is not a reserved/special bda, then use it */
            if (memcmp(local_addr->address, null_bdaddr, BD_ADDR_LEN) != 0)
            {
                valid_bda = TRUE;
                BTIF_TRACE_ERROR6("Got Factory BDA %02X:%02X:%02X:%02X:%02X:%02X",
                    local_addr->address[0], local_addr->address[1], local_addr->address[2],
                    local_addr->address[3], local_addr->address[4], local_addr->address[5]);
            }

            close(addr_fd);
        }
    }

    if(!valid_bda)
    {
        val_size = sizeof(val);
        if(btif_config_get_str("Local", "Adapter", "Address", val, &val_size))
        {
            str2bd(val, local_addr);
            BTIF_TRACE_ERROR1("local bdaddr from bt_config.xml is  %s", val);
            return;
        }
     }

    /* No factory BDADDR found. Look for previously generated random BDA */
    if ((!valid_bda) && \
        (property_get(PERSIST_BDADDR_PROPERTY, val, NULL)))
    {
        str2bd(val, local_addr);
        valid_bda = TRUE;
        BTIF_TRACE_ERROR6("Got prior random BDA %02X:%02X:%02X:%02X:%02X:%02X",
            local_addr->address[0], local_addr->address[1], local_addr->address[2],
            local_addr->address[3], local_addr->address[4], local_addr->address[5]);
    }

    /* Generate new BDA if necessary */
    if (!valid_bda)
    {
        bdstr_t bdstr;
        /* Seed the random number generator */
        srand((unsigned int) (time(0)));

        /* No autogen BDA. Generate one now. */
        local_addr->address[0] = 0x22;
        local_addr->address[1] = 0x22;
        local_addr->address[2] = (uint8_t) ((rand() >> 8) & 0xFF);
        local_addr->address[3] = (uint8_t) ((rand() >> 8) & 0xFF);
        local_addr->address[4] = (uint8_t) ((rand() >> 8) & 0xFF);
        local_addr->address[5] = (uint8_t) ((rand() >> 8) & 0xFF);

        /* Convert to ascii, and store as a persistent property */
        bd2str(local_addr, &bdstr);

        BTIF_TRACE_ERROR2("No preset BDA. Generating BDA: %s for prop %s",
             (char*)bdstr, PERSIST_BDADDR_PROPERTY);

        if (property_set(PERSIST_BDADDR_PROPERTY, (char*)bdstr) < 0)
            BTIF_TRACE_ERROR1("Failed to set random BDA in prop %s",PERSIST_BDADDR_PROPERTY);
    }

    //save the bd address to config file
    bdstr_t bdstr;
    bd2str(local_addr, &bdstr);
    val_size = sizeof(val);
    if (btif_config_get_str("Local", "Adapter", "Address", val, &val_size))
    {
        if (strcmp(bdstr, val) ==0)
        {
            // BDA is already present in the config file.
            return;
        }
    }
    btif_config_set_str("Local", "Adapter", "Address", bdstr);
    btif_config_save();
}

/*****************************************************************************
**
**   btif core api functions
**
*****************************************************************************/

/*******************************************************************************
**
** Function         btif_init_bluetooth
**
** Description      Creates BTIF task and prepares BT scheduler for startup
**
** Returns          bt_status_t
**
*******************************************************************************/

bt_status_t btif_init_bluetooth()
{
    UINT8 status;

    BTIF_TRACE_ERROR0("Creates BTIF task and prepares BT scheduler for startup");


    btif_config_init();
    bte_main_boot_entry();

    /* As part of the init, fetch the local BD ADDR */
    BTIF_TRACE_ERROR0("As part of the init, fetch the local BD ADDR");

    memset(&btif_local_bd_addr, 0, sizeof(bt_bdaddr_t));
    btif_fetch_local_bdaddr(&btif_local_bd_addr);

    BTIF_TRACE_ERROR0("start btif task ");
    /* start btif task */
    status = GKI_create_task(btif_task, BTIF_TASK, BTIF_TASK_STR,
                (UINT16 *) ((UINT8 *)btif_task_stack + BTIF_TASK_STACK_SIZE),
                sizeof(btif_task_stack));

    if (status != GKI_SUCCESS)
        return BT_STATUS_FAIL;

    return BT_STATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         btif_associate_evt
**
** Description      Event indicating btif_task is up
**                  Attach btif_task to JVM
**
** Returns          void
**
*******************************************************************************/

static bt_status_t btif_associate_evt(void)
{
    BTIF_TRACE_ERROR1("%s: notify ASSOCIATE_JVM", __FUNCTION__);
    HAL_CBACK(bt_hal_cbacks, thread_evt_cb, ASSOCIATE_JVM);

    return BT_STATUS_SUCCESS;
}


/*******************************************************************************
**
** Function         btif_enable_bluetooth
**
** Description      Performs chip power on and kickstarts OS scheduler
**
** Returns          bt_status_t
**
*******************************************************************************/

bt_status_t btif_enable_bluetooth(void)
{
    BTIF_TRACE_ERROR0("BTIF ENABLE BLUETOOTH");

    if (btif_core_state != BTIF_CORE_STATE_DISABLED)
    {
        ALOGD("not disabled\n");
        //return BT_STATUS_DONE;
    }

    btif_core_state = BTIF_CORE_STATE_ENABLING;

    /* Create the GKI tasks and run them */
    bte_main_enable();

    return BT_STATUS_SUCCESS;
}


/*******************************************************************************
**
** Function         btif_enable_bluetooth_evt
**
** Description      Event indicating bluetooth enable is completed
**                  Notifies HAL user with updated adapter state
**
** Returns          void
**
*******************************************************************************/

void btif_enable_bluetooth_evt(tBTA_STATUS status, BD_ADDR local_bd)
{
    bt_bdaddr_t bd_addr;
    bdstr_t bdstr;

    bdcpy(bd_addr.address, local_bd);
    BTIF_TRACE_ERROR3("%s: status %d, local bd [%s]", __FUNCTION__, status,
                                                     bd2str(&bd_addr, &bdstr));
    status = BTA_SUCCESS;

    if (bdcmp(btif_local_bd_addr.address,local_bd))
    {
        bdstr_t buf;
        bt_property_t prop;

        /**
         * The Controller's BDADDR does not match to the BTIF's initial BDADDR!
         * This could be because the factory BDADDR was stored separatley in
         * the Controller's non-volatile memory rather than in device's file
         * system.
         **/
        BTIF_TRACE_WARNING0("***********************************************");
        BTIF_TRACE_WARNING6("BTIF init BDA was %02X:%02X:%02X:%02X:%02X:%02X",
            btif_local_bd_addr.address[0], btif_local_bd_addr.address[1],
            btif_local_bd_addr.address[2], btif_local_bd_addr.address[3],
            btif_local_bd_addr.address[4], btif_local_bd_addr.address[5]);
        BTIF_TRACE_WARNING6("Controller BDA is %02X:%02X:%02X:%02X:%02X:%02X",
            local_bd[0], local_bd[1], local_bd[2],
            local_bd[3], local_bd[4], local_bd[5]);
        BTIF_TRACE_WARNING0("***********************************************");

        bdcpy(btif_local_bd_addr.address, local_bd);

        //save the bd address to config file
        bd2str(&btif_local_bd_addr, &buf);
        btif_config_set_str("Local", "Adapter", "Address", buf);
        btif_config_save();

        //fire HAL callback for property change
        memcpy(buf, &btif_local_bd_addr, sizeof(bt_bdaddr_t));
        prop.type = BT_PROPERTY_BDADDR;
        prop.val = (void*)buf;
        prop.len = sizeof(bt_bdaddr_t);
        HAL_CBACK(bt_hal_cbacks, adapter_properties_cb, BT_STATUS_SUCCESS, 1, &prop);
    }

    bte_main_postload_cfg();
#if (defined(HCILP_INCLUDED) && HCILP_INCLUDED == TRUE)
    bte_main_enable_lpm(TRUE);
#endif
    /* add passing up bd address as well ? */

    /* callback to HAL */
    if (status == BTA_SUCCESS)
    {
        /* initialize a2dp service */
        btif_av_init();

        /* init rfcomm & l2cap api */
        btif_sock_init();

        /* init pan */
        btif_pan_init();

        /* load did configuration */
        bte_load_did_conf(BTE_DID_CONF_FILE);

#ifdef BTIF_DM_OOB_TEST
        btif_dm_load_local_oob();
#endif
        /* now fully enabled, update state */
        btif_core_state = BTIF_CORE_STATE_ENABLED;

        HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_ON);
    }
    else
    {
        /* cleanup rfcomm & l2cap api */
        btif_sock_cleanup();

        btif_pan_cleanup();

        /* we failed to enable, reset state */
        btif_core_state = BTIF_CORE_STATE_DISABLED;

        HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_OFF);
    }
}

/*******************************************************************************
**
** Function         btif_disable_bluetooth
**
** Description      Inititates shutdown of Bluetooth system.
**                  Any active links will be dropped and device entering
**                  non connectable/discoverable mode
**
** Returns          void
**
*******************************************************************************/
bt_status_t btif_disable_bluetooth(void)
{
    tBTA_STATUS status;

    if (!btif_is_enabled())
    {
        BTIF_TRACE_ERROR0("btif_disable_bluetooth : not yet enabled");
        return BT_STATUS_NOT_READY;
    }

    BTIF_TRACE_ERROR0("BTIF DISABLE BLUETOOTH");

    btif_dm_on_disable();
    btif_core_state = BTIF_CORE_STATE_DISABLING;

    /* cleanup rfcomm & l2cap api */
    btif_sock_cleanup();

    btif_pan_cleanup();

    status = BTA_DisableBluetooth();

    btif_config_flush();

    if (status != BTA_SUCCESS)
    {
        BTIF_TRACE_ERROR1("disable bt failed (%d)", status);

        /* reset the original state to allow attempting disable again */
        btif_core_state = BTIF_CORE_STATE_ENABLED;

        return BT_STATUS_FAIL;
    }
    return BT_STATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         btif_disable_bluetooth_evt
**
** Description      Event notifying BT disable is now complete.
**                  Terminates main stack tasks and notifies HAL
**                  user with updated BT state.
**
** Returns          void
**
*******************************************************************************/

void btif_disable_bluetooth_evt(void)
{
    BTIF_TRACE_ERROR1("%s", __FUNCTION__);

#if (defined(HCILP_INCLUDED) && HCILP_INCLUDED == TRUE)
    bte_main_enable_lpm(FALSE);
#endif

    bte_main_disable();

    /* update local state */
    btif_core_state = BTIF_CORE_STATE_DISABLED;

    /* callback to HAL */
    HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_OFF);

    if (btif_shutdown_pending)
    {
        BTIF_TRACE_ERROR1("%s: calling btif_shutdown_bluetooth", __FUNCTION__);
        btif_shutdown_bluetooth();
    }
}


/*******************************************************************************
**
** Function         btif_shutdown_bluetooth
**
** Description      Finalizes BT scheduler shutdown and terminates BTIF
**                  task.
**
** Returns          void
**
*******************************************************************************/

bt_status_t btif_shutdown_bluetooth(void)
{
    BTIF_TRACE_ERROR1("%s", __FUNCTION__);

    if (btif_core_state == BTIF_CORE_STATE_DISABLING)
    {
        BTIF_TRACE_WARNING0("shutdown during disabling");
        /* shutdown called before disabling is done */
        btif_shutdown_pending = 1;
        return BT_STATUS_NOT_READY;
    }

    if (btif_is_enabled())
    {
        BTIF_TRACE_WARNING0("shutdown while still enabled, initiate disable");

        /* shutdown called prior to disabling, initiate disable */
        btif_disable_bluetooth();
        btif_shutdown_pending = 1;
        return BT_STATUS_NOT_READY;
    }

    btif_shutdown_pending = 0;

    if (btif_core_state == BTIF_CORE_STATE_ENABLING)
    {
        // Java layer abort BT ENABLING, could be due to ENABLE TIMEOUT
        // Direct call from cleanup()@bluetooth.c
        // bring down HCI/Vendor lib
        bte_main_disable();
        btif_core_state = BTIF_CORE_STATE_DISABLED;
        HAL_CBACK(bt_hal_cbacks, adapter_state_changed_cb, BT_STATE_OFF);
    }

    GKI_destroy_task(BTIF_TASK);
    btif_queue_release();
    bte_main_shutdown();

    btif_dut_mode = 0;

    bt_utils_cleanup();

    BTIF_TRACE_ERROR1("%s done", __FUNCTION__);

    return BT_STATUS_SUCCESS;
}


/*******************************************************************************
**
** Function         btif_disassociate_evt
**
** Description      Event indicating btif_task is going down
**                  Detach btif_task to JVM
**
** Returns          void
**
*******************************************************************************/

static bt_status_t btif_disassociate_evt(void)
{
    BTIF_TRACE_ERROR1("%s: notify DISASSOCIATE_JVM", __FUNCTION__);

    HAL_CBACK(bt_hal_cbacks, thread_evt_cb, DISASSOCIATE_JVM);

    /* shutdown complete, all events notified and we reset HAL callbacks */
    bt_hal_cbacks = NULL;

    return BT_STATUS_SUCCESS;
}

/****************************************************************************
**
**   BTIF Test Mode APIs
**
*****************************************************************************/
/*******************************************************************************
**
** Function         btif_dut_mode_cback
**
** Description     Callback invoked on completion of vendor specific test mode command
**
** Returns          None
**
*******************************************************************************/
static void btif_dut_mode_cback( tBTM_VSC_CMPL *p )
{
    /* For now nothing to be done. */
}

/*******************************************************************************
**
** Function         btif_dut_mode_configure
**
** Description      Configure Test Mode - 'enable' to 1 puts the device in test mode and 0 exits
**                       test mode
**
** Returns          BT_STATUS_SUCCESS on success
**
*******************************************************************************/
bt_status_t btif_dut_mode_configure(uint8_t enable)
{
    BTIF_TRACE_ERROR1("%s", __FUNCTION__);

    if (btif_core_state != BTIF_CORE_STATE_ENABLED) {
        BTIF_TRACE_ERROR0("btif_dut_mode_configure : Bluetooth not enabled");
        return BT_STATUS_NOT_READY;
    }

    btif_dut_mode = enable;
    if (enable == 1) {
        BTA_EnableTestMode();
    } else {
        BTA_DisableTestMode();
    }
    return BT_STATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         btif_dut_mode_send
**
** Description     Sends a HCI Vendor specific command to the controller
**
** Returns          BT_STATUS_SUCCESS on success
**
*******************************************************************************/
bt_status_t btif_dut_mode_send(uint16_t opcode, uint8_t *buf, uint8_t len)
{
    /* TODO: Check that opcode is a vendor command group */
    BTIF_TRACE_ERROR1("%s", __FUNCTION__);
    if (!btif_is_dut_mode()) {
         BTIF_TRACE_ERROR0("Bluedroid HAL needs to be init with test_mode set to 1.");
         return BT_STATUS_FAIL;
    }
    BTM_VendorSpecificCommand(opcode, len, buf, btif_dut_mode_cback);
    return BT_STATUS_SUCCESS;
}

/*****************************************************************************
**
**   btif api adapter property functions
**
*****************************************************************************/

static bt_status_t btif_in_get_adapter_properties(void)
{
    bt_property_t properties[6];
    uint32_t num_props;

    bt_bdaddr_t addr;
    bt_bdname_t name;
    bt_scan_mode_t mode;
    uint32_t disc_timeout;
    bt_bdaddr_t bonded_devices[BTM_SEC_MAX_DEVICE_RECORDS];
    bt_uuid_t local_uuids[BT_MAX_NUM_UUIDS];
    num_props = 0;

    /* BD_ADDR */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_BDADDR,
                               sizeof(addr), &addr);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* BD_NAME */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_BDNAME,
                               sizeof(name), &name);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* SCAN_MODE */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_ADAPTER_SCAN_MODE,
                               sizeof(mode), &mode);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* DISC_TIMEOUT */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT,
                               sizeof(disc_timeout), &disc_timeout);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* BONDED_DEVICES */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_ADAPTER_BONDED_DEVICES,
                               sizeof(bonded_devices), bonded_devices);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    /* LOCAL UUIDs */
    BTIF_STORAGE_FILL_PROPERTY(&properties[num_props], BT_PROPERTY_UUIDS,
                               sizeof(local_uuids), local_uuids);
    btif_storage_get_adapter_property(&properties[num_props]);
    num_props++;

    HAL_CBACK(bt_hal_cbacks, adapter_properties_cb,
                     BT_STATUS_SUCCESS, num_props, properties);

    return BT_STATUS_SUCCESS;
}

static bt_status_t btif_in_get_remote_device_properties(bt_bdaddr_t *bd_addr)
{
    bt_property_t remote_properties[8];
    uint32_t num_props = 0;

    bt_bdname_t name, alias;
    uint32_t cod, devtype;
    bt_uuid_t remote_uuids[BT_MAX_NUM_UUIDS];

    memset(remote_properties, 0, sizeof(remote_properties));
    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_BDNAME,
                               sizeof(name), &name);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_REMOTE_FRIENDLY_NAME,
                               sizeof(alias), &alias);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_CLASS_OF_DEVICE,
                               sizeof(cod), &cod);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_TYPE_OF_DEVICE,
                               sizeof(devtype), &devtype);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    BTIF_STORAGE_FILL_PROPERTY(&remote_properties[num_props], BT_PROPERTY_UUIDS,
                               sizeof(remote_uuids), remote_uuids);
    btif_storage_get_remote_device_property(bd_addr,
                                            &remote_properties[num_props]);
    num_props++;

    HAL_CBACK(bt_hal_cbacks, remote_device_properties_cb,
                     BT_STATUS_SUCCESS, bd_addr, num_props, remote_properties);

    return BT_STATUS_SUCCESS;
}


/*******************************************************************************
**
** Function         execute_storage_request
**
** Description      Executes adapter storage request in BTIF context
**
** Returns          bt_status_t
**
*******************************************************************************/

static void execute_storage_request(UINT16 event, char *p_param)
{
    uint8_t is_local;
    int num_entries = 0;
    bt_status_t status = BT_STATUS_SUCCESS;

    BTIF_TRACE_EVENT1("execute storage request event : %d", event);

    switch(event)
    {
        case BTIF_CORE_STORAGE_ADAPTER_WRITE:
        {
            btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
            bt_property_t *p_prop = &(p_req->write_req.prop);
            BTIF_TRACE_EVENT3("type: %d, len %d, 0x%x", p_prop->type,
                               p_prop->len, p_prop->val);

            status = btif_storage_set_adapter_property(p_prop);
            HAL_CBACK(bt_hal_cbacks, adapter_properties_cb, status, 1, p_prop);
        } break;

        case BTIF_CORE_STORAGE_ADAPTER_READ:
        {
            btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
            char buf[512];
            bt_property_t prop;
            prop.type = p_req->read_req.type;
            prop.val = (void*)buf;
            prop.len = sizeof(buf);
            status = btif_storage_get_adapter_property(&prop);
            HAL_CBACK(bt_hal_cbacks, adapter_properties_cb, status, 1, &prop);
        } break;

        case BTIF_CORE_STORAGE_ADAPTER_READ_ALL:
        {
            status = btif_in_get_adapter_properties();
        } break;

        case BTIF_CORE_STORAGE_NOTIFY_STATUS:
        {
            HAL_CBACK(bt_hal_cbacks, adapter_properties_cb, status, 0, NULL);
        } break;

        default:
            BTIF_TRACE_ERROR2("%s invalid event id (%d)", __FUNCTION__, event);
            break;
    }
}

static void execute_storage_remote_request(UINT16 event, char *p_param)
{
    bt_status_t status = BT_STATUS_FAIL;
    bt_property_t prop;

    BTIF_TRACE_EVENT1("execute storage remote request event : %d", event);

    switch (event)
    {
        case BTIF_CORE_STORAGE_REMOTE_READ:
        {
            char buf[1024];
            btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
            prop.type = p_req->read_req.type;
            prop.val = (void*) buf;
            prop.len = sizeof(buf);

            status = btif_storage_get_remote_device_property(&(p_req->read_req.bd_addr),
                                                             &prop);
            HAL_CBACK(bt_hal_cbacks, remote_device_properties_cb,
                            status, &(p_req->read_req.bd_addr), 1, &prop);
        }break;
        case BTIF_CORE_STORAGE_REMOTE_WRITE:
        {
           btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
           status = btif_storage_set_remote_device_property(&(p_req->write_req.bd_addr),
                                                            &(p_req->write_req.prop));
        }break;
        case BTIF_CORE_STORAGE_REMOTE_READ_ALL:
        {
           btif_storage_req_t *p_req = (btif_storage_req_t*)p_param;
           btif_in_get_remote_device_properties(&p_req->read_req.bd_addr);
        }break;
    }
}

void btif_adapter_properties_evt(bt_status_t status, uint32_t num_props,
                                    bt_property_t *p_props)
{
    HAL_CBACK(bt_hal_cbacks, adapter_properties_cb,
                     status, num_props, p_props);

}
void btif_remote_properties_evt(bt_status_t status, bt_bdaddr_t *remote_addr,
                                   uint32_t num_props, bt_property_t *p_props)
{
    HAL_CBACK(bt_hal_cbacks, remote_device_properties_cb,
                     status, remote_addr, num_props, p_props);
}

/*******************************************************************************
**
** Function         btif_in_storage_request_copy_cb
**
** Description     Switch context callback function to perform the deep copy for
**                 both the adapter and remote_device property API
**
** Returns          None
**
*******************************************************************************/
static void btif_in_storage_request_copy_cb(UINT16 event,
                                                 char *p_new_buf, char *p_old_buf)
{
     btif_storage_req_t *new_req = (btif_storage_req_t*)p_new_buf;
     btif_storage_req_t *old_req = (btif_storage_req_t*)p_old_buf;

     BTIF_TRACE_EVENT1("%s", __FUNCTION__);
     switch (event)
     {
         case BTIF_CORE_STORAGE_REMOTE_WRITE:
         case BTIF_CORE_STORAGE_ADAPTER_WRITE:
         {
             bdcpy(new_req->write_req.bd_addr.address, old_req->write_req.bd_addr.address);
             /* Copy the member variables one at a time */
             new_req->write_req.prop.type = old_req->write_req.prop.type;
             new_req->write_req.prop.len = old_req->write_req.prop.len;

             new_req->write_req.prop.val = (UINT8 *)(p_new_buf + sizeof(btif_storage_req_t));
             memcpy(new_req->write_req.prop.val, old_req->write_req.prop.val,
                    old_req->write_req.prop.len);
         }break;
     }
}

/*******************************************************************************
**
** Function         btif_get_adapter_properties
**
** Description      Fetch all available properties (local & remote)
**
** Returns          bt_status_t
**
*******************************************************************************/

bt_status_t btif_get_adapter_properties(void)
{
    BTIF_TRACE_EVENT1("%s", __FUNCTION__);

    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    return btif_transfer_context(execute_storage_request,
                                 BTIF_CORE_STORAGE_ADAPTER_READ_ALL,
                                 NULL, 0, NULL);
}

/*******************************************************************************
**
** Function         btif_get_adapter_property
**
** Description      Fetches property value from local cache
**
** Returns          bt_status_t
**
*******************************************************************************/

bt_status_t btif_get_adapter_property(bt_property_type_t type)
{
    btif_storage_req_t req;

    BTIF_TRACE_EVENT2("%s %d", __FUNCTION__, type);

    /* Allow get_adapter_property only for BDADDR and BDNAME if BT is disabled */
    if (!btif_is_enabled() && (type != BT_PROPERTY_BDADDR) && (type != BT_PROPERTY_BDNAME))
        return BT_STATUS_NOT_READY;

    memset(&(req.read_req.bd_addr), 0, sizeof(bt_bdaddr_t));
    req.read_req.type = type;

    return btif_transfer_context(execute_storage_request,
                                 BTIF_CORE_STORAGE_ADAPTER_READ,
                                (char*)&req, sizeof(btif_storage_req_t), NULL);
}

/*******************************************************************************
**
** Function         btif_set_adapter_property
**
** Description      Updates core stack with property value and stores it in
**                  local cache
**
** Returns          bt_status_t
**
*******************************************************************************/

bt_status_t btif_set_adapter_property(const bt_property_t *property)
{
    btif_storage_req_t req;
    bt_status_t status = BT_STATUS_SUCCESS;
    int storage_req_id = BTIF_CORE_STORAGE_NOTIFY_STATUS; /* default */
    char bd_name[BTM_MAX_LOC_BD_NAME_LEN +1];
    UINT16  name_len = 0;

    BTIF_TRACE_EVENT3("btif_set_adapter_property type: %d, len %d, 0x%x",
                      property->type, property->len, property->val);

     if (!btif_is_enabled())
         return BT_STATUS_NOT_READY;

    switch(property->type)
    {
        case BT_PROPERTY_BDNAME:
            {
                name_len = property->len > BTM_MAX_LOC_BD_NAME_LEN ? BTM_MAX_LOC_BD_NAME_LEN:
                                                                     property->len;
                memcpy(bd_name,property->val, name_len);
                bd_name[name_len] = '\0';

                BTIF_TRACE_EVENT1("set property name : %s", (char *)bd_name);

                BTA_DmSetDeviceName((char *)bd_name);

                storage_req_id = BTIF_CORE_STORAGE_ADAPTER_WRITE;
            }
            break;

        case BT_PROPERTY_ADAPTER_SCAN_MODE:
            {
                bt_scan_mode_t mode = *(bt_scan_mode_t*)property->val;
                tBTA_DM_DISC disc_mode;
                tBTA_DM_CONN conn_mode;

                switch(mode)
                {
                    case BT_SCAN_MODE_NONE:
                        disc_mode = BTA_DM_NON_DISC;
                        conn_mode = BTA_DM_NON_CONN;
                        break;

                    case BT_SCAN_MODE_CONNECTABLE:
                        disc_mode = BTA_DM_NON_DISC;
                        conn_mode = BTA_DM_CONN;
                        break;

                    case BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE:
                        disc_mode = BTA_DM_GENERAL_DISC;
                        conn_mode = BTA_DM_CONN;
                        break;

                    default:
                        BTIF_TRACE_ERROR1("invalid scan mode (0x%x)", mode);
                        return BT_STATUS_PARM_INVALID;
                }

                BTIF_TRACE_EVENT1("set property scan mode : %x", mode);

                BTA_DmSetVisibility(disc_mode, conn_mode, BTA_DM_IGNORE, BTA_DM_IGNORE);

                storage_req_id = BTIF_CORE_STORAGE_ADAPTER_WRITE;
            }
            break;
        case BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT:
            {
                /* Nothing to do beside store the value in NV.  Java
                   will change the SCAN_MODE property after setting timeout,
                   if required */
                storage_req_id = BTIF_CORE_STORAGE_ADAPTER_WRITE;
            }
            break;
        case BT_PROPERTY_BDADDR:
        case BT_PROPERTY_UUIDS:
        case BT_PROPERTY_ADAPTER_BONDED_DEVICES:
        case BT_PROPERTY_REMOTE_FRIENDLY_NAME:
            /* no write support through HAL, these properties are only populated from BTA events */
            status = BT_STATUS_FAIL;
            break;
        default:
            BTIF_TRACE_ERROR1("btif_get_adapter_property : invalid type %d",
            property->type);
            status = BT_STATUS_FAIL;
            break;
    }

    if (storage_req_id != BTIF_CORE_STORAGE_NO_ACTION)
    {
        int btif_status;
        /* pass on to storage for updating local database */

        memset(&(req.write_req.bd_addr), 0, sizeof(bt_bdaddr_t));
        memcpy(&(req.write_req.prop), property, sizeof(bt_property_t));

        return btif_transfer_context(execute_storage_request,
                                     storage_req_id,
                                     (char*)&req,
                                     sizeof(btif_storage_req_t)+property->len,
                                     btif_in_storage_request_copy_cb);
    }

    return status;

}

/*******************************************************************************
**
** Function         btif_get_remote_device_property
**
** Description      Fetches the remote device property from the NVRAM
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_get_remote_device_property(bt_bdaddr_t *remote_addr,
                                                 bt_property_type_t type)
{
    btif_storage_req_t req;

    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    memcpy(&(req.read_req.bd_addr), remote_addr, sizeof(bt_bdaddr_t));
    req.read_req.type = type;
    return btif_transfer_context(execute_storage_remote_request,
                                 BTIF_CORE_STORAGE_REMOTE_READ,
                                 (char*)&req, sizeof(btif_storage_req_t),
                                 NULL);
}

/*******************************************************************************
**
** Function         btif_get_remote_device_properties
**
** Description      Fetches all the remote device properties from NVRAM
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_get_remote_device_properties(bt_bdaddr_t *remote_addr)
{
    btif_storage_req_t req;

    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    memcpy(&(req.read_req.bd_addr), remote_addr, sizeof(bt_bdaddr_t));
    return btif_transfer_context(execute_storage_remote_request,
                                 BTIF_CORE_STORAGE_REMOTE_READ_ALL,
                                 (char*)&req, sizeof(btif_storage_req_t),
                                 NULL);
}

/*******************************************************************************
**
** Function         btif_set_remote_device_property
**
** Description      Writes the remote device property to NVRAM.
**                  Currently, BT_PROPERTY_REMOTE_FRIENDLY_NAME is the only
**                  remote device property that can be set
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_set_remote_device_property(bt_bdaddr_t *remote_addr,
                                                 const bt_property_t *property)
{
    btif_storage_req_t req;

     if (!btif_is_enabled())
         return BT_STATUS_NOT_READY;

    memcpy(&(req.write_req.bd_addr), remote_addr, sizeof(bt_bdaddr_t));
    memcpy(&(req.write_req.prop), property, sizeof(bt_property_t));

    return btif_transfer_context(execute_storage_remote_request,
                                 BTIF_CORE_STORAGE_REMOTE_WRITE,
                                 (char*)&req,
                                 sizeof(btif_storage_req_t)+property->len,
                                 btif_in_storage_request_copy_cb);
}


/*******************************************************************************
**
** Function         btif_get_remote_service_record
**
** Description      Looks up the service matching uuid on the remote device
**                  and fetches the SCN and service_name if the UUID is found
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_get_remote_service_record(bt_bdaddr_t *remote_addr,
                                               bt_uuid_t *uuid)
{
    if (!btif_is_enabled())
        return BT_STATUS_NOT_READY;

    return btif_dm_get_remote_service_record(remote_addr, uuid);
}


/*******************************************************************************
**
** Function         btif_get_enabled_services_mask
**
** Description      Fetches currently enabled services
**
** Returns          tBTA_SERVICE_MASK
**
*******************************************************************************/

tBTA_SERVICE_MASK btif_get_enabled_services_mask(void)
{
    return btif_enabled_services;
}

/*******************************************************************************
**
** Function         btif_enable_service
**
** Description      Enables the service 'service_ID' to the service_mask.
**                  Upon BT enable, BTIF core shall invoke the BTA APIs to
**                  enable the profiles
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_enable_service(tBTA_SERVICE_ID service_id)
{
    tBTA_SERVICE_ID *p_id = &service_id;

    /* If BT is enabled, we need to switch to BTIF context and trigger the
     * enable for that profile
     *
     * Otherwise, we just set the flag. On BT_Enable, the DM will trigger
     * enable for the profiles that have been enabled */

    btif_enabled_services |= (1 << service_id);

    BTIF_TRACE_ERROR2("%s: current services:0x%x", __FUNCTION__, btif_enabled_services);

    if (btif_is_enabled())
    {
        btif_transfer_context(btif_dm_execute_service_request,
                              BTIF_DM_ENABLE_SERVICE,
                              (char*)p_id, sizeof(tBTA_SERVICE_ID), NULL);
    }

    return BT_STATUS_SUCCESS;
}
/*******************************************************************************
**
** Function         btif_disable_service
**
** Description      Disables the service 'service_ID' to the service_mask.
**                  Upon BT disable, BTIF core shall invoke the BTA APIs to
**                  disable the profiles
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_disable_service(tBTA_SERVICE_ID service_id)
{
    tBTA_SERVICE_ID *p_id = &service_id;

    /* If BT is enabled, we need to switch to BTIF context and trigger the
     * disable for that profile so that the appropriate uuid_property_changed will
     * be triggerred. Otherwise, we just need to clear the service_id in the mask
     */

    btif_enabled_services &=  (tBTA_SERVICE_MASK)(~(1<<service_id));

    BTIF_TRACE_ERROR2("%s: Current Services:0x%x", __FUNCTION__, btif_enabled_services);

    if (btif_is_enabled())
    {
        btif_transfer_context(btif_dm_execute_service_request,
                              BTIF_DM_DISABLE_SERVICE,
                              (char*)p_id, sizeof(tBTA_SERVICE_ID), NULL);
    }

    return BT_STATUS_SUCCESS;
}

/*******************************************************************************
**
** Function         btif_config_hci_snoop_log
**
** Description      enable or disable HCI snoop log
**
** Returns          bt_status_t
**
*******************************************************************************/
bt_status_t btif_config_hci_snoop_log(uint8_t enable)
{
    bte_main_config_hci_logging(enable != 0,
             btif_core_state == BTIF_CORE_STATE_DISABLED);
    return BT_STATUS_SUCCESS;
}




/**
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
#define MASK_BTIF 0b10000000 // 0b 1000 0000
#define MASK_BTE  0b01000000 // 0b 0100 0000

#define setBTIF(zz) ( zz | MASK_BTIF )
#define setBTE(zz)  ( zz | MASK_BTE  )

#define isBTIF(vv) (( ( (vv&MASK_BTIF) >> 7)  && !( (vv&MASK_BTE )  >> 6) ) )
#define isBTE(ww)  (( ( (ww&MASK_BTE ) >> 6)  && !( (ww&MASK_BTIF)  >> 7) ) )

#define getBTIF(yy) ( yy & (~(MASK_BTE|MASK_BTIF)) )
#define getBTE(yy)  ( yy & (~(MASK_BTE|MASK_BTIF)) )

enum
{
    INIT,
    ENABLE,
    DISABLE,
    CLEANUP,
    GET_ADAPTER_PROPERTIES,
    GET_ADAPTER_PROPERTY,
    SET_ADAPTER_PROPERTY, // SET_ADAPTER_PROPERTY =  SET_ADAPTER_PROPERTY_BDNAME,
    GET_REMOTE_DEVICE_PROPERTIES,
    GET_REMOTE_DEVICE_PROPERTY,
    SET_REMOTE_DEVICE_PROPERTY,
    GET_REMOTE_SERVICE_RECORD,
    GET_REMOTE_SERVICES,
    START_DISCOVERY,
    CANCEL_DISCOVERY,
    CREATE_BOND,
    REMOVE_BOND,
    CANCEL_BOND,
    PIN_REPLY,
    SSP_REPLY,
    GET_PROFILE_INTERFACE,
    DUT_MODE_CONFIGURE,
    DUT_MODE_SEND,
    LE_TEST_MODE,
    CONFIG_HCI_SNOOP_LOG,
//... ... ...//
    SET_ADAPTER_PROPERTY_BDNAME,
    SET_ADAPTER_PROPERTY_SCAN_MODE_NONE,
    SET_ADAPTER_PROPERTY_SCAN_MODE_CONNECTABLE,
    SET_ADAPTER_PROPERTY_SCAN_MODE_CONNECTABLE_DISCOVERABLE,
    SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_2M,
    SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_5M,
    SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_1H,
    SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_NE
};

enum
{
        INQ_RES,
        BOND_STATE,
        DISC_RES
};

#define BTA_SERVICE_ID_TO_SERVICE_MASK(id)       (1 << (id))

typedef struct
{
    BD_ADDR         bda;                /* HID device bd address    */
    UINT8         status;             /* operation status         */
    UINT8           handle;             /* device handle            */
} tBTA_HH_DEV_INFO;

static void str2addr(BD_ADDR bd_addr, char* bb)
{
    int ii = 0;

    for(ii=5;ii>=0;ii--)
        bd_addr[ii] = (unsigned char) strtoul(bb, (char **)(bb+ii*2), 16);

    return ;
}

static void str2cod(DEV_CLASS dev_class, char* cc)
{
  char *pch = cc ;
  char *p   = (char *) malloc ( 2) ;
  int ii=0;

  while ( pch!=NULL && ii != 6)
  {
    strncpy(p, pch+ii*2, 2);
    dev_class[ii] = (unsigned char) strtoul(p, (char **)p, 16);
    ii++;
  }
    return ;
}

static void btd_to_btif(BT_HDR *p_msg){

    UINT8 cmd ;
    char bd_name[512] ;
    BD_ADDR bda={0x1, 0x1, 0x1, 0x1, 0x1, 0x1};
    UINT32 nameLen=0;
    int offHdr = 2;

    memset (bd_name, 0, BD_NAME_LEN);
    memset (bda, 0, BD_ADDR_LEN);

    bt_property_t *prop = (bt_property_t*) malloc (sizeof(bt_property_t));
    tBTA_DM_SEARCH *p_search_data = (tBTA_DM_SEARCH * )malloc (sizeof (tBTA_DM_SEARCH) ) ;

    DEV_CLASS dev_class = {0x20, 0x02, 0x0C};
    memcpy(p_search_data->inq_res.dev_class , dev_class , DEV_CLASS_LEN) ;

    memcpy ( &cmd , (UINT8 *)(p_msg+1) , 1);

    BTIF_TRACE_ERROR2("%s: cmd:%d", __FUNCTION__, cmd);

    memcpy ( (char *)bd_name , (UINT8 *)(p_msg+1) + p_msg->len + offHdr, p_msg->len - offHdr);
    BTIF_TRACE_ERROR1 ("btd_to_btif BDNAME: %s",   bd_name);

    char * pch = NULL;
    char dest_name[BD_NAME_LEN*3];
    char dest_addr[BD_ADDR_LEN*3];
    char dest_cod[DEV_CLASS_LEN*3];

    pch = strtok (bd_name,"#");
    pch = strtok (NULL, "#");


    UINT32 cod =  (UINT32 ) strtoul(pch, (char **)&pch, 16);
    uint2devclass( cod, dev_class);

    memcpy(p_search_data->inq_res.dev_class , dev_class , DEV_CLASS_LEN) ;
    pch = strtok (bd_name,"@");

    strcpy (dest_name, pch );
    nameLen = strlen(dest_name) ;

    pch = strtok (NULL, "@");
    strcpy (dest_addr, pch );

    int ii = 0;
    char  ps [512];
    for(ii=5;ii>=0;ii--){
        strncpy(ps,dest_addr+ii*2,2);
        bda[ii] = (UINT8) strtoul(ps, (char**)ps, 16);
    }

    BTIF_TRACE_ERROR3 ("btd_to_btif dev_class: %02x-%02x-%02x",   dev_class[0], dev_class[1], dev_class[2]);

    BTIF_TRACE_ERROR1 ("btd_to_btif BDNAME: %s",   bd_name);

    BTIF_TRACE_ERROR6 ("btd_to_btif bdaA: %02x-%02x-%02x-%02x-%02x-%02x",
            bda[0], bda[1], bda[2],
            bda[3], bda[4], bda[5]);

    if(isBTIF(cmd)){
        switch ( getBTIF(cmd) ){
            case INIT:
            {
                BTIF_TRACE_ERROR0("INIT");
            }
                break;
            case ENABLE:
            {
                BTIF_TRACE_ERROR0("ENABLE");
                btif_enable_bluetooth();
            }
                break;
            case DISABLE:
            {
                BTIF_TRACE_ERROR0("DISABLE");
                btif_disable_bluetooth();
                BTA_DisableBluetooth();
            }
                break;
            case CLEANUP:
            {
                BTIF_TRACE_ERROR0("CLEANUP");
            }
                break;
            case GET_ADAPTER_PROPERTIES:
            {
                BTIF_TRACE_ERROR0("GET_ADAPTER_PROPERTIES");
            }
                break;
            case GET_ADAPTER_PROPERTY:
            {
                BTIF_TRACE_ERROR0("GET_ADAPTER_PROPERTY");
            }
                break;
            case SET_ADAPTER_PROPERTY:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY");
                prop->type=BT_PROPERTY_BDNAME;
                prop->len = nameLen;
                prop->val = bd_name ;
                btif_set_adapter_property(prop);
            }
                break;
            case GET_REMOTE_DEVICE_PROPERTIES:
            {
                BTIF_TRACE_ERROR0("GET_REMOTE_DEVICE_PROPERTIES");
                btif_get_remote_device_properties((bt_bdaddr_t *)bda );
            }
                break;
            case GET_REMOTE_DEVICE_PROPERTY:
            {
                BTIF_TRACE_ERROR0("GET_REMOTE_DEVICE_PROPERTY");
            }
                break;
            case SET_REMOTE_DEVICE_PROPERTY:
            {
                BTIF_TRACE_ERROR0("SET_REMOTE_DEVICE_PROPERTY");
                prop->type=BT_PROPERTY_BDNAME;
                prop->len = nameLen;
                prop->val = bd_name ;
                btif_set_remote_device_property( (bt_bdaddr_t *)bda , prop);
            }
                break;
            case GET_REMOTE_SERVICE_RECORD:
            {
                BTIF_TRACE_ERROR0("GET_REMOTE_SERVICE_RECORD");
            }
                break;
            case GET_REMOTE_SERVICES:
            {
                BTIF_TRACE_ERROR0("GET_REMOTE_SERVICES");
                btif_dm_get_remote_services( (bt_bdaddr_t *)bda );
            }
                break;
            case START_DISCOVERY:
            {
                BTIF_TRACE_ERROR0("START_DISCOVERY");
                btif_dm_start_discovery();
            }
                break;
            case CANCEL_DISCOVERY:
            {
                BTIF_TRACE_ERROR0("CANCEL_DISCOVERY");
                btif_dm_cancel_discovery();
            }
                break;
            case CREATE_BOND:
            {
                BTIF_TRACE_ERROR0("CREATE_BOND");
                btif_dm_create_bond((bt_bdaddr_t *)bda ) ;
            }
                break;
            case REMOVE_BOND:
            {
                BTIF_TRACE_ERROR0("REMOVE_BOND");
                btif_dm_remove_bond((bt_bdaddr_t *)bda );
            }
                break;
            case CANCEL_BOND:
            {
                BTIF_TRACE_ERROR0("CANCEL_BOND");
                btif_dm_cancel_bond((bt_bdaddr_t *)bda );
            }
                break;
            case PIN_REPLY:
            {
                BTIF_TRACE_ERROR0("INIT");
            }
                break;
            case SSP_REPLY:
            {
                BTIF_TRACE_ERROR0("SSP_REPLY");
            }
                break;
            case GET_PROFILE_INTERFACE:
            {
                BTIF_TRACE_ERROR0("GET_PROFILE_INTERFACE");
            }
                break;
            case DUT_MODE_CONFIGURE:
            {
                BTIF_TRACE_ERROR0("DUT_MODE_CONFIGURE");
            }
                break;
            case DUT_MODE_SEND:
            {
                BTIF_TRACE_ERROR0("DUT_MODE_SEND");
            }
                break;
            case LE_TEST_MODE:
            {
                BTIF_TRACE_ERROR0("LE_TEST_MODE");
            }
                break;
            case CONFIG_HCI_SNOOP_LOG:
            {
                BTIF_TRACE_ERROR0("CONFIG_HCI_SNOOP_LOG");
            }
                break;
//... ... ...//
            case SET_ADAPTER_PROPERTY_BDNAME:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY_BDNAME");
                prop->type=BT_PROPERTY_BDNAME;
                prop->len = nameLen;
                prop->val = bd_name ;
                btif_set_adapter_property(prop);
            }
                break;
            case SET_ADAPTER_PROPERTY_SCAN_MODE_NONE:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY_SCAN_MODE_NONE");
                prop->type=BT_PROPERTY_ADAPTER_SCAN_MODE;
                bt_scan_mode_t mode = BT_SCAN_MODE_NONE;
                prop->val=(void*)(&mode);
                prop->len = sizeof(bt_scan_mode_t);
                btif_set_adapter_property(prop);
            }
                break;
            case SET_ADAPTER_PROPERTY_SCAN_MODE_CONNECTABLE:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY_SCAN_MODE_CONNECTABLE");
                prop->type=BT_PROPERTY_ADAPTER_SCAN_MODE;
                bt_scan_mode_t mode = BT_SCAN_MODE_CONNECTABLE;
                prop->val=(void*)(&mode);
                prop->len = sizeof(bt_scan_mode_t);
                btif_set_adapter_property(prop);
            }
                break;
            case SET_ADAPTER_PROPERTY_SCAN_MODE_CONNECTABLE_DISCOVERABLE:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY_SCAN_MODE_CONNECTABLE_DISCOVERABLE");
                prop->type=BT_PROPERTY_ADAPTER_SCAN_MODE;
                bt_scan_mode_t mode = BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE;
                prop->val=(void*)(&mode);
                prop->len = sizeof(bt_scan_mode_t);
                btif_set_adapter_property(prop);
            }
                break;
            case SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_2M:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_2M");
                prop->type=BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT;
                int mode = 120;
                prop->val=(void*)(&mode);
                prop->len=(int)sizeof(int);
                btif_set_adapter_property(prop);
            }
                break;
            case SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_5M:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_5M");
                prop->type=BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT;
                int mode = 300;
                prop->val=(void*)(&mode);
                prop->len=(int)sizeof(int);
                btif_set_adapter_property(prop);
            }
                break;
            case SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_1H:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_1H");
                prop->type=BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT;
                int mode = 3600;
                prop->val=(void*)(&mode);
                prop->len=(int)sizeof(int);
                btif_set_adapter_property(prop);
            }
                break;
            case SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_NE:
            {
                BTIF_TRACE_ERROR0("SET_ADAPTER_PROPERTY_DISCOVERY_TIMEOUT_NE");
                prop->type=BT_PROPERTY_ADAPTER_DISCOVERY_TIMEOUT;
                int mode = 0;
                prop->val=(void*)(&mode);
                prop->len=(int)sizeof(int);
                btif_set_adapter_property(prop);
            }
                break;
            default:
            {
                BTIF_TRACE_ERROR0("default");
            }
                break;
        }
    }else if(isBTE(cmd)){
        switch ( getBTE(cmd) ){
            case INQ_RES:
            {
                BTIF_TRACE_ERROR0("INQ_RES");
                bdcpy(p_search_data->inq_res.bd_addr, bda);
                bte_dm_aic_evt( (tBTA_DM_SEARCH_EVT) BTA_DM_INQ_RES_EVT, p_search_data) ;
            }
                break;
            case BOND_STATE:
            {
                BTIF_TRACE_ERROR0("BOND_STATE");
                bte_dm_aic_evt_3(bda);
            }
                break;
            case DISC_RES:
            {
                BTIF_TRACE_ERROR0("DISC_RES");
                UINT32 num_uuids = 1;
                bdcpy(p_search_data->disc_res.bd_addr, bda);
                p_search_data->disc_res.p_uuid_list = (UINT8*)GKI_getbuf(num_uuids*MAX_UUID_SIZE);
                p_search_data->disc_res.num_uuids = num_uuids;
                p_search_data->disc_res.services |= (tBTA_SERVICE_MASK)(BTA_SERVICE_ID_TO_SERVICE_MASK(BTA_HSP_SERVICE_ID)) ;
                p_search_data->disc_res.result = BTA_SUCCESS;
                bte_dm_aic_evt_2( (tBTA_DM_SEARCH_EVT) BTA_DM_DISC_RES_EVT, p_search_data) ;
            }
                break;
            default:
            {
                BTIF_TRACE_ERROR0("default");
            }
                break;
        }
    }
}
