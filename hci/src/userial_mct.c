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

/******************************************************************************
 *
 *  Filename:      userial_mct.c
 *
 *  Description:   Contains open/read/write/close functions on multi-channels
 *
 ******************************************************************************/
#define LOG_TAG "bt_userial_mct"

#include <utils/Log.h>
#include <pthread.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <sys/socket.h>
#include "bt_hci_bdroid.h"
#include "userial.h"
#include "utils.h"
#include "vendor.h"
#include "bt_vendor_lib.h"
#include "bt_utils.h"

#include <strings.h>

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef USERIAL_DBG
#define USERIAL_DBG TRUE
#endif

#if (USERIAL_DBG == TRUE)
#define USERIALDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define USERIALDBG(param, ...) {}
#endif

#define MAX_SERIAL_PORT (USERIAL_PORT_3 + 1)


#define READ_LIMIT 1024
#define UBTDPORT  33800

enum {
    USERIAL_RX_EXIT,
};

/******************************************************************************
**  Externs
******************************************************************************/
uint16_t hci_mct_receive_evt_msg(void);
uint16_t hci_mct_receive_acl_msg(void);


/******************************************************************************
**  Local type definitions
******************************************************************************/

typedef struct
{
    int             fd[CH_MAX];
    uint8_t         port;
    pthread_t       read_thread;
    BUFFER_Q        rx_q;
    HC_BT_HDR      *p_rx_hdr;
} tUSERIAL_CB;

/******************************************************************************
**  Static variables
******************************************************************************/

static tUSERIAL_CB userial_cb;
static volatile uint8_t userial_running = 0;

/******************************************************************************
**  Static functions
******************************************************************************/

/*****************************************************************************
**   Socket signal functions to wake up userial_read_thread for termination
**
**   creating an unnamed pair of connected sockets
**      - signal_fds[0]: join fd_set in select call of userial_read_thread
**      - signal_fds[1]: trigger from userial_close
*****************************************************************************/
static int signal_fds[2]={0,1};
static inline int create_signal_fds(fd_set* set)
{
    if(signal_fds[0]==0 && socketpair(AF_UNIX, SOCK_STREAM, 0, signal_fds)<0)
    {
        ALOGE("create_signal_sockets:socketpair failed, errno: %d", errno);
        return -1;
    }
    FD_SET(signal_fds[0], set);
    return signal_fds[0];
}
static inline int send_wakeup_signal(char sig_cmd)
{
    return send(signal_fds[1], &sig_cmd, sizeof(sig_cmd), 0);
}
static inline char reset_signal()
{
    char sig_recv = -1;
    recv(signal_fds[0], &sig_recv, sizeof(sig_recv), MSG_WAITALL);
    return sig_recv;
}
static inline int is_signaled(fd_set* set)
{
    return FD_ISSET(signal_fds[0], set);
}

/*******************************************************************************
**
** Function        userial_evt_read_thread
**
** Description     The reading thread on EVT and ACL_IN channels
**
** Returns         void *
**
*******************************************************************************/
 static int start_server(int port) {
    int server = -1;
    struct sockaddr_in srv_addr;
    long haddr;

    bzero(&srv_addr, sizeof(srv_addr));
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = INADDR_ANY;
    srv_addr.sin_port = htons(port);

    if ((server = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ALOGE(" Start_server : batteryUnable to create socket\n");
        return -1;
    }

    int yes = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));

    if (bind(server, (struct sockaddr *)&srv_addr, sizeof(srv_addr)) < 0) {
        ALOGE(" Start_server : batteryUnable to bind socket, errno=%d\n", errno);
        return -1;
    }

    return server;
}

static int wait_for_client(int server) {
    int client = -1;

    if (listen(server, 1) < 0) {
        SLOGE("Unable to listen to socket, errno=%d\n", errno);
        return -1;
    }

    SLOGE("wait_for_client\n");
    client = accept(server, NULL, 0);

    if (client < 0) {
        SLOGE("Unable to accept socket for main conection, errno=%d\n", errno);
        return -1;
    }

    return client;
}

static void *userial_read_thread(void *arg)
{
    int rx_length = 0;

    int server = -1;
    int client = -1;


    USERIALDBG( "start userial_read_thread");
    raise_priority_a2dp(TASK_HIGH_BTU);
    //_timeout = POLL_TIMEOUT;

    if ((server = start_server(UBTDPORT)) == -1) {
        USERIALDBG("userial_read_thread : start_server unable to create socket\n");
        return NULL;
    }

   // Listen for main connection
    while ((client = wait_for_client(server)) != -1)
    {
        //char current_packet[READ_LIMIT]
        void *buf = (void *) malloc (READ_LIMIT * sizeof(unsigned char));
        memset(buf, 0, READ_LIMIT);

        userial_cb.fd[0] = client ;
        userial_cb.port=UBTDPORT;
        if((rx_length = recv(client, buf, 1, MSG_PEEK))== -1)
        {
            USERIALDBG("userial_read_thread::  Error receiving data  !!! ");
        }else{
            USERIALDBG("userial_read_thread::  rx_length %d ", rx_length);
            if (rx_length > 0){
                if ( *((char *)(buf)) == -1 )
                {
                    hci_mct_receive_acl_msg();
                }else if ( *((char *)(buf)) == -2 )
                {
                    hci_mct_receive_evt_msg();
                }else{
                    hci_mct_receive_evt_msg_aic(MSG_HC_TO_BTIF_HCI_EVT);
                }
            }
        }
    } /* end while */

    pthread_exit(NULL);
    return NULL;
}

/*****************************************************************************
**   Userial API Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        userial_init
**
** Description     Initializes the userial driver
**
*******************************************************************************/
bool userial_init(void)
{
    int idx;

    USERIALDBG("userial_init");
    memset(&userial_cb, 0, sizeof(tUSERIAL_CB));
    for (idx=0; idx < CH_MAX; idx++)
        userial_cb.fd[idx] = -1;
    utils_queue_init(&(userial_cb.rx_q));
    return true;
}


/*******************************************************************************
**
** Function        userial_open
**
** Description     Open Bluetooth device with the port ID
**
*******************************************************************************/
bool userial_open(userial_port_t port)
{
    int result;

    USERIALDBG("userial_open(port:%d)", port);

    if (userial_running)
    {
        /* Userial is open; close it first */
        userial_close();
        utils_delay(50);
    }

    if (port >= MAX_SERIAL_PORT)
    {
        ALOGE("Port > MAX_SERIAL_PORT");
        return false;
    }
/*
    result = vendor_send_command(BT_VND_OP_USERIAL_OPEN, &userial_cb.fd);
    if ((result != 2) && (result != 4))
    {
        ALOGE("userial_open: wrong numbers of open fd in vendor lib [%d]!",
                result);
        ALOGE("userial_open: HCI MCT expects 2 or 4 open file descriptors");
        vendor_send_command(BT_VND_OP_USERIAL_CLOSE, NULL);
        return false;
    }

    ALOGI("CMD=%d, EVT=%d, ACL_Out=%d, ACL_In=%d", \
        userial_cb.fd[CH_CMD], userial_cb.fd[CH_EVT], \
        userial_cb.fd[CH_ACL_OUT], userial_cb.fd[CH_ACL_IN]);

    if ((userial_cb.fd[CH_CMD] == -1) || (userial_cb.fd[CH_EVT] == -1) ||
        (userial_cb.fd[CH_ACL_OUT] == -1) || (userial_cb.fd[CH_ACL_IN] == -1))
    {
        ALOGE("userial_open: failed to open BT transport");
        vendor_send_command(BT_VND_OP_USERIAL_CLOSE, NULL);
        return false;
    }
*/

    userial_cb.port = port;

    /* Start listening thread */
    if (pthread_create(&userial_cb.read_thread, NULL, userial_read_thread, NULL) != 0)
    {
        ALOGE("pthread_create failed!");
        vendor_send_command(BT_VND_OP_USERIAL_CLOSE, NULL);
        return false;
    }

    return true;
}

/*******************************************************************************
**
** Function        userial_read
**
** Description     Read data from the userial channel
**
** Returns         Number of bytes actually read from the userial port and
**                 copied into p_data.  This may be less than len.
**
*******************************************************************************/
uint16_t userial_read(uint16_t msg_id, uint8_t *p_buffer, uint16_t len)
{
    int ret = -1;
    // int ch_idx = (msg_id == MSG_HC_TO_STACK_HCI_EVT) ? CH_EVT : CH_ACL_IN;
    int ch_idx =0;

    ALOGW( "userial_read: read() reading %d !", len);


    ret = read(userial_cb.fd[ch_idx], p_buffer, (size_t)len);
    if (ret <= 0)
        ALOGW( "userial_read: read() returned %d!", ret);

    return (uint16_t) ((ret >= 0) ? ret : 0);
}

/*******************************************************************************
**
** Function        userial_write
**
** Description     Write data to the userial port
**
** Returns         Number of bytes actually written to the userial port. This
**                 may be less than len.
**
*******************************************************************************/
uint16_t userial_write(uint16_t msg_id, const uint8_t *p_data, uint16_t len)
{
    int ret, total = 0;
    int ch_idx = (msg_id == MSG_STACK_TO_HC_HCI_CMD) ? CH_CMD : CH_ACL_OUT;

    while(len != 0)
    {
        ret = write(userial_cb.fd[ch_idx], p_data+total, len);
        total += ret;
        len -= ret;
    }

    return ((uint16_t)total);
}

void userial_close_reader(void) {
    // Join the reader thread if it is still running.
    if (userial_running) {
        send_wakeup_signal(USERIAL_RX_EXIT);
        int result = pthread_join(userial_cb.read_thread, NULL);
        USERIALDBG("%s Joined userial reader thread: %d", __func__, result);
        if (result)
            ALOGE("%s failed to join reader thread: %d", __func__, result);
        return;
    }
    ALOGW("%s Already closed userial reader thread", __func__);
}

/*******************************************************************************
**
** Function        userial_close
**
** Description     Close the userial port
**
*******************************************************************************/
void userial_close(void)
{
    int idx, result;

    USERIALDBG("userial_close");

    if (userial_running)
        send_wakeup_signal(USERIAL_RX_EXIT);

    if ((result=pthread_join(userial_cb.read_thread, NULL)) < 0)
        ALOGE( "pthread_join() FAILED result:%d", result);

    vendor_send_command(BT_VND_OP_USERIAL_CLOSE, NULL);

    for (idx=0; idx < CH_MAX; idx++)
        userial_cb.fd[idx] = -1;
}
