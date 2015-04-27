#ifndef	__USR_EWPT_H__
#define	__USR_EWPT_H__

#include "app_env.h"
#include "uart.h"

#define EVENT_UART_TX_ID 3
enum com_st
{
    COM_DEEPSLEEP	= 0,
    COM_ADV			= 1,
    COM_CONN_EMPTY   = 2,
    COM_CONN_FULL	= 3,
    
    COM_UART_TX_IDLE = 4,
    COM_UART_TX_ONGOING=5
};
struct com_env_tag
{
    uint8_t com_state ;
    
    ///Message id
    uint8_t msg_id;
    
    ///UART TX parameter 
    uint8_t tx_state;       //either transmitting or done.
    struct co_list queue_tx;///Queue of kernel messages corresponding to packets sent through HCI
    
};

extern  struct com_env_tag  com_env;

extern void com_pdu_send(uint8_t len, uint8_t *par);
extern void com_init(void);
extern void com_tx_done(void);
#endif
/// end of usr_ewpt.h
