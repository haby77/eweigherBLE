#ifndef	__USR_EWPT_H__
#define	__USR_EWPT_H__

#include "app_env.h"
#include "uart.h"

#define QPPS_VAL_CHAR_NUM	(2)

#define	COM_WAKEUP_TRIGGER		(GPIO_P13)
#define	COM_WAKEUP						(GPIO_P03)
#define	SCALE_WAKEUP					(GPIO_P12)

#define EVENT_COM_WAKEUP_ID			1

#define EVENT_UART_TX_ID 3
#define EVENT_UART_RX_FRAME_ID 6
#define EVENT_UART_RX_TIMEOUT_ID 5
#define	EVENT_SCALE_POWER_ON_ID		7
#define	EVENT_SCALE_POWER_OFF_ID		8


enum com_st
{
    COM_DEEPSLEEP	= 0,
    COM_ADV			= 1,
    COM_CONN_EMPTY   = 2,
    COM_CONN_FULL	= 3,
    
    COM_UART_TX_IDLE = 4,
    COM_UART_TX_ONGOING=5
};

enum com_error_st
{
		error_in_firstbyte = 0,
		error_in_secondbyte,
		lastbyte_sended,
		no_error
};

enum scale_st
{
		power_on = 0,
		power_down,
		work_busy
};

typedef struct user_data
{
		uint8_t update_flag;
		uint8_t len;
		uint8_t data[10];
} usr_data;

struct com_env_tag
{
    uint8_t com_state ;
    
    ///Message id
    uint8_t msg_id;
    
    ///UART TX parameter 
    uint8_t tx_state;       //either transmitting or done.
    struct co_list queue_tx;///Queue of kernel messages corresponding to packets sent through HCI
    
    ///UART RX parameter 
    uint8_t com_rx_len;
    uint8_t com_rx_buf[QPPS_VAL_CHAR_NUM*QPP_DATA_MAX_LEN];
		uint8_t scale_state;
		usr_data scale_user_data;

};

extern  struct com_env_tag  com_env;

extern void com_pdu_send(uint8_t len, uint8_t *par);
extern void app_event_com_wakeup_handler(void);
extern void com_init(void);
extern void com_tx_done(void);
extern void com_wakeup_cb(void);
extern int app_com_scale_wakeup_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int app_com_uart_rx_done_ind_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int app_com_rx_timeout_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int app_scale_power_on_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern int app_scale_power_off_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id);
extern void	com_uart_rx_start(void);
extern void com_event_uart_rx_frame_handler(void);
extern void com_event_uart_rx_timeout_handler(void);
extern void scale_event_power_on_handler(void);
extern void scale_event_power_off_handler(void);
void com_uart_rx(void);
extern void	wakeup_scale(void);

#endif
/// end of usr_ewpt.h
