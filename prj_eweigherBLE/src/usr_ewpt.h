#ifndef	__USR_EWPT_H__
#define	__USR_EWPT_H__

#include "app_env.h"
#include "uart.h"

//the notify char number
#define QPPS_VAL_CHAR_NUM	(2)

/*no need
#if defined(EWPT_V2_0)
#define	COM_WAKEUP_TRIGGER		(GPIO_P13)
#elif defined(EWPT_V3_0)
#define	COM_WAKEUP_TRIGGER		(GPIO_P31)
#endif
*/

//#define	COM_WAKEUP						(GPIO_P03)
#define	SCALE_STATUS_PIN				(GPIO_P31)			//Power On:High          Power Off:Low
#define	SCALE_WAKEUP_PIN					(GPIO_P12)

#define EVENT_COM_WAKEUP_ID			1

#define EVENT_UART_TX_ID 3
#define EVENT_UART_RX_FRAME_ID 6
#define EVENT_UART_RX_TIMEOUT_ID 5
#define	EVENT_SCALE_POWER_ON_ID		7
#define	EVENT_SCALE_POWER_OFF_ID		8


enum com_st
{
    COM_IDLE	= 0,
    COM_TRAN	= 1,
		COM_BUSY  = 2,
    
    COM_UART_TX_IDLE = 4,
    COM_UART_TX_ONGOING=5
};

enum	scale_err_code
{
  SCALE_NO_ERR = 0x0,
	SCALE_POWER_OFF,
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
		SCALE_POWER_ON = 0x00,
		SCALE_POWER_DOWN,
		SCALE_QPPS_NTF_ON = 0x02,
		SCALE_QPPS_NTF_NOT_ON,
		SCALE_QPPS_DATA_SEND = 0x05,
		SCALE_XOR_SUM_ERR = 0x07,
		SCALE_COM_RX_LENGTH_ERR  = 0x0a,
		SCALE_COM_RX_FIRST_BYTE_ERR = 0x10,
		SCALE_COM_RX_SECOND_BYTE_ERR = 0x11,
		SCALE_COM_LAST_BYTE_ERR = 0x12,
		SCALE_COM_CRC_CHECKOUT_ERR = 0x1a,
		SCALE_COM_RESP_NO_ERR = 0x20,
		SCALE_COM_RESP_ERR,
		SCALE_POWER_BUSY = 0x2a,
		SCALE_ERR_NB
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
		uint8_t result_st;
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
extern enum scale_st get_scale_status(void);


#endif
/// end of usr_ewpt.h
