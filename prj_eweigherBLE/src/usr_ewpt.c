#include	"usr_ewpt.h"
#include	"lib.h"

#if		(BLE_EWPT_SERVER)

struct com_env_tag  com_env;


 
void com_init(void)
{
    com_env.com_state = COM_DEEPSLEEP;

    //for com uart tx  
    com_env.tx_state = COM_UART_TX_IDLE;	//initialize tx state
    co_list_init(&com_env.queue_tx);			//init TX queue
    
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_TX_ID, com_tx_done))
    ASSERT_ERR(0);
}


void app_event_com_tx_handler(void)
{
	ke_evt_set(1UL<<EVENT_UART_TX_ID);
}


void com_uart_write(struct ke_msg *msg)
{
    //go to start tx state
    com_env.tx_state = COM_UART_TX_ONGOING;
		
    uart_write(EWPT_COM_UART, ((uint8_t *)&msg->param), msg->param_len, app_event_com_tx_handler);
		//delay(0x1fff);
}


// Push msg into eaci tx queue
static void com_push(struct ke_msg *msg)
{
    // Push the message into the list of messages pending for transmission
    co_list_push_back(&com_env.queue_tx, &msg->hdr);

    // Check if there is no transmission ongoing
    if (com_env.tx_state == COM_UART_TX_IDLE)
        // Forward the message to the HCI UART for immediate transmission
        com_uart_write(msg);
}


/**
 ****************************************************************************************
 * @brief EACI send PDU
 *
 ****************************************************************************************
 */
void com_pdu_send(uint8_t len, uint8_t *par)
{
    // Allocate one msg for EACI tx
    uint8_t *msg_param = (uint8_t*)ke_msg_alloc(0, 0, 0, len);

    // Save the PDU in the MSG
    memcpy(msg_param, par, len);

     //extract the ke_msg pointer from the param passed and push it in HCI queue
    com_push(ke_param2msg(msg_param));
}

 
 /**
 ****************************************************************************************
 * @brief After-process when one PDU has been sent.
 *
 ****************************************************************************************
 */
void com_tx_done(void)
{
    struct ke_msg * msg;
    // Clear the event
    ke_evt_clear(1<<EVENT_UART_TX_ID);
    // Go back to IDLE state
    com_env.tx_state = COM_UART_TX_IDLE;
    //release current message (which was just sent)
    msg = (struct ke_msg *)co_list_pop_front(&com_env.queue_tx);
    // Free the kernel message space
    ke_msg_free(msg);
    // Check if there is a new message pending for transmission
    if ((msg = (struct ke_msg *)co_list_pick(&com_env.queue_tx)) != NULL)
    {
        // Forward the message to the HCI UART for immediate transmission
        com_uart_write(msg);
    }
}

#endif
/// end of usr_ewpt.c
