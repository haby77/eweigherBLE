#include	"usr_ewpt.h"
#include	"lib.h"
#include "app_sys.h"

#if		(BLE_EWPT_SERVER)

#define COM_FRAME_TIMEOUT 2 //2*10ms

struct com_env_tag  com_env;

void com_gpio_init(void)
{
		gpio_set_direction(COM_WAKEUP_TRIGGER,GPIO_OUTPUT);
		gpio_write_pin(COM_WAKEUP_TRIGGER,GPIO_HIGH);
	
		gpio_wakeup_config(COM_WAKEUP,GPIO_WKUP_BY_LOW);
		gpio_enable_interrupt(COM_WAKEUP);
	
		gpio_set_direction(SCALE_WAKEUP,GPIO_OUTPUT);
		gpio_write_pin(SCALE_WAKEUP,GPIO_HIGH);
	
		if(KE_EVENT_OK != ke_evt_callback_set(EVENT_COM_WAKEUP_ID, 
                                            app_event_com_wakeup_handler))
    {
        ASSERT_ERR(0);
    }
	
}
 
void com_init(void)
{
    com_env.com_state = COM_DEEPSLEEP;

    //for com uart tx  
    com_env.tx_state = COM_UART_TX_IDLE;	//initialize tx state
		com_env.scale_state = power_down;
		com_env.result_st = 0x0;
		com_env.scale_user_data.update_flag = 1;
    co_list_init(&com_env.queue_tx);			//init TX queue

#if LED_DEBUG	
		debug_LED_init();
#endif
		com_gpio_init();
     
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_TX_ID, com_tx_done))
    ASSERT_ERR(0);
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_RX_FRAME_ID, com_event_uart_rx_frame_handler))
    ASSERT_ERR(0);
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_UART_RX_TIMEOUT_ID, com_event_uart_rx_timeout_handler))
    ASSERT_ERR(0);
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_SCALE_POWER_ON_ID, scale_event_power_on_handler))
    ASSERT_ERR(0);
    if(KE_EVENT_OK != ke_evt_callback_set(EVENT_SCALE_POWER_OFF_ID, scale_event_power_off_handler))
    ASSERT_ERR(0);

}

void com_wakeup_cb(void)
{
	    // If BLE is in the sleep mode, wakeup it.
    if(ble_ext_wakeup_allow())
    {
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
        if (sleep_env.deep_sleep)
        {
            wakeup_32k_xtal_switch_clk();
        }
#endif

        sw_wakeup_ble_hw();

    }
    // key debounce:
    // We can set a soft timer to debounce.
    // After wakeup BLE, the timer is not calibrated immediately and it is not precise.
    // So We set a event, in the event handle, set the soft timer.
    ke_evt_set(1UL << EVENT_COM_WAKEUP_ID);
}

void	wakeup_scale(void)
{
		gpio_write_pin(SCALE_WAKEUP,GPIO_LOW);
		ke_timer_set(APP_COM_SCALE_WAKEUP_TIMER,TASK_APP,10);
}

int app_com_scale_wakeup_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
		gpio_write_pin(SCALE_WAKEUP,GPIO_HIGH);
		ke_timer_clear(APP_COM_SCALE_WAKEUP_TIMER,TASK_APP);
		return(KE_MSG_CONSUMED);
}


void	app_com_wakeup_process(void)
{
	if(gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
	{		
		switch(com_env.com_state)
		{
			case COM_DEEPSLEEP:
			{
			}break;
			case COM_ADV:
			{

			}break;
			case COM_CONN_EMPTY:
			{
				wakeup_scale();
				//com_uart_rx_start();
				QPRINTF("wake up!\r\n");
			}break;
			case COM_CONN_FULL:
			{

			}break;
							
			default :break;
		}
	}
}

/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_com_wakeup_handler(void)
{
    // delay 20ms to debounce
    ke_evt_clear(1UL << EVENT_COM_WAKEUP_ID);
	
#if (defined (CFG_PT_BOTTON))
	 ke_timer_set(APP_SYS_GPIO_TXWAKEUP_TIMER, TASK_APP, 2);
#else
	 app_com_wakeup_process();
#endif
}


void app_event_com_tx_handler(void)
{
	ke_evt_set(1UL<<EVENT_UART_TX_ID);
}

void	com_uart_rx_start(void)
{
		//QPRINTF("com_uart_rx_start\r\n");
    com_env.com_rx_len = 0;
    uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
}

void com_uart_rx(void)
{
		//QPRINTF("Here!\r\n");
		com_env.com_rx_len++;
		com_env.com_state = COM_CONN_FULL;
		if (com_env.com_rx_len > 1 && com_env.com_rx_len < 40)
		{
				if ((com_env.com_rx_len == 6) && (com_env.com_rx_buf[0] == 0x02) && (com_env.com_rx_buf[1] == 0x6f) && (com_env.com_rx_buf[2] == 0x66) && (com_env.com_rx_buf[3] == 0x66) && (com_env.com_rx_buf[4] == 0x03) && (com_env.com_rx_buf[com_env.com_rx_len - 1] == 0xE0))
				{
							com_env.scale_state = power_down;
							com_env.result_st = 0x0;
							QPRINTF("power off!\r\n");			
				}
				else
				{
						if (com_env.com_rx_buf[com_env.com_rx_len - 1] == 0x03)
						{
								if(com_env.com_rx_len == 4 && com_env.com_rx_buf[1] == 0x6f && com_env.com_rx_buf[2] == 0x6e)
								{
									com_env.scale_state = power_on;
									//ke_timer_set(APP_SCALE_POWER_ON_TIMER,TASK_APP,2);
									QPRINTF("power on!\r\n");
								}
								if(com_env.com_rx_len == 5 && com_env.com_rx_buf[1] == 0x6f 
									&& com_env.com_rx_buf[2] == 0x66 && com_env.com_rx_buf[3] == 0x66)
								{
										ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
										uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);	
								}
								if(com_env.com_rx_buf[1] == 0x40)
								{
										if (com_env.com_rx_len == 36)
										{
												uint8_t i,CRC_COM = 0;
												for (i = 0;i<(com_env.com_rx_len-2);i++)
															CRC_COM ^= com_env.com_rx_buf[i];
												if (CRC_COM == com_env.com_rx_buf[com_env.com_rx_len-2])
												{
														QPRINTF("\r\ndata correct!\r\n");
														ke_evt_clear(1UL<<EVENT_UART_RX_TIMEOUT_ID);
														QPRINTF("com_env.result_st %d\r\n",com_env.result_st);
														ke_evt_set(1UL << EVENT_UART_RX_FRAME_ID);
														uint8_t temp_array[10];
														temp_array[0] = 0x02;
														temp_array[1] = 0x00;
														temp_array[2] = 0x30;
														temp_array[3] = 0x31;
														temp_array[4] = 0x37;
														temp_array[5] = 0x30;
														temp_array[6] = 0x33;
														temp_array[7] = 0x30;
														temp_array[8] = temp_array[0]^temp_array[1]^temp_array[2]^temp_array[3]^temp_array[4]^temp_array[5]^temp_array[6]^temp_array[7];
														temp_array[9] = 0x03;
														com_pdu_send(10,&(temp_array[0]));
												}
												else
												{
														QPRINTF("CRC verify error!\r\n");
														if (gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
																com_uart_rx_start();
												}
										}
										else
										{
												ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
												uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);								
										}
										
								}
								if (com_env.com_rx_buf[1] == 0x53)
								{
										if (com_env.com_rx_len == 11)
										{
												uint8_t i,CRC_COM = 0;
												for (i = 0;i<(com_env.com_rx_len-2);i++)
															CRC_COM ^= com_env.com_rx_buf[i];
												if (CRC_COM == com_env.com_rx_buf[com_env.com_rx_len-2])
												{
														QPRINTF("\r\ndata correct!\r\n");
														//app_qpps_data_send(app_qpps_env->conhdl,0,(com_env.com_rx_len-1),com_env.com_rx_buf);
														com_env.scale_user_data.update_flag = 1;
														ke_evt_clear(1UL<<EVENT_UART_RX_TIMEOUT_ID);
														ke_evt_set(1UL << EVENT_UART_RX_FRAME_ID);
												}
												else
												{
														QPRINTF("CRC verify error!\r\n");
														uint8_t temp_error[] = "set usr_data error!";
														app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(temp_error),temp_error);
														if (gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
																com_uart_rx_start();
												}
										}
										else
										{
												ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
												uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);								
										}
								}
						}
						else
						{							
								//QPRINTF("xor_sum error!\r\n");
								if (gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
								{
										ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
										uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
								}
						}
					}
//				else
//				{
//						QPRINTF("CRC verify error!\r\n");
//						if (gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
//						com_uart_rx_start();
//				}
		}
		else
		{
				if (com_env.com_rx_len == 1)
				{
						if(com_env.com_rx_buf[0] == 0x02)
						{
								if (gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
								{
										ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
										uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
								}
						}
						else
						{
								QPRINTF("first byte error! 0x%X\r\n",com_env.com_rx_buf[0]);
								if (gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
									com_uart_rx_start();
						}
				}
				else
				{
						QPRINTF("length error!\r\n");
						if (gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
							com_uart_rx_start();
				}
		}
//	  if(com_env.com_rx_len==(QPPS_VAL_CHAR_NUM + 1)*QPP_DATA_MAX_LEN)  //receive data buf is full, should sent them to ble
//    {
//				ke_evt_set(1UL << EVENT_UART_RX_FRAME_ID);
//				QPRINTF("full!\r\n");
//			///leo test
//				com_uart_rx_start();
//				com_env.com_state = COM_CONN_EMPTY;
//			///leo test end
//    }
//    else
//    {
//					ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
//        	uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
//    }
}

void com_uart_write(struct ke_msg *msg)
{
    //go to start tx state
    com_env.tx_state = COM_UART_TX_ONGOING;
		
    uart_write(EWPT_COM_UART, ((uint8_t *)&msg->param), msg->param_len, app_event_com_tx_handler);
		delay(0x1fff);
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

void com_event_uart_rx_frame_handler(void)
{
	QPRINTF("com_event_uart_rx_frame_handler\r\n");
	struct app_uart_data_ind *com_data = ke_msg_alloc(APP_COM_UART_RX_DONE_IND,
															 TASK_APP,
															 TASK_APP,
															 com_env.com_rx_len+1);
	com_data->len=com_env.com_rx_len;
	memcpy(com_data->data,com_env.com_rx_buf,com_env.com_rx_len);
	for (uint8_t i=0;i<com_data->len;i++)
			QPRINTF("%X ",com_data->data[i]);
	QPRINTF("\r\n");
		if ((com_data->len != 36) && (com_env.result_st == 0x0))
		{
			ke_msg_send(com_data);
		}
		else
		{
			if ((com_data->len == 36) && (com_env.result_st == 0x0))
			{			
					com_env.result_st = 0x1;
					ke_msg_send(com_data);
			}
			else
			{
					QPRINTF("len:%d ,com_env.result_st:%d\r\n",com_data->len,com_env.result_st);
					com_uart_rx_start();
					com_env.com_state = COM_CONN_EMPTY;
			}
		}
	ke_timer_clear(APP_COM_RX_TIMEOUT_TIMER, TASK_APP);

	ke_evt_clear(1UL << EVENT_UART_RX_FRAME_ID);
}

void com_event_uart_rx_timeout_handler(void)
{
	ke_timer_set(APP_COM_RX_TIMEOUT_TIMER, TASK_APP, COM_FRAME_TIMEOUT);

	ke_evt_clear(1UL << EVENT_UART_RX_TIMEOUT_ID);
}

void scale_event_power_on_handler(void)
{
		
}

void scale_event_power_off_handler(void)
{

}

int app_com_uart_rx_done_ind_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
		
		switch(msgid)
    {
        case APP_COM_UART_RX_DONE_IND:
        {
            struct app_uart_data_ind* frame = (struct app_uart_data_ind*)param;
            
            //for (uint8_t i=0;i<frame->len;i++)
            //QPRINTF("Cfm %d.\r\n", *(frame->data+i));
            
            ///leo test
//              pt_pdu_send(frame->len, &(frame->data[0]));
						///leo test end
            
            if(frame->len) //have data
            { 
                //calculate page num;
								 //com_env.result_st = 0x1;
                 uint8_t pagket_res = frame->len%QPP_DATA_MAX_LEN;
                 uint8_t pagket_num;
                 if(pagket_res)
                 pagket_num = frame->len/QPP_DATA_MAX_LEN + 1;
                 else
                 pagket_num = frame->len/QPP_DATA_MAX_LEN;
								                  
                 uint8_t cnt=0,sent_pagket=0; 

                for (cnt = 0; (sent_pagket<pagket_num) && cnt < (QPPS_VAL_CHAR_NUM + 1); cnt++)
                {
                     if ((app_qpps_env->char_status >> cnt) & QPPS_VALUE_NTF_CFG)
                     {
												app_qpps_env->char_status &= ~(QPPS_VALUE_NTF_CFG << cnt);
											 
                         if((pagket_res)&&(pagket_num-sent_pagket==1)/* && (com_env.result_st == 0x0)*/)
														app_qpps_data_send(app_qpps_env->conhdl, cnt, pagket_res, (frame->data+sent_pagket*20));
                         else
												 {
														//if (com_env.result_st == 0x00)
														app_qpps_data_send(app_qpps_env->conhdl, cnt, QPP_DATA_MAX_LEN, (frame->data+sent_pagket*20)); 
												 }
                         
                         sent_pagket++;
                     }
                }
            }
        }break;
        default :break;
    }
       
    return (KE_MSG_CONSUMED);
}

int app_com_rx_timeout_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uart_rx_int_enable(EWPT_COM_UART, MASK_DISABLE);  //disable uart rx interrupt 
    struct app_uart_data_ind *com_data = ke_msg_alloc(APP_COM_UART_RX_DONE_IND,
                                 TASK_APP,
                                 TASK_APP,
                                 com_env.com_rx_len+1);
    com_data->len=com_env.com_rx_len;
    memcpy(com_data->data,com_env.com_rx_buf,com_env.com_rx_len);
		for (uint8_t i=0;i<com_data->len;i++)
				QPRINTF("0x%2X ",com_data->data[i]);
		QPRINTF("\r\n");
		if ((com_data->len != 36) && (com_env.result_st == 0x0))
		{
			ke_msg_send(com_data);
		}
		else
		{
			if ((com_data->len == 36) && (com_env.result_st == 0x0))
			{			
					ke_msg_send(com_data);
					com_env.result_st = 0x1;
			}
			else
			{
					QPRINTF("len:%d ,com_env.result_st:%d\r\n",com_data->len,com_env.result_st);
					com_uart_rx_start();
					com_env.com_state = COM_CONN_EMPTY;
			}
		}
    
    return (KE_MSG_CONSUMED);
}

int app_scale_power_on_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    
    return (KE_MSG_CONSUMED);
}

int app_scale_power_off_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    
    return (KE_MSG_CONSUMED);
}


#endif


/// end of usr_ewpt.c
