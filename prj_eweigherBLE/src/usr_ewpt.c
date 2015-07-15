#include	"usr_ewpt.h"
#include	"lib.h"
#include "app_sys.h"

#if		(BLE_EWPT_SERVER)

#define COM_FRAME_TIMEOUT 2 //2*10ms

struct com_env_tag  com_env;

/*
 
* COM INIT
 
****************************************************************************************
 
*/
void com_gpio_init(void)
{
/* 	no need	
		gpio_set_direction(COM_WAKEUP_TRIGGER,GPIO_OUTPUT);
		gpio_write_pin(COM_WAKEUP_TRIGGER,GPIO_HIGH);
	
		gpio_wakeup_config(COM_WAKEUP,GPIO_WKUP_BY_LOW);
		gpio_enable_interrupt(COM_WAKEUP);
*/	
	
		//set SCALE_WAKEUP_PIN to wakeup scale,a 50ms low pulse
		gpio_set_direction(SCALE_WAKEUP_PIN,GPIO_OUTPUT);
		gpio_write_pin(SCALE_WAKEUP_PIN,GPIO_HIGH);
	
		//set a com start event
		if(KE_EVENT_OK != ke_evt_callback_set(EVENT_COM_WAKEUP_ID, 
                                            app_event_com_wakeup_handler))
    {
        ASSERT_ERR(0);
    }
	
}
 
void com_init(void)
{
    com_env.com_state = COM_IDLE;

    //for com uart tx  
    com_env.tx_state = COM_UART_TX_IDLE;	//initialize tx state
		com_env.result_st = 0x0;
		com_env.scale_user_data.update_flag = 1;
    co_list_init(&com_env.queue_tx);			//init TX queue

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

/*void com_wakeup_cb(void)
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
*/

/*
 
* SCALE WAKEUP
 
****************************************************************************************
 
*/
void	wakeup_scale(void)
{
		QPRINTF("\r\n@@@Wakeup Scale!\r\n");
		gpio_write_pin(SCALE_WAKEUP_PIN,GPIO_LOW);
		ke_timer_set(APP_COM_SCALE_WAKEUP_TIMER,TASK_APP,10);
}

int app_com_scale_wakeup_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
		gpio_write_pin(SCALE_WAKEUP_PIN,GPIO_HIGH);
		ke_timer_clear(APP_COM_SCALE_WAKEUP_TIMER,TASK_APP);
		if (SCALE_POWER_ON == get_scale_status())
		{
			uint8_t status =  SCALE_POWER_ON;
			app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(status),&status);
		}
		else
		{
			uint8_t status =  SCALE_POWER_OFF;
			app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(status),&status);
		}
		return(KE_MSG_CONSUMED);
}


enum scale_st get_scale_status(void)
{
	return (gpio_read_pin(SCALE_STATUS_PIN) ==GPIO_HIGH ? SCALE_POWER_ON : SCALE_POWER_DOWN);
}
/*
 
* COM STATUS PROCESS
 
****************************************************************************************
 
*/
void	app_com_wakeup_process(void)
{		
		switch(com_env.com_state)
		{
			case COM_IDLE:
			{
				/*
				if (SCALE_POWER_ON == get_scale_status())
				{
					com_env.com_state = COM_TRAN;
					com_uart_rx_start();
					QPRINTF("\r\n@@@ewpt com rx start!\r\n");
				}
				else
				*/
#ifdef	CATCH_LOG
				QPRINTF("\r\n@@@COM_IDLE!\r\n");	
#endif
				uart_rx_enable(EWPT_COM_UART,MASK_DISABLE);
				break;
			}
			
			case COM_TRAN:
			{
/*				
				if (SCALE_POWER_ON == get_scale_status())
				{
*/
					uart_rx_enable(EWPT_COM_UART,MASK_ENABLE);
					com_uart_rx_start();
#ifdef	CATCH_LOG
					QPRINTF("\r\n@@@COM_TRAN!\r\n");
#endif
				/*
				}
				else
				{
					com_env.com_state = COM_IDLE;					
					uart_rx_enable(EWPT_COM_UART,MASK_DISABLE);
				}
				*/
				break;
			}
			
			case COM_BUSY:
			{
				if (SCALE_POWER_ON != get_scale_status())
				{
					com_env.com_state = COM_IDLE;					
					uart_rx_enable(EWPT_COM_UART,MASK_DISABLE);
				}
				break;
			}
							
			default :break;
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
	  
		app_com_wakeup_process();
}



/*
 
* COM RECEIVE
 
****************************************************************************************
 
*/
void	com_uart_rx_start(void)
{
#ifdef	CATCH_LOG
		QPRINTF("com_uart_rx_start\r\n");
#endif
    com_env.com_rx_len = 0;
    uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
}

void com_uart_rx(void)
{
		com_env.com_rx_len++;
		com_env.com_state = COM_BUSY;
		uint8_t status,err;
	
		//if the 1 < length <40,that's a data,or it will be first byte or err
		if (com_env.com_rx_len > 1 && com_env.com_rx_len < 40)
		{
			  // if it's a power off command 
				if ((com_env.com_rx_len == 6) && (com_env.com_rx_buf[0] == 0x02) && (com_env.com_rx_buf[1] == 0x6f) && (com_env.com_rx_buf[2] == 0x66) && (com_env.com_rx_buf[3] == 0x66) && (com_env.com_rx_buf[4] == 0x03) && (com_env.com_rx_buf[com_env.com_rx_len - 1] == 0xE0))
				{
							com_env.result_st = 0x0;
#ifdef	CATCH_LOG					
							QPRINTF("power off!\r\n");
#endif				
							uint8_t status;
							status = SCALE_POWER_DOWN;
							app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(uint8_t),&status);
							ke_evt_set(1UL << EVENT_UART_RX_FRAME_ID);
				}
				else
				{
						//if it's the end of data
						if (com_env.com_rx_buf[com_env.com_rx_len - 1] == 0x03)
						{
								// if it's a power on command
								if(com_env.com_rx_len == 4 && com_env.com_rx_buf[1] == 0x6f && com_env.com_rx_buf[2] == 0x6e)
								{
#ifdef	CATCH_LOG					
									QPRINTF("power on!\r\n");
#endif
							status = SCALE_POWER_ON;
							app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(uint8_t),&status);
							ke_evt_set(1UL << EVENT_UART_RX_FRAME_ID);
//									{
//										uint8_t result;
//										result = (gpio_read_pin(GPIO_P31) == GPIO_HIGH);
//										QPRINTF("GPIO_P31:  ");
//										QPRINTF("%s\r\n",result ? "HIGH":"LOW");
//										app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(uint8_t),&result);
//									}
								}
								if(com_env.com_rx_len == 5 && com_env.com_rx_buf[1] == 0x6f 
									&& com_env.com_rx_buf[2] == 0x66 && com_env.com_rx_buf[3] == 0x66)
								{
										ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
										uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);	
								}
								else	//it's a data command or  sync resp
								{
										//it's a data command
										if((0x40 <= com_env.com_rx_buf[1]) && (0x44 >= com_env.com_rx_buf[1]))
										{
												//cheakout	data command length
												if (com_env.com_rx_len == 36)
												{
														uint8_t i,CRC_COM = 0;
														for (i = 0;i<(com_env.com_rx_len-2);i++)
																	CRC_COM ^= com_env.com_rx_buf[i];
														//CRC Checkout
														if (CRC_COM == com_env.com_rx_buf[com_env.com_rx_len-2])
														{
															QPRINTF("\r\ndata correct!\r\n");
															ke_evt_clear(1UL<<EVENT_UART_RX_TIMEOUT_ID);
		#ifdef	CATCH_LOG					
															QPRINTF("com_env.result_st %d\r\n",com_env.result_st);
		#endif
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
																err = SCALE_COM_CRC_CHECKOUT_ERR;						
																app_qpps_data_send(app_qpps_env->conhdl,0,1,&err);
																com_uart_rx_start();
														}
												}
												else	//data length err,continue receive data
												{
														ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
														uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);								
												}
												
										}
										else
										{
											//it's a sync resp
											if (com_env.com_rx_buf[1] == 0x53)
											{
													QPRINTF("\r\n@@@sync resp!\r\n");
													//cheakout length
													if (com_env.com_rx_len == 10)
													{
															uint8_t i,CRC_COM = 0;
															for (i = 0;i<(com_env.com_rx_len-2);i++)
																		CRC_COM ^= com_env.com_rx_buf[i];
															if (CRC_COM == com_env.com_rx_buf[com_env.com_rx_len-2])
															{
																	QPRINTF("\r\ndata correct!\r\n");
																	status = SCALE_COM_RESP_NO_ERR,
																	app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(status),&status);
																	com_env.scale_user_data.update_flag = 1;
																	ke_evt_clear(1UL<<EVENT_UART_RX_TIMEOUT_ID);
																	ke_evt_set(1UL << EVENT_UART_RX_FRAME_ID);
															}
															else
															{
																	QPRINTF("CRC verify error!\r\n");
																	app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(err),&err);
																	com_uart_rx_start();
															}
													}
												else	//length err,continue reveive
												{
														QPRINTF("\r\n@@@com_env.com_rx_len:%d\r\n",com_env.com_rx_len);
														ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
														uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);								
												}
											}
											else
											{
												QPRINTF("\r\n@@@it's not in command and resp!\r\n");
												com_uart_rx_start();
											}
										}
								}
						}
						else  //it's not a end flag and continue receive data
						{							
										ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
										uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
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
				//if it's the fire byte,and it must be 0x02. or it's a err data with no end bit
				if (com_env.com_rx_len == 1)
				{
						//the first byte must be 0x02,continue receive.
						if(com_env.com_rx_buf[0] == 0x02)
						{
								//set a timer when timeout finish receive and send data to app
								ke_evt_set(1UL << EVENT_UART_RX_TIMEOUT_ID);
								uart_read(EWPT_COM_UART, &com_env.com_rx_buf[com_env.com_rx_len], 1, com_uart_rx);
						}
						else
						{
#ifdef	CATCH_LOG
					QPRINTF("\r\n@@@First byte error!\r\n");
#endif
							err = SCALE_COM_RX_FIRST_BYTE_ERR;						
							app_qpps_data_send(app_qpps_env->conhdl,0,1,&err);
							com_uart_rx_start();
						}
				}
				else
				{
#ifdef	CATCH_LOG
					QPRINTF("\r\n@@@length error!\r\n");
#endif
						err = SCALE_COM_RX_LENGTH_ERR;						
						app_qpps_data_send(app_qpps_env->conhdl,0,1,&err);
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
					com_env.com_state = COM_TRAN;
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
					com_env.com_state = COM_TRAN;
			}
		}
    
    return (KE_MSG_CONSUMED);
}
/*
 
* COM TX
 
****************************************************************************************
 
*/
void app_event_com_tx_handler(void)
{
	ke_evt_set(1UL<<EVENT_UART_TX_ID);
}
//
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

//void com_uart_write(struct ke_msg *msg)
//{
//    //go to start tx state
//    com_env.tx_state = COM_UART_TX_ONGOING;
//		
//    uart_write(EWPT_COM_UART, ((uint8_t *)&msg->param), msg->param_len, app_event_com_tx_handler);
//		delay(0x1fff);
//}
/// end of usr_ewpt.c
