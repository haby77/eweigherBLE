/**
 ****************************************************************************************
 *
 * @file usr_design.c
 *
 * @brief Product related design.
 *
 * Copyright (C) Quintic 2012-2013
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  USR
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <stdint.h>
#include "app_env.h"
#include "led.h"
#include "uart.h"
#include "lib.h"
#include "usr_design.h"
#include "gpio.h"
#include "button.h"
#if (defined(QN_ADV_WDT))
#include "wdt.h"
#endif
#include "sleep.h"

#if	 (FB_JOYSTICKS)
#include "joysticks.h"
#endif

#if		(BLE_EWPT_SERVER)
#include "usr_ewpt.h"
#endif
/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

#define LED_ON_DUR_ADV_FAST        2
#define LED_OFF_DUR_ADV_FAST       (uint16_t)((GAP_ADV_FAST_INTV2*0.625)/10)
#define LED_ON_DUR_ADV_SLOW        2
#define LED_OFF_DUR_ADV_SLOW       (uint16_t)((GAP_ADV_SLOW_INTV*0.625)/10)
#define LED_ON_DUR_CON          0xffff
#define LED_OFF_DUR_CON                   0
#define LED_ON_DUR_IDLE                   0
#define LED_OFF_DUR_IDLE                  0xffff

//#define APP_HEART_RATE_MEASUREMENT_TO     1400 // 14s
//#define APP_HRPS_ENERGY_EXPENDED_STEP     50
//#define EVENT_BUTTON1_PRESS_ID            0

///IOS Connection Parameter
#define IOS_CONN_INTV_MAX                              0x0640
#define IOS_CONN_INTV_MIN                              0x0480
#define IOS_SLAVE_LATENCY                              0x0000
#define IOS_STO_MULT                                   0x12C0

//scale error reason
enum	scale_error
{
		ERROR_SCALE_POWER_DOWN = 0x0,
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#if (defined(QN_ADV_WDT))
static void adv_wdt_to_handler(void)
{
    ke_state_set(TASK_APP, APP_IDLE);

    // start adv
    app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                          app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                          app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                          GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
}
struct usr_env_tag usr_env = {LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE, false, adv_wdt_to_handler};
#else
struct usr_env_tag usr_env = {LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE};
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Calculate the number of non-zero bit.
 *
 ****************************************************************************************
 */
static uint32_t get_bit_num(uint32_t val)
{
    uint32_t bit_cnt = 0;

    while (val != 0)
    {
        if (val & 0x1)
            bit_cnt++;
        val >>= 1;
    }
    return bit_cnt;
}

uint8_t get_all_notify_on(void)
{
  uint8_t bit_num = get_bit_num(app_qpps_env->char_status);
	if (bit_num >= QPPS_VAL_CHAR_NUM)
		return	SCALE_QPPS_NTF_ON;
	else
		return	SCALE_QPPS_NTF_NOT_ON;
}

/**
 ****************************************************************************************
 * @brief   Led1 for BLE status
 ****************************************************************************************
 */
static void usr_led1_set(uint16_t timer_on, uint16_t timer_off)
{
    usr_env.led1_on_dur = timer_on;
    usr_env.led1_off_dur = timer_off;

    if (timer_on == 0 || timer_off == 0)
    {
        if (timer_on == 0)
        {
            led_set(1, LED_OFF);
        }
        if (timer_off == 0)
        {
            led_set(1, LED_ON);
        }
        ke_timer_clear(APP_SYS_LED_1_TIMER, TASK_APP);
    }
    else
    {
        led_set(1, LED_OFF);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, timer_off);
    }
}

/**
 ****************************************************************************************
 * @brief   Led 1 flash process
 ****************************************************************************************
 */
static void usr_led1_process(void)
{
    if(led_get(1) == LED_ON)
    {
        led_set(1, LED_OFF);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, usr_env.led1_off_dur);
    }
    else
    {
        led_set(1, LED_ON);
        ke_timer_set(APP_SYS_LED_1_TIMER, TASK_APP, usr_env.led1_on_dur);
    }
}

/**
 ****************************************************************************************
 * @brief   Application task message handler
 ****************************************************************************************
 */
void app_task_msg_hdl(ke_msg_id_t const msgid, void const *param)
{
    switch(msgid)
    {
        case GAP_SET_MODE_REQ_CMP_EVT:
				{
            if(APP_IDLE == ke_state_get(TASK_APP))
            {
                usr_led1_set(LED_ON_DUR_ADV_FAST, LED_OFF_DUR_ADV_FAST);
                ke_timer_set(APP_ADV_INTV_UPDATE_TIMER, TASK_APP, 30 * 100);
#if (defined(QN_ADV_WDT))
                usr_env.adv_wdt_enable = true;
#endif
            }
            else if(APP_ADV == ke_state_get(TASK_APP))
            {
                usr_led1_set(LED_ON_DUR_ADV_SLOW, LED_OFF_DUR_ADV_SLOW);
#if (defined(QN_ADV_WDT))
                usr_env.adv_wdt_enable = true;
#endif
            }
            break;
					}

        case GAP_ADV_REQ_CMP_EVT:
				{
            usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);
            ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP);
            break;
				}

        case GAP_DISCON_CMP_EVT:
				{
            usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);

            // start adv
            app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                    app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                    app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                    GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);
            break;
				}
				
        case GAP_LE_CREATE_CONN_REQ_CMP_EVT:
				{
            if(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.status == CO_ERROR_NO_ERROR)
            {
                if(GAP_PERIPHERAL_SLV == app_get_role())
                {
                    ke_timer_clear(APP_ADV_INTV_UPDATE_TIMER, TASK_APP);
                    usr_led1_set(LED_ON_DUR_CON, LED_OFF_DUR_CON);
#if (defined(QN_ADV_WDT))
                    usr_env.adv_wdt_enable = false;
#endif

                    // Update cnx parameters
                    //if (((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.con_interval >  IOS_CONN_INTV_MAX)
                    {
                        // Update connection parameters here
                        struct gap_conn_param_update conn_par;
                        /// Connection interval minimum
                        conn_par.intv_min = IOS_CONN_INTV_MIN;
                        /// Connection interval maximum
                        conn_par.intv_max = IOS_CONN_INTV_MAX;
                        /// Latency
                        conn_par.latency = IOS_SLAVE_LATENCY;
                        /// Supervision timeout, Time = N * 10 msec
                        conn_par.time_out = IOS_STO_MULT;
                        app_gap_param_update_req(((struct gap_le_create_conn_req_cmp_evt *)param)->conn_info.conhdl, &conn_par);
                    }
                }
            }
            break;
					}
						
				case QPPS_DAVA_VAL_IND:
							{
#if		(BLE_EWPT_SERVER)																
						//all notify on and app can get 
            uint8_t bit_num = get_bit_num(app_qpps_env->char_status);
            if (bit_num >= QPPS_VAL_CHAR_NUM + 1)
						{							
									if (SCALE_POWER_ON == get_scale_status())
									{
											//get the user data from app and printf
											struct qpps_data_val_ind* par = (struct qpps_data_val_ind*)param;
#ifdef CATCH_LOG										
											for (uint8_t i=0;i<par->length;i++)
													QPRINTF("%02X ",par->data[i]);
											QPRINTF("\r\n");
#endif								
											//check data packet head and tail
											if ((par->data[0] == 0x02) && (par->data[1] == 0x53) && (par->data[par->length-1] == 0x03))
											{
													//calc the xor value
													uint8_t i,xor_sum = 0;
												
													//xor all data
													for (i = 0;i < par->length-2;i++)
														xor_sum ^= par->data[i];
												
													//xor checkout
													if (xor_sum == par->data[par->length - 2])
													{
															uint8_t err;
															memcpy(com_env.scale_user_data.data,par->data,par->length);
#ifdef	CATCH_LOG														
															QPRINTF("user_data:");
															for (uint8_t i=0;i<par->length;i++)
																	QPRINTF("0x%2X ",com_env.scale_user_data.data[i]);
															QPRINTF("\r\n");
#endif														
															///send sync information to com
															com_pdu_send(par->length,&(par->data[0]));
															com_env.scale_user_data.update_flag = 0;
															err = SCALE_QPPS_DATA_SEND;
															app_qpps_data_send(app_qpps_env->conhdl,0,1,&err);
															QPRINTF("\r\n\r\n------->ready!\r\n\r\n");
													}
													else
													{
															uint8_t err;
															err = SCALE_XOR_SUM_ERR;
															app_qpps_data_send(app_qpps_env->conhdl,0,1,&err);
															char xor_sum_a[4];
															sprintf(xor_sum_a,"0x%X",xor_sum);
															app_qpps_data_send(app_qpps_env->conhdl,1,sizeof(xor_sum_a),(uint8_t *)xor_sum_a);
#ifdef	CATCH_LOG
														QPRINTF("xor_sum :0x%02X  error!\r\n",xor_sum);
#endif
													}
											}
											else
											{
													/// send the power on status to app
													uint8_t status = SCALE_POWER_ON;
													app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(status),&status);
													{
#ifdef	CATCH_LOG
														uint8_t result;
														result = (gpio_read_pin(GPIO_P31) == GPIO_HIGH);
														QPRINTF("GPIO_P31:  ");
														QPRINTF("%s\r\n",result ? "HIGH":"LOW");
														app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(uint8_t),&result);
#endif
													}
											}
									}
									else
									{
#ifdef	CATCH_LOG
											QPRINTF("\r\n@@@notify on and power down,wakeup!\r\n");
#endif										
											//try wakeup scale
											wakeup_scale();
											
											//send err code to app
											uint8_t error =  SCALE_POWER_OFF;
											app_qpps_data_send(app_qpps_env->conhdl,0,1,&error);
									}
								}
						else
						{
#ifdef	CATCH_LOG
								QPRINTF("\r\n@@@not all notify is on\r\n");
#endif							
								com_env.com_state = COM_IDLE;
								uint8_t error =  SCALE_QPPS_NTF_NOT_ON;
								app_qpps_data_send(app_qpps_env->conhdl,0,1,&error);
								ke_evt_set(1UL << EVENT_COM_WAKEUP_ID);
						}
#endif
						break;
							}

        case QPPS_DISABLE_IND:
            break;

        case QPPS_CFG_INDNTF_IND:
        {
#if	(BLE_EWPT_SERVER)
						//if all notify is on,the enter the tran mode and start receive com data
            uint8_t bit_num = get_bit_num(app_qpps_env->char_status);
            if (bit_num >= QPPS_VAL_CHAR_NUM + 1)  
            {                
                QPRINTF("all notify is on!\r\n");
								uint8_t status =  SCALE_QPPS_NTF_ON;
								app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(status),&status);
								com_env.com_state = COM_TRAN;
								if (SCALE_POWER_DOWN == get_scale_status())
									wakeup_scale();
								else
								{
										QPRINTF("\r\n@@@POWER_ON!\r\n");
										uint8_t err =  SCALE_POWER_ON;
										app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(err),&err);									
								}
/*								gpio_write_pin(COM_WAKEUP_TRIGGER,GPIO_LOW);
								if (gpio_read_pin(COM_WAKEUP) == GPIO_LOW)
										com_uart_rx_start();
								com_env.com_state = COM_CONN_EMPTY;

*/
						}
						else
						{
								com_env.com_state = COM_IDLE;
								uint8_t err =  SCALE_QPPS_NTF_NOT_ON;
								app_qpps_data_send(app_qpps_env->conhdl,0,sizeof(err),&err);
						}
 						ke_evt_set(1UL << EVENT_COM_WAKEUP_ID);
#endif           
				break;
        }
				
				case	QPPS_DATA_SEND_CFM	:							
					{
#if	(BLE_EWPT_SERVER)
								// when finish send data to app,restart receive data for com
								uint8_t bit_num = get_bit_num(app_qpps_env->char_status);
								if (bit_num >= QPPS_VAL_CHAR_NUM)
								{
									com_env.com_state = COM_TRAN;
								}
								else
								{
									com_env.com_state = COM_IDLE;
								}
								ke_evt_set(1UL << EVENT_COM_WAKEUP_ID);
#endif
						break;
					}

#if	(BLE_OTA_SERVER)						
        case OTAS_TRANSIMIT_STATUS_IND:
            QPRINTF(" APP get OTA transmit status = %d , describe = %d \r\n" , ((struct otas_transimit_status_ind*)param)->status,
                                                                              ((struct otas_transimit_status_ind*)param)->status_des);
            
            //only need response once when ota status is in ota status start request
            if(((struct otas_transimit_status_ind*)param)->status == OTA_STATUS_START_REQ)  
            {
                app_ota_ctrl_resp(START_OTA);
            }
            break;
//end
#endif

        default:
            break;
    }
}

/**
 ****************************************************************************************
 * @brief Handles LED status timer.
 *
 * @param[in] msgid      APP_SYS_UART_DATA_IND
 * @param[in] param      Pointer to struct app_uart_data_ind
 * @param[in] dest_id    TASK_APP
 * @param[in] src_id     TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_led_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(msgid == APP_SYS_LED_1_TIMER)
    {
        usr_led1_process();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles advertising mode timer event.
 *
 * @param[in] msgid     APP_ADV_INTV_UPDATE_TIMER
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 * @description
 *
 * This handler is used to inform the application that first phase of adversting mode is timeout.
 ****************************************************************************************
 */
int app_gap_adv_intv_update_timer_handler(ke_msg_id_t const msgid, void const *param,
                                          ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    if(APP_ADV == ke_state_get(TASK_APP))
    {
        usr_led1_set(LED_ON_DUR_IDLE, LED_OFF_DUR_IDLE);

        // Update Advertising Parameters
        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE, 
                                app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE), 
                                app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                                GAP_ADV_SLOW_MAX_INTV, GAP_ADV_SLOW_MAX_INTV);
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   Restore peripheral setting after wakeup
 ****************************************************************************************
 */
void usr_sleep_restore(void)
{
#if QN_DBG_PRINT
    uart_init(QN_DEBUG_UART, USARTx_CLK(0), UART_9600);
    uart_tx_enable(QN_DEBUG_UART, MASK_ENABLE);
    uart_rx_enable(QN_DEBUG_UART, MASK_ENABLE);
#endif
	
#if	BLE_EWPT_SERVER
    uart_init(EWPT_COM_UART, USARTx_CLK(0), UART_9600);
    uart_tx_enable(EWPT_COM_UART, MASK_ENABLE);
    uart_rx_enable(EWPT_COM_UART, MASK_ENABLE);
#endif

#if (defined(QN_ADV_WDT))
    if(usr_env.adv_wdt_enable)
    {
        wdt_init(1007616, WDT_INT_MOD); // 30.75s
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Handles button press after cancel the jitter.
 *
 * @param[in] msgid     Id of the message received
 * @param[in] param     None
 * @param[in] dest_id   TASK_APP
 * @param[in] src_id    TASK_APP
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int app_button_timer_handler(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    switch(msgid)
    {
        case APP_SYS_BUTTON_1_TIMER:
            // make sure the button is pressed
            if(gpio_read_pin(BUTTON1_PIN) == GPIO_LOW)
            {
                if(APP_IDLE == ke_state_get(TASK_APP))
                {
                    if(!app_qpps_env->enabled)
                    {
                        // start adv
                        app_gap_adv_start_req(GAP_GEN_DISCOVERABLE|GAP_UND_CONNECTABLE,
                                app_env.adv_data, app_set_adv_data(GAP_GEN_DISCOVERABLE),
                                app_env.scanrsp_data, app_set_scan_rsp_data(app_get_local_service_flag()),
                                GAP_ADV_FAST_INTV1, GAP_ADV_FAST_INTV2);

#if (QN_DEEP_SLEEP_EN)
                        // prevent entering into deep sleep mode
                        sleep_set_pm(PM_SLEEP);
#endif
                    }
                }
                else if(APP_ADV == ke_state_get(TASK_APP))
                {
                    // stop adv
                    app_gap_adv_stop_req();

#if (QN_DEEP_SLEEP_EN)
                    // allow entering into deep sleep mode
                    sleep_set_pm(PM_DEEP_SLEEP);
#endif
                }
            }
            break;

        default:
            ASSERT_ERR(0);
            break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles button press before key debounce.
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
void app_event_button1_press_handler(void)
{
#if ((QN_DEEP_SLEEP_EN) && (!QN_32K_RCO))
    if (sleep_env.deep_sleep)
    {
        sleep_env.deep_sleep = false;
        // start 32k xtal wakeup timer
        wakeup_32k_xtal_start_timer();
    }
#endif

    // delay 20ms to debounce
#if (FB_JOYSTICKS)
   ke_timer_set(APP_KEY_SCAN_TIMER,TASK_APP,2);
#else
   ke_timer_set(APP_SYS_BUTTON_1_TIMER, TASK_APP, 2);
#endif
    ke_evt_clear(1UL << EVENT_BUTTON1_PRESS_ID);
}

/**
 ****************************************************************************************
 * @brief   Button 1 click callback
 * @description
 *  Button 1 is used to enter adv mode.
 ****************************************************************************************
 */
void usr_button1_cb(void)
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

#if (FB_JOYSTICKS)
     usr_button_env.button_st = button_press;
#endif
    // key debounce:
    // We can set a soft timer to debounce.
    // After wakeup BLE, the timer is not calibrated immediately and it is not precise.
    // So We set a event, in the event handle, set the soft timer.
    ke_evt_set(1UL << EVENT_BUTTON1_PRESS_ID);
}

/**
 ****************************************************************************************
 * @brief   All GPIO interrupt callback
 ****************************************************************************************
 */
void gpio_interrupt_callback(enum gpio_pin pin)
{
    switch(pin)
    {
        case BUTTON1_PIN:
            //Button 1 is used to enter adv mode.
            usr_button1_cb();
            break;

/*#if	(BLE_EWPT_SERVER)				
				case	COM_WAKEUP:
							com_wakeup_cb();
#endif*/
				
#if (defined(QN_TEST_CTRL_PIN))
        case QN_TEST_CTRL_PIN:
            //When test controll pin is changed to low level, this function will reboot system.
            gpio_disable_interrupt(QN_TEST_CTRL_PIN);
            syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_REBOOT_SYS);
            break;
#endif

        default:
            break;
    }
}


/**
 ****************************************************************************************
 * @brief   User initialize
 ****************************************************************************************
 */
void usr_init(void)
{
#if	(BLE_EWPT_SERVER)
		com_init();
#endif	

	if(KE_EVENT_OK != ke_evt_callback_set(EVENT_BUTTON1_PRESS_ID, 
                                            app_event_button1_press_handler))
    {
        ASSERT_ERR(0);
    }
#if		(FB_JOYSTICKS)
		 if(KE_EVENT_OK != ke_evt_callback_set(EVENT_ADC_KEY_SAMPLE_CMP_ID,
                                           app_event_adc_key_sample_cmp_handler))
		{
				ASSERT_ERR(0);
		}
#endif
}


/// @} USR

