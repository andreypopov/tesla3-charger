//-------Объявляем переменные и структуры---------------------
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8] = {0,};
uint8_t RxData[8] = {0,};

uint32_t TxMailbox = 0;

uint8_t pcs_alive_counter=0;


#define PCS_ALERT_MATRIX_SIZE  16U // Размер массива (желательно степень двойки)

// Глобальные или статические переменные в чистом Си
volatile uint8_t pcs_alert_matrix[PCS_ALERT_MATRIX_SIZE] = {0};
volatile uint8_t PCS_AlertCnt = 0;
volatile uint16_t AlertCANId = 0;
volatile uint8_t AlertRxError = 0;
volatile _Bool Short2B2 = false; //This newer PCS was captured with the 5-byte 0x2B2 format

// Флаг-заменитель C++ параметров: 1 — логирование включено, 0 — выключено
volatile uint8_t param_alert_log_enabled = 1;

//============================================================
volatile uint8_t msec;

volatile uint8_t msec_10; //счетчик чтобы получить 10ms
volatile uint8_t msec_50; //счетчик чтобы получить 50ms
volatile uint8_t msec_100; //счетчик чтобы получить 100ms

volatile uint16_t seconds;

uint32_t mailBoxNum;

volatile _Bool upd_disp; //бит обновления дисплея


volatile uint8_t t_buzzer;

uint16_t VOLTAGE_DCDC=1400;

uint8_t pcs_status=0;

volatile _Bool pcs_w=0;
volatile _Bool dcdc_w=0;
volatile _Bool chg_w=0;

volatile _Bool ac_w=0; //AC подтверждается ответом 0x264, а не константой
volatile _Bool chg_request=0; //защёлкнутый запрос на включение зарядки

volatile uint8_t pin_dcdc_status;
volatile uint8_t pin_chg_status;

_Bool mux545=true;
_Bool mux3b2=true;

uint8_t pcs_counter;
uint8_t Count545;

uint16_t CHGpwr=0;             //текущий кодированный setpoint 0x2B2; старт всегда с 0 W
uint16_t CHGpwrTarget=0;       //целевой кодированный setpoint 0x2B2
uint16_t CHGdesiredPowerTargetW=0; //физическая цель: ток уставки x измеренное AC
volatile uint8_t CHGpowerRequestMultiplier=1; //RC6: только стандартный 1 W/bit
uint16_t PCS_Power_Req;

uint8_t mess[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //заготовка отправки пакета

uint8_t cell_SOC;

/*
 * Главный регистр тока зарядки. Допустимый диапазон 0...16 A.
 * Значение выше 16 A программно ограничивается до 16 A.
 */
volatile uint8_t CHGcurrentSetpointA=16;
volatile uint8_t CHGcurrentAppliedA=16;
uint8_t ACILim=16;
uint16_t AClim;
uint16_t ACamps;
uint16_t ACpwr;
uint16_t DCDCvolts;
uint16_t DCDCamps;
uint16_t HVvolts; //не подставляем фиктивное HV до получения 0x2C4
uint16_t LVvolts;
uint16_t ACvolts;

//Decoded PCS diagnostics, visible in the debugger and on the LCD.
volatile uint8_t pcs_main_state;
volatile uint8_t pcs_hv_charge_status;
volatile uint8_t pcs_grid_config;
volatile uint8_t pcs_pwm_enable_line;
volatile uint8_t pcs_hw_variant;
volatile uint8_t pcs_shutdown_request;
volatile uint8_t pcs_instant_ac_power_deci_kw;
volatile uint8_t pcs_max_ac_power_deci_kw;
volatile uint8_t pcs_phase_a_request_deci_amps;
volatile uint8_t pcs_phase_b_request_deci_amps;
volatile uint8_t pcs_phase_c_request_deci_amps;
volatile uint8_t pcs_active_charge_branches;
volatile uint16_t pcs_ac_current_request_deci_amps;
volatile uint8_t pcs_ac_current_request_amps;
volatile uint16_t ac_current_raw;
volatile uint16_t ac_voltage_raw;
volatile uint16_t dcdc_lv_voltage_raw;
volatile uint16_t dcdc_hv_voltage_raw;
volatile uint16_t dcdc_output_current_raw;
volatile uint16_t dcdc_legacy_current_raw;
volatile uint16_t charge_overcurrent_trip_count;
volatile uint8_t pcs_alert_page;
volatile uint8_t pcs_last_alert_id;
volatile uint16_t pcs_alert_can_id;
volatile uint8_t pcs_alert_rx_error;
volatile uint8_t pcs_alert_hvp_mia_active;
volatile uint8_t pcs_alert_bms_mia_active;
volatile uint8_t pcs_alert_cp_mia_active;
volatile uint8_t pcs_alert_vcfront_mia_active;
volatile uint8_t pcs_alert_charge_power_rationality_active;
volatile uint8_t pcs_alert_can_rationality_active;
volatile uint8_t pcs_alert_ui_mia_active;
volatile uint32_t rx3A4Count;
volatile uint32_t rx3A4PageCount[2];
volatile uint32_t rx424Count;
volatile uint32_t rx76CCount;
volatile uint8_t pcs_debug_mux76c;
volatile uint8_t pcs_charge_port_profile;
volatile uint16_t pcs_charge_phase_current_raw[3];
volatile uint16_t pcs_charge_phase_current_milliamps[3];
volatile uint16_t pcs_charge_phase_current_total_milliamps;
volatile uint32_t rx76CPhaseCount[3];
volatile uint8_t dbg_rx76C_phase[3][8];
volatile uint8_t dbg_rx3A4_page[2][8];
volatile uint8_t dbg_rx424[8];
volatile uint8_t dbg_rx424_dlc;

//Last encoded commands. These are useful during SWD debugging.
volatile uint8_t dbg_tx22A[4];
volatile uint8_t dbg_tx2B2[5];
volatile uint8_t dbg_tx2B2_dlc;
volatile uint8_t dbg_tx23D[4];
volatile uint8_t dbg_tx21D[8];
volatile uint8_t dbg_tx25D[8];
volatile uint8_t dbg_tx333[5];



volatile uint32_t rxmess;

uint16_t t_starting=0;

_Bool RX264=0;
_Bool RX224=0;
_Bool RX2C4=0;
