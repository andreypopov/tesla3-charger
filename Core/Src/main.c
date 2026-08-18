/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with Tesla Model 3 PCS logic
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "defines.h"
#include "ili9341.h"
#include "pcs_protocol.h"
#include "registers.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_TX_QUEUE_SIZE          32U
#define CAN_BUSY_TIMEOUT_MS        100U  // Таймаут зависания Mailbox
#define PCS_WAKE_DELAY_MS          2000U // Время начального пробуждения PCS
#define PCS_READY_TIMEOUT_MS       5000U
#define PCS_RX_TIMEOUT_MS          1000U
#define PCS_HV_RX_TIMEOUT_MS       2500U
#define PCS_HV_MIN_V               276U  // Минимальное рабочее напряжение ВВ батареи
#define PCS_HV_MAX_V               420U  // Максимальное напряжение (расширено под US)
#define PCS_AC_START_MIN_V         180U  // Для этой установки: стабильная сеть 220 В
#define PCS_AC_HOLD_MIN_V          80U   // Гистерезис отключения при пропадании сети
#define PCS_AC_STABLE_MS           1000U
#define PCS_AC_DROP_MS             500U
#define CHARGE_RAMP_STEP_W         10U   // Шаг плавного набора мощности заряда (Ваты)
#define CHARGE_RAMP_DOWN_STEP_W    100U  // Снижение мощности быстрее повышения
#define CHARGE_CURRENT_MAX_A       16U   // Аппаратный предел этой установки
#define CHARGE_NOMINAL_AC_V        230U  // Подстановка только до первого валидного 0x264
#define CHARGE_AC_MAX_V            260U  // Защита расчёта от ошибочного значения CAN
#define CHARGE_POWER_MAX_W         4000U // Верхний предел запроса 0x2B2
#define AUTO_CHARGE_FROM_AC        1U    // Нет charge port/EVSE: старт по реальному 0x264
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
typedef enum
{
  PCS_STATE_BOOT = 0,
  PCS_STATE_STANDBY = 1,
  PCS_STATE_WAIT_READY = 2,
  PCS_STATE_RAMP = 3,
  PCS_STATE_CHARGING = 4,
  PCS_STATE_FAULT_CAN = 10,
  PCS_STATE_FAULT_HV = 11,
  PCS_STATE_FAULT_AC = 12,
  PCS_STATE_FAULT_PCS = 13
} PCS_ControlState;

typedef struct
{
  CAN_TxHeaderTypeDef header;
  uint8_t data[8];
} CAN_QueuedFrame;

static CAN_QueuedFrame canTxQueue[CAN_TX_QUEUE_SIZE];
static uint8_t canTxHead = 0;
static uint8_t canTxTail = 0;

static volatile _Bool task10ms = false;
static volatile _Bool task50ms = false;
static volatile _Bool task100ms = false;
static volatile uint32_t schedulerMisses = 0;

static volatile uint32_t canTxOk = 0;
static volatile uint32_t canTxErrors = 0;
static volatile uint32_t canTxQueueOverflows = 0;
static volatile uint32_t canErrorEvents = 0;
static volatile uint32_t canLastError = 0;
static volatile uint32_t canLastEsr = 0;
static volatile uint32_t canHardwareResets = 0;

static volatile uint32_t lastPcsRxMs = 0;
static volatile uint32_t last204Ms = 0;
static volatile uint32_t last224Ms = 0;
static volatile uint32_t last264Ms = 0;
static volatile uint32_t last2C4Ms = 0;
static volatile uint32_t rx204Count = 0;
static volatile uint32_t rx224Count = 0;
static volatile uint32_t rx264Count = 0;
static volatile uint32_t rx2C4Count = 0;
static uint32_t lastMailboxFreeMs = 0;

static PCS_ControlState controlState = PCS_STATE_BOOT;
static uint32_t stateEnteredMs = 0;
static uint8_t buttonStable = 1;
static uint8_t buttonSample = 1;
static uint8_t buttonDebounceCount = 0;
static uint32_t acPresentSinceMs = 0;
static uint32_t acAbsentSinceMs = 0;
static _Bool autoStartArmed = true;
static volatile uint8_t pcsAlertMatrixLast[8] = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void set_control_state(PCS_ControlState newState);
static _Bool elapsed_ms(uint32_t since, uint32_t interval);
static _Bool can_queue_frame(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData);
static void can_service_tx(void);
static void PCS_cksum_local(uint8_t *data, uint16_t id);
static void control_step_100ms(void);
static void sample_charge_button(void);
static void update_auto_charge_request(_Bool acStartValid, _Bool acHoldValid);
static void update_charge_setpoints(void);
static _Bool ramp_charge_power(void);
static void stop_charge(void);
static void apply_output_state(void);
void send_mes_10(void);
void send_mes_50(void);
void send_mes_100(void);
void get_HV(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void set_control_state(PCS_ControlState newState)
{
  controlState = newState;
  stateEnteredMs = HAL_GetTick();
}

static _Bool elapsed_ms(uint32_t since, uint32_t interval)
{
  return (uint32_t)(HAL_GetTick() - since) >= interval;
}

static _Bool can_queue_frame(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint8_t next = (uint8_t)((canTxHead + 1U) % CAN_TX_QUEUE_SIZE);
  if (next == canTxTail)
  {
    canTxQueueOverflows++;
    if (primask == 0U) { __enable_irq(); }
    return false;
  }

  canTxQueue[canTxHead].header = *pHeader;
  uint8_t dlc = (pHeader->DLC > 8) ? 8 : pHeader->DLC;
  for (uint8_t i = 0; i < dlc; i++)
  {
    canTxQueue[canTxHead].data[i] = pData[i];
  }
  canTxHead = next;

  if (primask == 0U) { __enable_irq(); }
  return true;
}

static void can_service_tx(void)
{
  uint32_t now = HAL_GetTick();

  // 1. Проверяем наличие свободных ящиков (Mailboxes) в контроллере CAN
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0U)
  {
    lastMailboxFreeMs = now;
  }
  else
  {
    // Если все ящики заняты дольше таймаута (Bus-Off или зависание шины) — сбрасываем запросы
    if ((now - lastMailboxFreeMs) > CAN_BUSY_TIMEOUT_MS)
    {
      canHardwareResets++;
      HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);

      __disable_irq();
      canTxTail = canTxHead; // Очищаем кольцевой буфер очереди
      __enable_irq();

      lastMailboxFreeMs = HAL_GetTick();
      return;
    }
  }

  // 2. Цикл отправки пакетов из кольцевого буфера очереди в физическую шину
  while (canTxTail != canTxHead)
  {
    // Если в железе STM32 временно кончились свободные почтовые ящики — прерываем цикл до следующего вызова
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
    {
      break;
    }

    uint32_t mailbox = 0;

    // СТРОГО СТАНДАРТНЫЙ ВЫЗОВ HAL. Все экспериментальные хаки TIR удалены!
    // Драйвер HAL сам корректно разложит байты мощности пакета 0x2B2 по регистрам данных.
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
        &hcan,
        &canTxQueue[canTxTail].header,
        canTxQueue[canTxTail].data,
        &mailbox);

    if (status == HAL_OK)
    {
      // Пакет успешно скопирован в железный ящик микросхемы и улетит в сеть автоматически
      canTxTail = (uint8_t)((canTxTail + 1U) % CAN_TX_QUEUE_SIZE);
      canTxOk++;
    }
    else if (status == HAL_BUSY)
    {
      break;
    }
    else
    {
      // Обработка критической ошибки отправки
      canTxErrors++;
      canLastError = HAL_CAN_GetError(&hcan);
      canTxTail = (uint8_t)((canTxTail + 1U) % CAN_TX_QUEUE_SIZE);
    }
  }
}


void get_HV(void)
{
  uint16_t cell_med = HVvolts / 0.88;

  if (cell_med >= 420) { cell_SOC = 100; return; }
  if (cell_med >= 414) { cell_SOC = 95;  return; }
  if (cell_med >= 408) { cell_SOC = 90;  return; }
  if (cell_med >= 402) { cell_SOC = 85;  return; }
  if (cell_med >= 396) { cell_SOC = 80;  return; }
  if (cell_med >= 390) { cell_SOC = 75;  return; }
  if (cell_med >= 384) { cell_SOC = 70;  return; }
  if (cell_med >= 378) { cell_SOC = 65;  return; }
  if (cell_med >= 372) { cell_SOC = 60;  return; }
  if (cell_med >= 366) { cell_SOC = 55;  return; }
  if (cell_med >= 360) { cell_SOC = 50;  return; }
  if (cell_med >= 354) { cell_SOC = 45;  return; }
  if (cell_med >= 348) { cell_SOC = 40;  return; }
  if (cell_med >= 342) { cell_SOC = 35;  return; }
  if (cell_med >= 336) { cell_SOC = 30;  return; }
  if (cell_med >= 330) { cell_SOC = 25;  return; }
  if (cell_med >= 324) { cell_SOC = 20;  return; }
  if (cell_med >= 318) { cell_SOC = 15;  return; }
  if (cell_med >= 312) { cell_SOC = 10;  return; }
  if (cell_med >= 306) { cell_SOC = 5;   return; }else{cell_SOC = 0;};

}

void PCS_Send_Power_Request_US(uint16_t power_watts, uint8_t chg_active)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t local_pwr_buffer[5] = {0};
  _Bool enabled = chg_active &&
      (controlState == PCS_STATE_RAMP || controlState == PCS_STATE_CHARGING);

  header.StdId = 0x2B2;
  header.ExtId = 0x00;
  header.RTR = CAN_RTR_DATA;
  header.IDE = CAN_ID_STD;
  header.DLC = PCS_Encode2B2(local_pwr_buffer, power_watts, enabled, Short2B2);
  header.TransmitGlobalTime = DISABLE;

  dbg_tx2B2_dlc = header.DLC;
  for (uint8_t i = 0; i < 5U; i++)
  {
    dbg_tx2B2[i] = local_pwr_buffer[i];
  }

  can_queue_frame(&header, local_pwr_buffer);
}

static void apply_output_state(void)
{
  if (pcs_w)  { pcs_on;  } else { pcs_off;  }
  if (dcdc_w) { dcdc_on; } else { dcdc_off; }
  if (chg_w)  { chg_on;  } else { chg_off;  }
}

static void stop_charge(void)
{
  CHGpwr = 0;
  chg_w = 0;
  apply_output_state();
}

static void sample_charge_button(void)
{
  uint8_t sample = (sb_up == GPIO_PIN_SET) ? 1U : 0U;

  if (sample != buttonSample)
  {
    buttonSample = sample;
    buttonDebounceCount = 0;
    return;
  }

  if (buttonDebounceCount < 3U)
  {
    buttonDebounceCount++;
    return;
  }

  if (buttonStable != buttonSample)
  {
    buttonStable = buttonSample;
    if (buttonStable == 0U)
    {
      chg_request = !chg_request;
      buzzer_on;
      t_buzzer = 2;
    }
  }
}

static void update_auto_charge_request(_Bool acStartValid, _Bool acHoldValid)
{
#if AUTO_CHARGE_FROM_AC
  uint32_t now = HAL_GetTick();

  if (acStartValid)
  {
    acAbsentSinceMs = 0U;
    if (acPresentSinceMs == 0U)
    {
      acPresentSinceMs = (now == 0U) ? 1U : now;
    }

    if (autoStartArmed && (uint32_t)(now - acPresentSinceMs) >= PCS_AC_STABLE_MS)
    {
      chg_request = 1;
      autoStartArmed = false;
      buzzer_on;
      t_buzzer = 2;
    }
  }
  else if (!acHoldValid)
  {
    acPresentSinceMs = 0U;
    if (acAbsentSinceMs == 0U)
    {
      acAbsentSinceMs = (now == 0U) ? 1U : now;
    }

    if ((uint32_t)(now - acAbsentSinceMs) >= PCS_AC_DROP_MS)
    {
      chg_request = 0;
      autoStartArmed = true;
    }
  }
#else
  (void)acStartValid;
  (void)acHoldValid;
#endif
}

static void update_charge_setpoints(void)
{
  uint16_t calculationVoltage = CHARGE_NOMINAL_AC_V;
  uint8_t appliedCurrent = PCS_ClampChargeCurrent(CHGcurrentSetpointA,
                                                   CHARGE_CURRENT_MAX_A);

  if ((ACvolts >= PCS_AC_HOLD_MIN_V) && (ACvolts <= CHARGE_AC_MAX_V))
  {
    calculationVoltage = ACvolts;
  }

  CHGcurrentAppliedA = appliedCurrent;
  ACILim = appliedCurrent;
  CHGpwrTarget = PCS_CalculateChargePowerTarget(appliedCurrent,
                                                calculationVoltage,
                                                CHARGE_POWER_MAX_W);

  /* Нулевой регистр означает немедленный безопасный запрос 0 W. */
  if (appliedCurrent == 0U)
  {
    CHGpwr = 0U;
  }
}

static _Bool ramp_charge_power(void)
{
  if (CHGpwr < CHGpwrTarget)
  {
    uint16_t nextPower = (uint16_t)(CHGpwr + CHARGE_RAMP_STEP_W);
    CHGpwr = (nextPower > CHGpwrTarget) ? CHGpwrTarget : nextPower;
  }
  else if (CHGpwr > CHGpwrTarget)
  {
    uint16_t delta = (uint16_t)(CHGpwr - CHGpwrTarget);
    CHGpwr = (delta > CHARGE_RAMP_DOWN_STEP_W)
        ? (uint16_t)(CHGpwr - CHARGE_RAMP_DOWN_STEP_W)
        : CHGpwrTarget;
  }

  return CHGpwr == CHGpwrTarget;
}

static void control_step_100ms(void)
{
  uint32_t now = HAL_GetTick();
  _Bool pcsCommsRecent = (lastPcsRxMs != 0U) && ((uint32_t)(now - lastPcsRxMs) < PCS_RX_TIMEOUT_MS);
  _Bool hvRecent = (last2C4Ms != 0U) && ((uint32_t)(now - last2C4Ms) < PCS_HV_RX_TIMEOUT_MS);
  _Bool hvValid = hvRecent && (HVvolts >= PCS_HV_MIN_V) && (HVvolts <= PCS_HV_MAX_V);
  _Bool acRecent = (last264Ms != 0U) && ((uint32_t)(now - last264Ms) < PCS_RX_TIMEOUT_MS);
  _Bool acStartValid = acRecent && (ACvolts >= PCS_AC_START_MIN_V);
  _Bool acHoldValid = acRecent && (ACvolts >= PCS_AC_HOLD_MIN_V);
  _Bool pcsStatusRecent = (last204Ms != 0U) && ((uint32_t)(now - last204Ms) < PCS_RX_TIMEOUT_MS);
  _Bool pcsFault = pcsStatusRecent && ((pcs_main_state == 8U) || (pcs_hv_charge_status == 3U));

  update_charge_setpoints();
  sample_charge_button();
  update_auto_charge_request(acStartValid, acHoldValid);
  ac_w = acHoldValid;

  switch (controlState)
  {
    case PCS_STATE_BOOT:
      pcs_w = 1;
      dcdc_w = 1;
      chg_w = 0;
      CHGpwr = 0;
      apply_output_state();
      if (elapsed_ms(stateEnteredMs, PCS_WAKE_DELAY_MS))
      {
        set_control_state(PCS_STATE_STANDBY);
      }
      break;

    case PCS_STATE_STANDBY:
      stop_charge();
      if (chg_request)
      {
        dcdc_w = 1;
        chg_w = 0;
        CHGpwr = 0;
        apply_output_state();
        set_control_state(PCS_STATE_WAIT_READY);
      }
      break;

    case PCS_STATE_WAIT_READY:
      if (!chg_request)
      {
        stop_charge();
        set_control_state(PCS_STATE_STANDBY);
      }
      else if (pcsCommsRecent && hvValid && acStartValid && !pcsFault &&
               elapsed_ms(stateEnteredMs, 500U))
      {
        chg_w = 1;
        apply_output_state();
        CHGpwr = 0;
        set_control_state(PCS_STATE_RAMP);
      }
      else if (elapsed_ms(stateEnteredMs, PCS_READY_TIMEOUT_MS))
      {
        chg_request = 0;
        stop_charge();
        if (!pcsCommsRecent)      { set_control_state(PCS_STATE_FAULT_CAN); }
        else if (!hvValid)        { set_control_state(PCS_STATE_FAULT_HV);  }
        else if (!acStartValid)   { set_control_state(PCS_STATE_FAULT_AC);  }
        else                      { set_control_state(PCS_STATE_FAULT_PCS); }
      }
      break;

    case PCS_STATE_RAMP:
      if (!chg_request)
      {
        stop_charge();
        set_control_state(PCS_STATE_STANDBY);
      }
      else if (!pcsCommsRecent)
      {
        chg_request = 0;
        stop_charge();
        set_control_state(PCS_STATE_FAULT_CAN);
      }
      else if (!hvValid)
      {
        chg_request = 0;
        stop_charge();
        set_control_state(PCS_STATE_FAULT_HV);
      }
      else if (pcsFault)
      {
        chg_request = 0;
        stop_charge();
        set_control_state(PCS_STATE_FAULT_PCS);
      }
      else if (!acHoldValid && elapsed_ms(stateEnteredMs, 3000U))
      {
        chg_request = 0;
        stop_charge();
        set_control_state(PCS_STATE_FAULT_AC);
      }
      else if (ramp_charge_power())
      {
        set_control_state(PCS_STATE_CHARGING);
      }
      break;

    case PCS_STATE_CHARGING:
      if (!chg_request)
      {
        stop_charge();
        set_control_state(PCS_STATE_STANDBY);
      }
      else if (!pcsCommsRecent || !hvValid || !acHoldValid || pcsFault)
      {
        chg_request = 0;
        stop_charge();
        if (!pcsCommsRecent)      { set_control_state(PCS_STATE_FAULT_CAN); }
        else if (!hvValid)        { set_control_state(PCS_STATE_FAULT_HV); }
        else if (!acHoldValid)    { set_control_state(PCS_STATE_FAULT_AC); }
        else                      { set_control_state(PCS_STATE_FAULT_PCS); }
      }
      else
      {
        /* Регистр можно менять во время зарядки в обе стороны. */
        ramp_charge_power();
      }
      break;

    case PCS_STATE_FAULT_CAN:
    case PCS_STATE_FAULT_HV:
    case PCS_STATE_FAULT_AC:
    case PCS_STATE_FAULT_PCS:
      stop_charge();
      if (chg_request)
      {
        set_control_state(PCS_STATE_STANDBY);
      }
      break;

    default:
      chg_request = 0;
      stop_charge();
      set_control_state(PCS_STATE_FAULT_CAN);
      break;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8] = {0};

  if (HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    return;
  }

  uint32_t now = HAL_GetTick();
  rxmess++;

  if ((RxHeader.StdId == 0x264U) && (RxHeader.DLC >= 6U))
  {
    PCS_ChargeLineStatus line;
    PCS_Decode264(RxData, &line);

    RX264 = 1;
    rx264Count++;
    last264Ms = now;
    lastPcsRxMs = now;
    ac_voltage_raw = line.voltageRaw;
    ac_current_raw = line.currentRaw;
    AClim = line.currentLimitAmps;
    ACpwr = (uint16_t)line.powerDeciKw * 100U; /* Display watts, 0.1 kW/bit. */
    ACvolts = line.voltageVolts;
    ACamps = line.currentAmps;
  }

  if ((RxHeader.StdId == 0x224U) && (RxHeader.DLC >= 7U))
  {
    RX224 = 1;
    rx224Count++;
    last224Ms = now;
    lastPcsRxMs = now;
    DCDCamps = (uint16_t)((((uint16_t)RxData[3] << 8 | RxData[2]) & 0x0FFFU) / 10U);
    pin_dcdc_status = RxData[6];
  }

  /* 0x2C4 belongs to the PCS. Never transmit a synthetic frame with this ID. */
  if ((RxHeader.StdId == 0x2C4U) && (RxHeader.DLC >= 8U))
  {
    uint8_t muxId = RxData[0];
    RX2C4 = 1;
    rx2C4Count++;
    lastPcsRxMs = now;

    if ((muxId == 0xE6U) || (muxId == 0xC6U))
    {
      HVvolts = (uint16_t)((((uint16_t)RxData[3] << 8 | RxData[2]) & 0x0FFFU) * 0.146484f);
      LVvolts = (uint16_t)((((uint16_t)RxData[1] << 9 | RxData[0]) >> 6) * 0.0390625f);
      last2C4Ms = now;
    }
    else if (muxId == 0x04U)
    {
      HVvolts = (uint16_t)(((((uint16_t)RxData[7] << 8 | RxData[6]) >> 3) & 0x0FFFU) * 0.146484f);
      last2C4Ms = now;
    }
  }

  if ((RxHeader.StdId == 0x204U) && (RxHeader.DLC >= 8U))
  {
    PCS_ChargerStatus status;
    PCS_Decode204(RxData, &status);

    rx204Count++;
    last204Ms = now;
    lastPcsRxMs = now;
    pcs_main_state = status.mainState;
    pcs_hv_charge_status = status.hvChargeStatus;
    pcs_grid_config = status.gridConfig;
    pcs_pwm_enable_line = status.pwmEnableLine;
    pcs_hw_variant = status.hardwareVariant;
    pcs_shutdown_request = status.shutdownRequest;
    pcs_status = status.mainState;
    pin_chg_status = status.pwmEnableLine;
  }

  if ((RxHeader.StdId == 0x3A4U) && (RxHeader.DLC >= 8U))
  {
    rx3A4Count++;
    lastPcsRxMs = now;
    pcs_alert_page = RxData[0] & 0x0FU;
    for (uint8_t i = 0U; i < 8U; i++)
    {
      pcsAlertMatrixLast[i] = RxData[i];
    }
  }

  if ((RxHeader.StdId == 0x424U) && (RxHeader.DLC >= 5U))
  {
    rx424Count++;
    lastPcsRxMs = now;
    pcs_last_alert_id = RxData[0];
    pcs_alert_matrix[PCS_AlertCnt & (PCS_ALERT_MATRIX_SIZE - 1U)] = RxData[0];
    PCS_AlertCnt = (uint8_t)((PCS_AlertCnt + 1U) & (PCS_ALERT_MATRIX_SIZE - 1U));

    if (RxData[0] == 0x1EU) /* CAN rationality alert */
    {
      pcs_alert_rx_error = RxData[2] & 0x07U;
      pcs_alert_can_id = ((uint16_t)RxData[4] << 8) | RxData[3];
      AlertRxError = pcs_alert_rx_error;
      AlertCANId = pcs_alert_can_id;

      if (pcs_alert_can_id == 0x2B2U)
      {
        if (pcs_alert_rx_error == 0x02U) { Short2B2 = false; } /* 3 bytes too short */
        if (pcs_alert_rx_error == 0x01U) { Short2B2 = true;  } /* 5 bytes too long */
      }
    }
  }
}

static void PCS_cksum_local(uint8_t *data, uint16_t id)
{
  uint16_t checksum_calc = 0;
  for(int b = 0; b < 7; b++)
  {
    checksum_calc += data[b];
  }
  // ИСПРАВЛЕНИЕ: Берем строго байты ID отдельно, а не складываем ID целиком
  checksum_calc += (uint8_t)(id & 0xFF);
  checksum_calc += (uint8_t)((id >> 8) & 0xFF);

  data[7] = (uint8_t)(checksum_calc & 0xFF);
}



void send_mes_10(void)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t buffer[8] = {0};

  pcs_counter = (pcs_counter + 1) & 0x0F;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.ExtId = 0;
  //-----------------------------------------------------------------------------------------





  /*
   * 0x2C4 is deliberately not transmitted here. It is a PCS-originated
   * logging frame; transmitting a synthetic 0x2C4 caused an ID collision and
   * hid the real HV measurement in the capture.
   */

  /* Post-2020 CP charge-status message and AC input current limit. */
  header.StdId = 0x13D;
  header.DLC = 6;
  PCS_Encode13D(buffer, ACILim,
      controlState == PCS_STATE_RAMP || controlState == PCS_STATE_CHARGING);
  can_queue_frame(&header, buffer);

  /* Hardware-validated 0x22A variant from the working local firmware. */
  header.StdId = 0x22A;
  header.DLC = 8;
  header.RTR = CAN_RTR_DATA;

  for (uint8_t i = 0U; i < 8U; i++)
  {
    buffer[i] = 0x00U;
  }

  /* 0x0B00 = проверенный на этом PCS запрос precharge 281,6 В. */
  buffer[0] = 0x00U;
  buffer[1] = 0x0BU;

  if (controlState == PCS_STATE_RAMP || controlState == PCS_STATE_CHARGING)
  {
    buffer[2] = 0x7DU;
  }
  else if (dcdc_w && chg_w) { buffer[2] = 0x7DU; }
  else if (dcdc_w)          { buffer[2] = 0x79U; }
  else if (chg_w)           { buffer[2] = 0x74U; }
  else                      { buffer[2] = 0x70U; }

  if (HVvolts > 50U)
  {
    buffer[3] = (uint8_t)((((HVvolts >> 8) & 0xFFU) << 4) |
                          ((HVvolts & 0xFFU) >> 4));
  }

  for (uint8_t i = 0U; i < 4U; i++)
  {
    dbg_tx22A[i] = buffer[i];
  }
  can_queue_frame(&header, buffer);

      // =========================================================================
        // 3. ОТПРАВКА ПАКЕТА 0x22D (DC-DC Control Profile)
        // =========================================================================
        header.StdId = 0x22D;
        header.DLC = 8;
        buffer[0] = 0x01;
        buffer[1] = VOLTAGE_DCDC & 0xFF;
        buffer[2] = (VOLTAGE_DCDC >> 8) & 0xFF;
        buffer[3] = 0x00;
        buffer[4] = 0x00;
        buffer[5] = 0x00;
        buffer[6] = pcs_counter;
        PCS_cksum_local(buffer, 0x22D);
        can_queue_frame(&header, buffer);

        // =========================================================================
          // 4. ОТПРАВКА ПАКЕТА 0x12D (Vehicle Mode Drive Readiness)
          // =========================================================================
          header.StdId = 0x12D;
          header.DLC = 8;
          buffer[0] = 0x03; buffer[1] = 0x00; buffer[2] = 0x00;
          buffer[3] = 0x00; buffer[4] = 0x00; buffer[5] = 0x00;
          buffer[6] = pcs_counter;
          PCS_cksum_local(buffer, 0x12D);
          can_queue_frame(&header, buffer);

  // 4. 0x115: Inverter Emulation Target
  //header.StdId = 0x115;
  //buffer[0] = 0x05; buffer[1] = 0x20; buffer[2] = 0x00;
  //buffer[3] = 0x00; buffer[4] = 0x00; buffer[5] = 0x00;
  //buffer[6] = pcs_counter;
  //PCS_cksum_local(buffer, 0x115);
  //can_queue_frame(&header, buffer);



            // =========================================================================
            // 5. BMS log2 message  This msg changed drastically between 2019 and 2020-2021 model firmwares. PCS pays close attention to these two muxes.
            // =========================================================================
            header.StdId = 0x3B2;
            header.DLC = 8;


            if(mux3b2)
            {
            	   buffer[0]=0xE5;  //mux 5=charging.
            	   buffer[1]=0x0D;
            	   buffer[2]=0xEB;
            	   buffer[3]=0xFF;
            	   buffer[4]=0x0C;
            	   buffer[5]=0x66;
            	   buffer[6]=0xBB;
            	   buffer[7]=0x11;

                   can_queue_frame(&header, buffer);

            	mux3b2 = false;
            }
            else
            {
            	buffer[0]=0xE3;  //mux 3=charge termination
            	buffer[1]=0x5D;
            	buffer[2]=0xFB;
            	buffer[3]=0xFF;
            	buffer[4]=0x0C;
            	buffer[5]=0x66;
            	buffer[6]=0xBB;
            	buffer[7]=0x06;

                can_queue_frame(&header, buffer);

            	mux3b2 = true;
            }



}

void send_mes_50(void)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t buffer[8] = {0};

  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.ExtId = 0;
  //===============================================



  // =========================================================================
  // 5. VCFront
  // =========================================================================
  header.StdId = 0x545;
  header.DLC = 8;

  if(mux545)
  {
    buffer[0] = 0x14;
    buffer[1] = 0x00;
    buffer[2] = 0x3F;
    buffer[3] = 0x70;
    buffer[4] = 0x9F;
    buffer[5] = 0x01;
    buffer[6] =((Count545 << 4) | 0xA);

    PCS_cksum_local(buffer, 0x545);

    can_queue_frame(&header, buffer);

    mux545 = false;
  }
  else
  {
    buffer[0] = 0x03;
    buffer[1] = 0x19;
    buffer[2] = 0x64;
    buffer[3] = 0x32;
    buffer[4] = 0x19;
    buffer[5] = 0x00;
    buffer[6] = (Count545 << 4);

    PCS_cksum_local(buffer, 0x545);

    can_queue_frame(&header, buffer);

    mux545 = true;
  }

  Count545++;
  if(Count545 > 0x0F) Count545 = 0;



}//======END SEND 50ms==============================================

void send_mes_100(void)
{

  CAN_TxHeaderTypeDef header = {0};
  uint8_t buffer[8] = {0};



  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.ExtId = 0;
  //------------------------------------------------------------



  if (controlState == PCS_STATE_RAMP || controlState == PCS_STATE_CHARGING)
    {
      // Передаем переменную CHGpwr, которая плавно растет в автомате состояний,
      // и флаг активности "1".
      PCS_Send_Power_Request_US(CHGpwr, 1);
    }
    else
    {
      // В режимах Standby, Fault или Boot жестко просим 0 Ватт и шлем команду "0"
      PCS_Send_Power_Request_US(0, 0);
    }








                  // =========================================================================
                  // 5. UI charge request message. Can be used as an ac current limit.
                  // =========================================================================
                  header.StdId = 0x333;
                  header.DLC = 4;
                  PCS_Encode333(buffer, ACILim);
                  can_queue_frame(&header, buffer);


  header.StdId = 0x115;
  header.DLC = 8;
  header.RTR = CAN_RTR_DATA;
  buffer[0] = 0x20; // Режим Паркинг (Park)
  buffer[1] = 0x00;
  buffer[2] = 0x01; // Цепь безопасности замкнута
  buffer[3] = 0x00;
  buffer[4] = 0x00;
  buffer[5] = 0x00;
  buffer[6] = 0x00;
  buffer[7] = 0x00;
  can_queue_frame(&header, buffer);



        // =========================================================================
        // 5. BMS Contactor request.Static msg.
        // =========================================================================
        header.StdId = 0x232;
        header.DLC = 8;

        buffer[0]=0x0A;
        buffer[1]=0x02;
        buffer[2]=0xD5;
        buffer[3]=0x09;
        buffer[4]=0xCB;
        buffer[5]=0x04;
        buffer[6]=0x00;
        buffer[7]=0x00;

        can_queue_frame(&header, buffer);
















          // =========================================================================
          // 5. VCFront vehicle status This message contains the 12v dcdc target setpoint. bits 16-26 as an 11bit unsigned int. scale 0.01
          // =========================================================================
          header.StdId = 0x3A1;
          header.DLC = 8;

          buffer[0] = 0x09;
          buffer[1] = 0x62;
          buffer[2] = VOLTAGE_DCDC & 0xFF;//78 , d gives us a 14v target 0x78;
          buffer[3] = ((VOLTAGE_DCDC >> 8)|0x99);//0x9D;0x9D;
          buffer[4] = 0x08;
          buffer[5] = 0x2C;
          buffer[6] = 0x12;
          buffer[7] = 0x5A;
          can_queue_frame(&header, buffer);



  // =========================================================================
  // 5. VCFront sensors. Static.
  // =========================================================================
  header.StdId = 0x321;
  header.DLC = 8;
  buffer[0] = 0x2C;
  buffer[1] = 0xB6;
  buffer[2] = 0xA8;
  buffer[3] = 0x7F;
  buffer[4] = 0x02;
  buffer[5] = 0x7F;
  buffer[6] = 0x00;
  buffer[7] = 0x00;
  can_queue_frame(&header, buffer);





    // =========================================================================
    // 5. CP Status. Only byte 0 bits 0 and 1 are important to the PCS
    // =========================================================================
    header.StdId = 0x25D;
    header.DLC = 8;



    //buffer[0] = 0xD8; // ИСПРАВЛЕНО: Байт US-порта
    //buffer[1] = 0x8C;
    //buffer[2] = 0x01;
    //buffer[3] = 0xB5;

    buffer[0]=0xD8; //D9 FOR EURO. D8 FOR US. 0=US Tesla , 1=Euro IEC , 2=GB, 3=IEC CCS. Must match PCS type!
    buffer[1]=0x8C;
    buffer[2]=0x01;
    buffer[3]=0xB5;
    buffer[4]=0x4A;
    buffer[5]=0xC1;
    buffer[6]=0x0A;
    buffer[7]=0xE0;
    can_queue_frame(&header, buffer);





    /* US charge-port emulation: advertise a real non-zero 16 A limit. */
    header.StdId = 0x23D;
    header.DLC = 2;
    PCS_Encode23D_US(buffer, ACILim);
    dbg_tx23D[0] = buffer[0];
    dbg_tx23D[1] = buffer[1];
    can_queue_frame(&header, buffer);

    /* Tested US Type-1/NACS connected profile. 0x60 is not a 60 Hz field. */
    header.StdId = 0x21D;
    header.DLC = 8;
    PCS_Encode21D_US(buffer, ACILim);
    can_queue_frame(&header, buffer);


      // =========================================================================
      // 5. ОТПРАВКА ПАКЕТА 0x20A (BMS Log Telemetry)
      // =========================================================================
      header.StdId = 0x20A;
      header.DLC = 6;
      buffer[0] = 0xF6;//0x55; // Статус BMS: Контакторы замкнуты
      buffer[1] = 0x15;//0x00;
      buffer[2] = 0x09;//0xD0; // Разрешение на разряд/заряд силовой сети
      buffer[3] = 0x82;//0x07; // Флаг готовности высоковольтной системы к току
      buffer[4] = 0x18;//0x00;
      buffer[5] = 0x01;//0x00;
      //buffer[6] = 0x00;
      //buffer[7] = 0x00;

      can_queue_frame(&header, buffer);


      // =========================================================================
      // 5. BMS status. Static msg
      // =========================================================================
      header.StdId = 0x212;
      header.DLC = 8;
      buffer[0] = 0xB9;
      buffer[1] = 0x1C;
      buffer[2] = 0x94;
      buffer[3] = 0xAD;
      buffer[4] = 0xC3;//0xC1;
      buffer[5] = 0x15;
      buffer[6] = 0x06;//0x36;
      buffer[7] = 0x63;//0x6B;
      can_queue_frame(&header, buffer);



      }
//Конец отправки CAN пакетов========================================================================





// ТАЙМЕР TIM_1 10msec========================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static uint8_t secondTicks = 0;
  if(htim->Instance != TIM1) { return; }

  if (task10ms) { schedulerMisses++; }
  task10ms = true;

  msec_50++;
  if(msec_50 >= 5U)
  {
    msec_50 = 0;
    if (task50ms) { schedulerMisses++; }
    task50ms = true;
  }

  msec_100++;
  if(msec_100 >= 10U)
  {
    msec_100 = 0;
    if (task100ms) { schedulerMisses++; }
    task100ms = true;

    if(t_buzzer != 0U)
    {
      t_buzzer--;
      if(t_buzzer == 0U) { buzzer_off; }
    }

    msec++;
    if(msec >= 25U)
    {
      msec = 0;
      upd_disp = 1;
    }

    secondTicks++;
    if(secondTicks >= 10U)
    {
      secondTicks = 0;
      seconds++;
    }
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *canHandle)
{
  canErrorEvents++;
  canLastError = HAL_CAN_GetError(canHandle);
  canLastEsr = canHandle->Instance->ESR;
  HAL_CAN_ResetError(canHandle);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//=====================================================================================
  pcs_w = 1; pcs_on;
    dcdc_w = 1; dcdc_on;
    chg_w = 0; chg_off;

    CHGpwr = 0;
    update_charge_setpoints();
    Short2B2 = USpcs;    // US/older PCS starts with DLC=3; alert 0x424 can switch to DLC=5.

    buzzer_on; t_buzzer = 2;

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    if (HAL_CAN_Start(&hcan) != HAL_OK) { Error_Handler(); }
    if (HAL_CAN_ActivateNotification(&hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_Delay(10);
    __enable_irq();

    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
    led_level = 50;

    HAL_Delay(200);
    ILI9341_Init();
    ILI9341_FillScreen(ILI9341_BLACK);
    set_control_state(PCS_STATE_BOOT);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    ILI9341_WriteString(90, 0, "PCS", Font_16x26, ILI9341_GREEN, ILI9341_BLACK);
    ILI9341_WriteString(0, 30, "PCS:", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 50, "DC-DC:", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 70, "CHARGER:", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 90, "I set/PCS=", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 110, "AC power=", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 130, "AC volts=", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 150, "AC amps=", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 170, "DC-DC amps=", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 190, "U hv=", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 210, "U lv=", Font_11x18, ILI9341_WHITE, ILI9341_BLACK);
    ILI9341_WriteString(0, 230, "STATE=", Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    ILI9341_WriteString(0, 245, "Pwrst=", Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    ILI9341_WriteString(0, 260, "RX=", Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    ILI9341_WriteString(120, 260, "ERR=", Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    ILI9341_WriteString(0, 275, "QOV=", Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    ILI9341_WriteString(120, 275, "MISS=", Font_7x10, ILI9341_CYAN, ILI9341_BLACK);
    ILI9341_WriteString(0, 290, "264 224 2C4 M H G P AL", Font_7x10, ILI9341_CYAN, ILI9341_BLACK);





	while (1)
	{
		  _Bool run10 = false;
			  _Bool run50 = false;
			  _Bool run100 = false;

			  // Безопасно забираем флаги таймеров из прерывания, сбрасывая ТОЛЬКО те, что реально сработали
			  __disable_irq();
			  if (task10ms)  { run10 = true;  task10ms = false;  }
			  if (task50ms)  { run50 = true;  task50ms = false;  }
			  if (task100ms) { run100 = true; task100ms = false; }
			  __enable_irq();

			  // 1. Исполнение быстрых 10мс задач (Отправка пакетов 0x22A, 0x13D, 0x22D)
			  if (run10)
			  {
			    send_mes_10();
			  }

			  // 2. Исполнение 50мс задач (Эмуляция VCFront 0x545)
			  if (run50)
			  {
			    send_mes_50();
			  }

			  // 3. Исполнение медленных 100мс задач (Автомат состояний, Мощность 0x2B2, Лимиты 0x21D)
			  if (run100)
			  {
			    control_step_100ms();
			    send_mes_100();
			  }

			  // Проверяем и обслуживаем кольцевую очередь отправки CAN
			  can_service_tx();

	  if(upd_disp==1)    //если пришло время обновить дисплей
	  {

		  get_HV();

          if(pcs_w==1){ILI9341_WriteString(90,30,"ON ",Font_11x18,ILI9341_WHITE,ILI9341_BLACK);}else{ILI9341_WriteString(90,30,"OFF",Font_11x18,ILI9341_WHITE,ILI9341_BLACK);};
          if(dcdc_w==1){ILI9341_WriteString(90,50,"ON ",Font_11x18,ILI9341_WHITE,ILI9341_BLACK);}else{ILI9341_WriteString(90,50,"OFF",Font_11x18,ILI9341_WHITE,ILI9341_BLACK);};
          if(chg_w==1){ILI9341_WriteString(90,70,"ON ",Font_11x18,ILI9341_WHITE,ILI9341_BLACK);}else{ILI9341_WriteString(90,70,"OFF",Font_11x18,ILI9341_WHITE,ILI9341_BLACK);};

          ILI9341_WriteString(160,0,"   %",Font_16x26,ILI9341_CYAN,ILI9341_BLACK);
          ILI9341_DrawChar(160,0,cell_SOC, Font_16x26, ILI9341_CYAN,ILI9341_BLACK);

          ILI9341_WriteString(110,90,"          ", Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_WriteString(100,110,"   ", Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_WriteString(100,130,"   ", Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_WriteString(100,150,"   ", Font_11x18, ILI9341_WHITE,ILI9341_BLACK);

          ILI9341_DrawChar(110,90,CHGcurrentAppliedA, Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_WriteString(150,90,"/", Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_DrawChar(170,90,AClim, Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_DrawChar(100,110,ACpwr, Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_DrawChar(100,130,ACvolts, Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_DrawChar(100,150,ACamps, Font_11x18, ILI9341_WHITE,ILI9341_BLACK);

          ILI9341_DrawChar(130,170,DCDCamps, Font_11x18, ILI9341_WHITE,ILI9341_BLACK);

          ILI9341_DrawChar(100,190,HVvolts, Font_11x18, ILI9341_WHITE,ILI9341_BLACK);
          ILI9341_DrawChar(100,210,LVvolts, Font_11x18, ILI9341_WHITE,ILI9341_BLACK);


	          ILI9341_WriteString(55,230,"    ",Font_7x10,ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_DrawChar(55,230,controlState, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_WriteString(55,245,"     ",Font_7x10,ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_DrawChar(55,245,CHGpwr, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_WriteString(25,260,"          ",Font_7x10,ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_DrawChar(25,260,rxmess, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_WriteString(150,260,"        ",Font_7x10,ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_DrawChar(150,260,canLastError, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_WriteString(30,275,"        ",Font_7x10,ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_DrawChar(30,275,canTxQueueOverflows, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_WriteString(155,275,"        ",Font_7x10,ILI9341_WHITE,ILI9341_BLACK);
	          ILI9341_DrawChar(155,275,schedulerMisses, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);

		          ILI9341_WriteString(0,305,"                                  ",Font_7x10,ILI9341_WHITE,ILI9341_BLACK);
		          ILI9341_DrawChar(0,305,RX264, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
		          ILI9341_DrawChar(28,305,RX224, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
		          ILI9341_DrawChar(56,305,RX2C4, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
		          ILI9341_DrawChar(84,305,pcs_main_state, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
		          ILI9341_DrawChar(105,305,pcs_hv_charge_status, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
		          ILI9341_DrawChar(126,305,pcs_grid_config, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
		          ILI9341_DrawChar(147,305,pcs_pwm_enable_line, Font_7x10, ILI9341_WHITE,ILI9341_BLACK);
		          ILI9341_DrawChar(182,305,pcs_last_alert_id, Font_7x10, ILI9341_YELLOW,ILI9341_BLACK);

          RX264=0;RX224=0;RX2C4=0;

		upd_disp=0;
	  };



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    //sFilterConfig.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
    Error_Handler();
    }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 639;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 18;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PIN_CHG_Pin|PIN_DCDC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PIN_PilotDown_GPIO_Port, PIN_PilotDown_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PIN_Res_Pin|PIN_DC_Pin|PIN_RS_Pin|PIN_Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PIN_CS_Pin|PIN_Out2_Pin|PIN_Out1_Pin|PIN_PCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PIN_CHG_Pin PIN_DCDC_Pin */
  GPIO_InitStruct.Pin = PIN_CHG_Pin|PIN_DCDC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PIN_PilotDown_Pin */
  GPIO_InitStruct.Pin = PIN_PilotDown_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PIN_PilotDown_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_Res_Pin PIN_DC_Pin PIN_RS_Pin PIN_Buzzer_Pin */
  GPIO_InitStruct.Pin = PIN_Res_Pin|PIN_DC_Pin|PIN_RS_Pin|PIN_Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_CS_Pin PIN_Out2_Pin PIN_Out1_Pin PIN_PCS_Pin */
  GPIO_InitStruct.Pin = PIN_CS_Pin|PIN_Out2_Pin|PIN_Out1_Pin|PIN_PCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_In2_Pin PIN_In1_Pin */
  GPIO_InitStruct.Pin = PIN_In2_Pin|PIN_In1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PIN_Sb_up_Pin PIN_Sb_down_Pin PIN_Sb_left_Pin PIN_Sb_right_Pin
                           PIN_Sb_ok_Pin PIN_power_on_Pin */
  GPIO_InitStruct.Pin = PIN_Sb_up_Pin|PIN_Sb_down_Pin|PIN_Sb_left_Pin|PIN_Sb_right_Pin
                          |PIN_Sb_ok_Pin|PIN_power_on_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
