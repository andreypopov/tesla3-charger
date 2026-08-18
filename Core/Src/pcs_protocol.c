#include "pcs_protocol.h"

static uint8_t half_amp_limit(uint8_t currentLimitAmps)
{
  uint16_t encoded = (uint16_t)currentLimitAmps * 2U;
  return (encoded > 0xFEU) ? 0xFEU : (uint8_t)encoded;
}

void PCS_Decode204(const uint8_t data[8], PCS_ChargerStatus *status)
{
  status->mainState = data[0] & 0x0FU;
  status->hvChargeStatus = (data[0] >> 4) & 0x03U;
  status->gridConfig = (data[0] >> 6) & 0x03U;
  status->instantAcPowerDeciKw = data[2];
  status->maximumAcPowerDeciKw = data[3];
  status->phaseAEnabled = data[1] & 0x01U;
  status->phaseBEnabled = (data[1] >> 1) & 0x01U;
  status->phaseCEnabled = (data[1] >> 2) & 0x01U;
  status->phaseACurrentRequestDeciAmps = data[4];
  status->phaseBCurrentRequestDeciAmps = data[5];
  status->phaseCCurrentRequestDeciAmps = data[6];
  status->pwmEnableLine = data[7] & 0x01U;
  status->shutdownRequest = (data[7] >> 1) & 0x03U;
  status->hardwareVariant = (data[7] >> 3) & 0x03U;
}

void PCS_Decode264(const uint8_t data[6], PCS_ChargeLineStatus *status)
{
  uint16_t lineWord = ((uint16_t)data[2] << 8) | data[1];

  status->voltageRaw = (((uint16_t)data[1] << 8) | data[0]) & 0x3FFFU;
  status->currentRaw = (lineWord >> 6) & 0x01FFU;
  status->currentLimitRaw = (((uint16_t)data[5] << 8) | data[4]) & 0x03FFU;

  /* DBC scales: voltage 0.0333 V/bit; currents 0.1 A/bit. */
  status->voltageVolts = (uint16_t)(((uint32_t)status->voltageRaw * 333U + 5000U) / 10000U);
  status->currentAmps = (uint16_t)((status->currentRaw + 5U) / 10U);
  status->currentLimitAmps = (uint16_t)((status->currentLimitRaw + 5U) / 10U);
  status->powerDeciKw = data[3];
}

void PCS_Decode2B4(const uint8_t data[5], PCS_DcdcRailStatus *status)
{
  uint32_t railWord = (uint32_t)data[0] |
                      ((uint32_t)data[1] << 8) |
                      ((uint32_t)data[2] << 16);

  status->lvVoltageRaw = (uint16_t)(railWord & 0x03FFU);
  status->hvVoltageRaw = (uint16_t)((railWord >> 10) & 0x0FFFU);
  status->outputCurrentRaw =
      (uint16_t)((((uint16_t)data[4] << 8) | data[3]) & 0x0FFFU);

  /* DBC scales: LV 5/128 V, HV 75/512 V and current 0.1 A per bit. */
  status->lvVoltageVolts =
      (uint16_t)(((uint32_t)status->lvVoltageRaw * 5U + 64U) / 128U);
  status->hvVoltageVolts =
      (uint16_t)(((uint32_t)status->hvVoltageRaw * 75U + 256U) / 512U);
  status->outputCurrentAmps = (uint16_t)((status->outputCurrentRaw + 5U) / 10U);
}

bool PCS_Decode76CChargePhase(const uint8_t data[8], PCS_ChargePhaseDebug *status)
{
  uint8_t phaseIndex;

  if (data[0] == 0x0CU)      { phaseIndex = 0U; }
  else if (data[0] == 0x16U) { phaseIndex = 1U; }
  else if (data[0] == 0x20U) { phaseIndex = 2U; }
  else                       { return false; }

  status->muxId = data[0];
  status->phaseIndex = phaseIndex;
  status->currentRaw = (((uint16_t)data[2] << 8) | data[1]) & 0x03FFU;
  /* Reference controller scale: 0.0025 A/bit, retained as rounded mA. */
  status->currentMilliAmps =
      (uint16_t)(((uint32_t)status->currentRaw * 5U + 1U) / 2U);
  return true;
}

void PCS_Encode13D(uint8_t data[6], uint8_t currentLimitAmps, bool chargeEnabled)
{
  /* Post-2020 CP charge-status format used by the reference controller. */
  data[0] = chargeEnabled ? 0x05U : 0x0AU;
  data[1] = half_amp_limit(currentLimitAmps);
  data[2] = 0xAAU;
  data[3] = 0x1AU;
  data[4] = 0xFFU;
  data[5] = 0x02U;
}

void PCS_Encode21D(uint8_t data[8], uint8_t pilotCurrentAmps,
                   uint8_t cableCurrentLimitAmps, PCS_ChargePortProfile profile)
{
  /*
   * 0x2D is the valid EU line-charge pilot profile used by the newer reference
   * controller. 0x5D preserves the older US Type-1/NACS connected profile.
   * The pilot is the live limit; byte 3 is only the cable capability.
   */
  data[0] = (profile == PCS_CHARGE_PORT_EU) ? 0x2DU : 0x5DU;
  data[1] = half_amp_limit(pilotCurrentAmps);
  data[2] = 0x00U;
  data[3] = (cableCurrentLimitAmps > 0x7FU) ? 0x7FU : cableCurrentLimitAmps;
  data[4] = 0x80U;
  data[5] = 0x00U;
  data[6] = 0x60U;
  data[7] = 0x10U;
}

void PCS_Encode22A(uint8_t data[4], uint16_t hvVolts, bool dcdcEnabled, bool chargeEnabled)
{
  uint16_t measuredHv = hvVolts & 0x07FFU;
  uint8_t mode = 0x00U;

  if (dcdcEnabled && chargeEnabled)      { mode = 0x0DU; }
  else if (dcdcEnabled)                  { mode = 0x09U; }
  else if (chargeEnabled)                { mode = 0x05U; }

  /* No precharge-voltage request. Bits 16..19 carry mode and HW enables. */
  data[0] = 0x00U;
  data[1] = 0x00U;
  data[2] = (uint8_t)(((measuredHv & 0x0FU) << 4) | mode);
  data[3] = (uint8_t)((measuredHv >> 4) & 0x7FU);
}

void PCS_Encode23D(uint8_t data[4], uint8_t currentLimitAmps, bool chargeEnabled)
{
  /*
   * Post-2020 CP charge status. The two-byte US frame belongs to older PCS
   * firmware. The attached trace confirms this PCS accepts the four-byte
   * layout; the remaining 8 A clamp must be diagnosed independently.
   */
  data[0] = chargeEnabled ? 0x05U : 0x0AU;
  data[1] = half_amp_limit(currentLimitAmps);
  data[2] = 0xFFU;
  data[3] = 0x0FU;
}

void PCS_Encode25D(uint8_t data[8], PCS_ChargePortProfile profile)
{
  /* CP_type in the low two bits: 0 = US Tesla, 1 = Euro IEC. */
  data[0] = (profile == PCS_CHARGE_PORT_EU) ? 0xD9U : 0xD8U;
  data[1] = 0x8CU;
  data[2] = 0x01U;
  data[3] = 0xB5U;
  data[4] = 0x4AU;
  data[5] = 0xC1U;
  data[6] = 0x0AU;
  data[7] = 0xE0U;
}

void PCS_Encode333(uint8_t data[4], uint8_t currentLimitAmps)
{
  data[0] = 0x04U;
  data[1] = (currentLimitAmps > 0x7FU) ? 0x7FU : currentLimitAmps;
  data[2] = 0x29U;
  data[3] = 0x07U;
}

uint8_t PCS_Encode2B2(uint8_t data[5], uint16_t powerWatts, bool chargeEnabled, bool shortFrame)
{
  data[0] = (uint8_t)(powerWatts & 0xFFU);
  data[1] = (uint8_t)((powerWatts >> 8) & 0xFFU);
  data[2] = chargeEnabled ? 0x02U : 0x00U;
  data[3] = 0x00U;
  data[4] = 0x00U;
  return shortFrame ? 3U : 5U;
}

uint8_t PCS_ClampChargeCurrent(uint8_t requestedAmps, uint8_t maximumAmps)
{
  return (requestedAmps > maximumAmps) ? maximumAmps : requestedAmps;
}

uint16_t PCS_CalculateChargePowerTarget(uint8_t currentAmps, uint16_t acVolts,
                                        uint16_t maximumWatts)
{
  uint32_t requestedWatts = (uint32_t)currentAmps * (uint32_t)acVolts;

  if (requestedWatts > maximumWatts)
  {
    requestedWatts = maximumWatts;
  }

  return (uint16_t)requestedWatts;
}

bool PCS_IsChargeOverCurrent(uint16_t measuredAmps, uint8_t setpointAmps,
                             uint8_t marginAmps)
{
  uint16_t tripThreshold = (uint16_t)setpointAmps + (uint16_t)marginAmps;
  return (setpointAmps > 0U) && (measuredAmps > tripThreshold);
}
