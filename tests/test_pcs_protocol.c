#include "pcs_protocol.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
  const uint8_t captured204[8] = {0x78, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x09};
  const uint8_t charging204[8] = {0x66, 0x04, 0x03, 0xFF, 0x00, 0x00, 0x50, 0x09};
  const uint8_t captured264[6] = {0xC3, 0x19, 0x80, 0x00, 0x40, 0x01};
  const uint8_t charging264[6] = {0xD2, 0x19, 0x94, 0x12, 0xA0, 0x00};
  const uint8_t charging2B4[5] = {0x6C, 0x31, 0x21, 0x79, 0x00};
  PCS_ChargerStatus charger;
  PCS_ChargeLineStatus line;
  PCS_DcdcRailStatus dcdc;
  uint8_t data[8] = {0};

  PCS_Decode204(captured204, &charger);
  assert(charger.mainState == 8U);
  assert(charger.hvChargeStatus == 3U);
  assert(charger.gridConfig == 1U);
  assert(charger.pwmEnableLine == 1U);
  assert(charger.hardwareVariant == 1U);

  PCS_Decode204(charging204, &charger);
  assert(charger.mainState == 6U && charger.hvChargeStatus == 2U);
  assert(charger.gridConfig == 1U && charger.phaseCEnabled == 1U);
  assert(charger.instantAcPowerDeciKw == 3U);
  assert(charger.maximumAcPowerDeciKw == 0xFFU);
  assert(charger.phaseACurrentRequestDeciAmps == 0U);
  assert(charger.phaseBCurrentRequestDeciAmps == 0U);
  assert(charger.phaseCCurrentRequestDeciAmps == 80U);

  PCS_Decode264(captured264, &line);
  assert(line.voltageVolts == 220U);
  assert(line.currentRaw == 0U);
  assert(line.currentAmps == 0U);
  assert(line.powerDeciKw == 0U);
  assert(line.currentLimitAmps == 32U);

  PCS_Decode264(charging264, &line);
  assert(line.voltageVolts == 220U);
  assert(line.currentRaw == 80U && line.currentAmps == 8U);
  assert(line.powerDeciKw == 18U);
  assert(line.currentLimitRaw == 160U && line.currentLimitAmps == 16U);

  PCS_Decode2B4(charging2B4, &dcdc);
  assert(dcdc.lvVoltageRaw == 364U && dcdc.lvVoltageVolts == 14U);
  assert(dcdc.hvVoltageRaw == 2124U && dcdc.hvVoltageVolts == 311U);
  assert(dcdc.outputCurrentRaw == 121U && dcdc.outputCurrentAmps == 12U);

  PCS_Encode22A(data, 319U, true, true);
  assert(data[0] == 0x00U && data[1] == 0x00U);
  assert(data[2] == 0xFDU && data[3] == 0x13U);

  assert(PCS_Encode2B2(data, 3000U, true, true) == 3U);
  assert(data[0] == 0xB8U && data[1] == 0x0BU && data[2] == 0x02U);

  PCS_Encode23D(data, 16U, true);
  assert(data[0] == 0x05U && data[1] == 0x20U);
  assert(data[2] == 0xFFU && data[3] == 0x0FU);

  PCS_Encode23D(data, 16U, false);
  assert(data[0] == 0x0AU && data[1] == 0x20U);
  assert(data[2] == 0xFFU && data[3] == 0x0FU);

  PCS_Encode21D_US(data, 16U, 32U);
  assert(data[0] == 0x5DU && data[1] == 0x20U && data[2] == 0x00U && data[3] == 0x20U);
  assert(data[4] == 0x80U && data[5] == 0x00U && data[6] == 0x60U && data[7] == 0x10U);

  PCS_Encode13D(data, 16U, true);
  assert(data[0] == 0x05U && data[1] == 0x20U && data[2] == 0xAAU);
  assert(data[3] == 0x1AU && data[4] == 0xFFU && data[5] == 0x02U);

  PCS_Encode13D(data, 8U, true);
  assert(data[1] == 0x10U);
  PCS_Encode21D_US(data, 8U, 32U);
  assert(data[1] == 0x10U && data[3] == 0x20U);
  PCS_Encode23D(data, 8U, true);
  assert(data[0] == 0x05U && data[1] == 0x10U);
  PCS_Encode333(data, 48U);
  assert(data[0] == 0x04U && data[1] == 0x30U);
  assert(data[2] == 0x29U && data[3] == 0x07U);

  assert(PCS_ClampChargeCurrent(8U, 16U) == 8U);
  assert(PCS_ClampChargeCurrent(16U, 16U) == 16U);
  assert(PCS_ClampChargeCurrent(25U, 16U) == 16U);

  assert(PCS_CalculateChargePowerTarget(8U, 220U, 4000U) == 1760U);
  assert(PCS_CalculateChargePowerTarget(16U, 220U, 4000U) == 3520U);
  assert(PCS_CalculateChargePowerTarget(16U, 260U, 4000U) == 4000U);

  assert(PCS_ChargePowerMultiplierForHardwareVariant(0U) == 1U);
  assert(PCS_ChargePowerMultiplierForHardwareVariant(1U) == 2U);
  assert(PCS_ChargePowerMultiplierForHardwareVariant(2U) == 1U);
  assert(PCS_ScaleChargePowerRequest(3472U, 2U, 8000U) == 6944U);
  assert(PCS_Encode2B2(data, 6944U, true, false) == 5U);
  assert(data[0] == 0x20U && data[1] == 0x1BU && data[2] == 0x02U);
  assert(data[3] == 0x00U && data[4] == 0x00U);
  assert(PCS_ScaleChargePowerRequest(4000U, 2U, 8000U) == 8000U);
  assert(PCS_ScaleChargePowerRequest(500U, 1U, 8000U) == 500U);
  assert(PCS_ScaleChargePowerRequest(5000U, 2U, 8000U) == 8000U);
  assert(PCS_ScaleChargePowerRequest(500U, 0U, 8000U) == 500U);
  assert(!PCS_IsChargeOverCurrent(18U, 16U, 2U));
  assert(PCS_IsChargeOverCurrent(19U, 16U, 2U));
  assert(!PCS_IsChargeOverCurrent(32U, 0U, 2U));

  puts("PCS protocol tests: OK");
  return 0;
}
