#include "pcs_protocol.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
  const uint8_t captured204[8] = {0x78, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x09};
  const uint8_t captured264[6] = {0xC3, 0x19, 0x80, 0x00, 0x40, 0x01};
  PCS_ChargerStatus charger;
  PCS_ChargeLineStatus line;
  uint8_t data[8] = {0};

  PCS_Decode204(captured204, &charger);
  assert(charger.mainState == 8U);
  assert(charger.hvChargeStatus == 3U);
  assert(charger.gridConfig == 1U);
  assert(charger.pwmEnableLine == 1U);
  assert(charger.hardwareVariant == 1U);

  PCS_Decode264(captured264, &line);
  assert(line.voltageVolts == 220U);
  assert(line.currentRaw == 0U);
  assert(line.currentAmps == 0U);
  assert(line.powerDeciKw == 0U);
  assert(line.currentLimitAmps == 32U);

  PCS_Encode22A(data, 319U, true, true);
  assert(data[0] == 0x00U && data[1] == 0x00U);
  assert(data[2] == 0xFDU && data[3] == 0x13U);

  assert(PCS_Encode2B2(data, 3000U, true, true) == 3U);
  assert(data[0] == 0xB8U && data[1] == 0x0BU && data[2] == 0x02U);

  PCS_Encode23D_US(data, 16U);
  assert(data[0] == 0x20U && data[1] == 0x00U);

  PCS_Encode21D_US(data, 16U);
  assert(data[0] == 0x5DU && data[1] == 0x20U && data[2] == 0x00U && data[3] == 0x10U);
  assert(data[4] == 0x80U && data[5] == 0x00U && data[6] == 0x60U && data[7] == 0x10U);

  PCS_Encode13D(data, 16U, true);
  assert(data[0] == 0x05U && data[1] == 0x20U && data[2] == 0xAAU);
  assert(data[3] == 0x1AU && data[4] == 0xFFU && data[5] == 0x02U);

  PCS_Encode13D(data, 8U, true);
  assert(data[1] == 0x10U);
  PCS_Encode21D_US(data, 8U);
  assert(data[1] == 0x10U && data[3] == 0x08U);
  PCS_Encode23D_US(data, 8U);
  assert(data[0] == 0x10U);
  PCS_Encode333(data, 8U);
  assert(data[0] == 0x04U && data[1] == 0x08U);
  assert(data[2] == 0x29U && data[3] == 0x07U);

  assert(PCS_ClampChargeCurrent(8U, 16U) == 8U);
  assert(PCS_ClampChargeCurrent(16U, 16U) == 16U);
  assert(PCS_ClampChargeCurrent(25U, 16U) == 16U);

  assert(PCS_CalculateChargePowerTarget(8U, 220U, 4000U) == 1760U);
  assert(PCS_CalculateChargePowerTarget(16U, 220U, 4000U) == 3520U);
  assert(PCS_CalculateChargePowerTarget(16U, 260U, 4000U) == 4000U);

  puts("PCS protocol tests: OK");
  return 0;
}
