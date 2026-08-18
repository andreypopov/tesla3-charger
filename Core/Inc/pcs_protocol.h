#ifndef PCS_PROTOCOL_H
#define PCS_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t mainState;
  uint8_t hvChargeStatus;
  uint8_t gridConfig;
  uint8_t phaseAEnabled;
  uint8_t phaseBEnabled;
  uint8_t phaseCEnabled;
  uint8_t pwmEnableLine;
  uint8_t shutdownRequest;
  uint8_t hardwareVariant;
} PCS_ChargerStatus;

typedef struct
{
  uint16_t voltageRaw;
  uint16_t currentRaw;
  uint16_t currentLimitRaw;
  uint16_t voltageVolts;
  uint16_t currentAmps;
  uint16_t currentLimitAmps;
  uint8_t powerDeciKw;
} PCS_ChargeLineStatus;

void PCS_Decode204(const uint8_t data[8], PCS_ChargerStatus *status);
void PCS_Decode264(const uint8_t data[6], PCS_ChargeLineStatus *status);

void PCS_Encode13D(uint8_t data[6], uint8_t currentLimitAmps, bool chargeEnabled);
void PCS_Encode21D_US(uint8_t data[8], uint8_t currentLimitAmps);
void PCS_Encode22A(uint8_t data[4], uint16_t hvVolts, bool dcdcEnabled, bool chargeEnabled);
void PCS_Encode23D_US(uint8_t data[2], uint8_t currentLimitAmps);
uint8_t PCS_Encode2B2(uint8_t data[5], uint16_t powerWatts, bool chargeEnabled, bool shortFrame);

#endif /* PCS_PROTOCOL_H */
