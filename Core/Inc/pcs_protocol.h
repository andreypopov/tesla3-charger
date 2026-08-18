#ifndef PCS_PROTOCOL_H
#define PCS_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint8_t mainState;
  uint8_t hvChargeStatus;
  uint8_t gridConfig;
  uint8_t instantAcPowerDeciKw;
  uint8_t maximumAcPowerDeciKw;
  uint8_t phaseAEnabled;
  uint8_t phaseBEnabled;
  uint8_t phaseCEnabled;
  uint8_t phaseACurrentRequestDeciAmps;
  uint8_t phaseBCurrentRequestDeciAmps;
  uint8_t phaseCCurrentRequestDeciAmps;
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

typedef struct
{
  uint16_t lvVoltageRaw;
  uint16_t hvVoltageRaw;
  uint16_t outputCurrentRaw;
  uint16_t lvVoltageVolts;
  uint16_t hvVoltageVolts;
  uint16_t outputCurrentAmps;
} PCS_DcdcRailStatus;

typedef enum
{
  PCS_CHARGE_PORT_US = 0,
  PCS_CHARGE_PORT_EU = 1
} PCS_ChargePortProfile;

typedef struct
{
  uint8_t muxId;
  uint8_t phaseIndex;
  uint16_t currentRaw;
  uint16_t currentMilliAmps;
} PCS_ChargePhaseDebug;

void PCS_Decode204(const uint8_t data[8], PCS_ChargerStatus *status);
void PCS_Decode264(const uint8_t data[6], PCS_ChargeLineStatus *status);
void PCS_Decode2B4(const uint8_t data[5], PCS_DcdcRailStatus *status);
bool PCS_Decode76CChargePhase(const uint8_t data[8], PCS_ChargePhaseDebug *status);

void PCS_Encode13D(uint8_t data[6], uint8_t currentLimitAmps, bool chargeEnabled);
void PCS_Encode21D(uint8_t data[8], uint8_t pilotCurrentAmps,
                   uint8_t cableCurrentLimitAmps, PCS_ChargePortProfile profile);
void PCS_Encode22A(uint8_t data[4], uint16_t hvVolts, bool dcdcEnabled, bool chargeEnabled);
void PCS_Encode23D(uint8_t data[4], uint8_t currentLimitAmps, bool chargeEnabled);
void PCS_Encode25D(uint8_t data[8], PCS_ChargePortProfile profile);
void PCS_Encode333(uint8_t data[4], uint8_t currentLimitAmps);
uint8_t PCS_Encode2B2(uint8_t data[5], uint16_t powerWatts, bool chargeEnabled, bool shortFrame);

uint8_t PCS_ClampChargeCurrent(uint8_t requestedAmps, uint8_t maximumAmps);
uint16_t PCS_CalculateChargePowerTarget(uint8_t currentAmps, uint16_t acVolts,
                                        uint16_t maximumWatts);
bool PCS_IsChargeOverCurrent(uint16_t measuredAmps, uint8_t setpointAmps,
                             uint8_t marginAmps);

#endif /* PCS_PROTOCOL_H */
