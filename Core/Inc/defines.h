#define led_level TIM2->CCR3
#define buzzer_on HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15, GPIO_PIN_SET)
#define buzzer_off HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15, GPIO_PIN_RESET)

#define sb_up HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)
#define sb_down HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)
#define sb_left HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)
#define sb_right HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)
#define sb_ok HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)

/*
 * Hardware output polarity.
 * Keep GPIO_PIN_SET for a direct, non-inverting 3.3V-to-5V logic buffer.
 * Change the corresponding ACTIVE_LEVEL to GPIO_PIN_RESET when the controller
 * board uses an inverting transistor stage (as on the reference V1 board).
 * Verify 5V at the PCS connector before applying HV/AC power.
 */
#define PCS_ENABLE_ACTIVE_LEVEL GPIO_PIN_SET
#define DCDC_ENABLE_ACTIVE_LEVEL GPIO_PIN_SET
#define CHG_ENABLE_ACTIVE_LEVEL GPIO_PIN_SET

#define GPIO_INACTIVE_LEVEL(active) ((active) == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET)

#define pcs_on HAL_GPIO_WritePin(PIN_PCS_GPIO_Port, PIN_PCS_Pin, PCS_ENABLE_ACTIVE_LEVEL)
#define pcs_off HAL_GPIO_WritePin(PIN_PCS_GPIO_Port, PIN_PCS_Pin, GPIO_INACTIVE_LEVEL(PCS_ENABLE_ACTIVE_LEVEL))

#define dcdc_on HAL_GPIO_WritePin(PIN_DCDC_GPIO_Port, PIN_DCDC_Pin, DCDC_ENABLE_ACTIVE_LEVEL)
#define dcdc_off HAL_GPIO_WritePin(PIN_DCDC_GPIO_Port, PIN_DCDC_Pin, GPIO_INACTIVE_LEVEL(DCDC_ENABLE_ACTIVE_LEVEL))

#define chg_on HAL_GPIO_WritePin(PIN_CHG_GPIO_Port, PIN_CHG_Pin, CHG_ENABLE_ACTIVE_LEVEL)
#define chg_off HAL_GPIO_WritePin(PIN_CHG_GPIO_Port, PIN_CHG_Pin, GPIO_INACTIVE_LEVEL(CHG_ENABLE_ACTIVE_LEVEL))

