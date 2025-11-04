
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include <stdbool.h>
#include <string.h>

TIM_HandleTypeDef TimHandle;
uint8_t ppm_count = 0;
uint32_t timeout = 100;
uint8_t nunchuck_data[6] = {0};

uint8_t i2cBuffer[2];

extern I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

#ifdef CONTROL_PPM
uint16_t ppm_captured_value[PPM_NUM_CHANNELS + 1] = {500, 500};
uint16_t ppm_captured_value_buffer[PPM_NUM_CHANNELS+1] = {500, 500};
uint32_t ppm_timeout = 0;

bool ppm_valid = true;

#define IN_RANGE(x, low, up) (((x) >= (low)) && ((x) <= (up)))

void PPM_ISR_Callback() {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_delay = TIM2->CNT;
  TIM2->CNT = 0;

  if (rc_delay > 3000) {
    if (ppm_valid && ppm_count == PPM_NUM_CHANNELS) {
      ppm_timeout = 0;
      memcpy(ppm_captured_value, ppm_captured_value_buffer, sizeof(ppm_captured_value));
    }
    ppm_valid = true;
    ppm_count = 0;
  }
  else if (ppm_count < PPM_NUM_CHANNELS && IN_RANGE(rc_delay, 900, 2100)){
    timeout = 0;
    ppm_captured_value_buffer[ppm_count++] = CLAMP(rc_delay, 1000, 2000) - 1000;
  } else {
    ppm_valid = false;
  }
}

// SysTick executes once each ms
void PPM_SysTick_Callback() {
  ppm_timeout++;
  // Stop after 500 ms without PPM signal
  if(ppm_timeout > 500) {
    int i;
    for(i = 0; i < PPM_NUM_CHANNELS; i++) {
      ppm_captured_value[i] = 500;
    }
    ppm_timeout = 0;
  }
}

void PPM_Init() {
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_TIM2_CLK_ENABLE();
  TimHandle.Instance = TIM2;
  TimHandle.Init.Period = UINT16_MAX;
  TimHandle.Init.Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0); // TODO
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_TIM_Base_Start(&TimHandle);
}
#endif

#ifdef CONTROL_PWM
extern TIM_HandleTypeDef htim2;

volatile uint16_t pwm_captured_value[PWM_NUM_CHANNELS] = {1500, 1500}; // Default to center (1500us)
volatile uint32_t pwm_timeout[PWM_NUM_CHANNELS] = {0, 0}; // Timeout counter - starts at 0
volatile uint16_t pwm_rising_edge[PWM_NUM_CHANNELS] = {0, 0};
volatile uint8_t pwm_edge_state[PWM_NUM_CHANNELS] = {0, 0}; // 0 = waiting for rising, 1 = waiting for falling
volatile uint8_t pwm_valid[PWM_NUM_CHANNELS] = {0, 0}; // Flag to indicate if valid PWM signal was received (starts invalid)

// PWM Input Capture Callbacks
void PWM_Channel1_ISR_Callback() {
  // TIM2 Channel 3 (PA2) - Channel 1 for steering
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3)) {
    if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC3)) {
      __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC3);
      
      uint16_t capture = htim2.Instance->CCR3;
      
      if (pwm_edge_state[0] == 0) {
        // Rising edge detected
        pwm_rising_edge[0] = capture;
        pwm_edge_state[0] = 1;
        // Switch to falling edge capture
        TIM_IC_InitTypeDef sConfigIC;
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
        sConfigIC.ICFilter = 0;
        HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);
      } else {
        // Falling edge detected - calculate pulse width
        uint16_t pulse_width;
        if (capture >= pwm_rising_edge[0]) {
          pulse_width = capture - pwm_rising_edge[0];
        } else {
          // Handle overflow
          pulse_width = (UINT16_MAX - pwm_rising_edge[0]) + capture;
        }
        
        // Clamp to valid servo range (1000-2000 microseconds)
        pulse_width = CLAMP(pulse_width, 1000, 2000);
        // Only update if pulse width is in valid range
        if (pulse_width >= 1000 && pulse_width <= 2000) {
          pwm_captured_value[0] = pulse_width;
          pwm_valid[0] = 1; // Mark as valid
          pwm_timeout[0] = 0;
        }
        pwm_edge_state[0] = 0;
        
        // Switch back to rising edge capture
        TIM_IC_InitTypeDef sConfigIC;
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
        sConfigIC.ICFilter = 0;
        HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);
      }
    }
  }
}

void PWM_Channel2_ISR_Callback() {
  // TIM2 Channel 4 (PA3) - Channel 2 for speed
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC4)) {
    if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC4)) {
      __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
      
      uint16_t capture = htim2.Instance->CCR4;
      
      if (pwm_edge_state[1] == 0) {
        // Rising edge detected
        pwm_rising_edge[1] = capture;
        pwm_edge_state[1] = 1;
        // Switch to falling edge capture
        TIM_IC_InitTypeDef sConfigIC;
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
        sConfigIC.ICFilter = 0;
        HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
      } else {
        // Falling edge detected - calculate pulse width
        uint16_t pulse_width;
        if (capture >= pwm_rising_edge[1]) {
          pulse_width = capture - pwm_rising_edge[1];
        } else {
          // Handle overflow
          pulse_width = (UINT16_MAX - pwm_rising_edge[1]) + capture;
        }
        
        // Clamp to valid servo range (1000-2000 microseconds)
        pulse_width = CLAMP(pulse_width, 1000, 2000);
        // Only update if pulse width is in valid range
        if (pulse_width >= 1000 && pulse_width <= 2000) {
          pwm_captured_value[1] = pulse_width;
          pwm_valid[1] = 1; // Mark as valid
          pwm_timeout[1] = 0;
        }
        pwm_edge_state[1] = 0;
        
        // Switch back to rising edge capture
        TIM_IC_InitTypeDef sConfigIC;
        sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
        sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
        sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
        sConfigIC.ICFilter = 0;
        HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
      }
    }
  }
}

// SysTick executes once each ms
void PWM_SysTick_Callback() {
  for (int i = 0; i < PWM_NUM_CHANNELS; i++) {
    // Increment timeout counter
    pwm_timeout[i]++;
    // Stop after 500 ms without valid PWM signal
    if (pwm_timeout[i] > 500) {
      pwm_captured_value[i] = 1500; // Center position (safe stop)
      pwm_valid[i] = 0; // Mark as invalid - no signal received
      // Keep timeout at 500 to prevent overflow
      if (pwm_timeout[i] > 1000) {
        pwm_timeout[i] = 500;
      }
    }
  }
}

void PWM_Init() {
  GPIO_InitTypeDef GPIO_InitStruct;
  TIM_IC_InitTypeDef sConfigIC;
  
  // Enable clocks
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();
  
  // Configure TIM2 for PWM input capture on both channels
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (SystemCoreClock / 1000000) - 1; // 1MHz = 1us resolution
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = UINT16_MAX;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_IC_Init(&htim2);
  
  // Configure PA2 for TIM2 Channel 3 (Input Capture) - Channel 1 for steering
  // PA2 is TIM2_CH3 alternate function
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // Pull-up to ensure clean signal
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // Configure PA3 for TIM2 Channel 4 (Input Capture) - Channel 2 for speed
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // Pull-up to ensure clean signal
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  // Start the timer base (required for input capture)
  HAL_TIM_Base_Start(&htim2);
  
  // Configure TIM2 Channel 3 (PA2) for input capture (start with rising edge)
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3);
  
  // Configure TIM2 Channel 4 (PA3) for input capture (start with rising edge)
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
  
  // Start input capture
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
  
  // Enable interrupts
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
#endif

void Nunchuck_Init() {
    //-- START -- init WiiNunchuck
  i2cBuffer[0] = 0xF0;
  i2cBuffer[1] = 0x55;

  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 2, 100);
  HAL_Delay(10);

  i2cBuffer[0] = 0xFB;
  i2cBuffer[1] = 0x00;

  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 2, 100);
  HAL_Delay(10);
}

void Nunchuck_Read() {
  i2cBuffer[0] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 1, 100);
  HAL_Delay(5);
  if (HAL_I2C_Master_Receive(&hi2c2,0xA4,(uint8_t*)nunchuck_data, 6, 100) == HAL_OK) {
    timeout = 0;
  } else {
    timeout++;
  }

  if (timeout > 3) {
    HAL_Delay(50);
    Nunchuck_Init();
  }

  //setScopeChannel(0, (int)nunchuck_data[0]);
  //setScopeChannel(1, (int)nunchuck_data[1]);
  //setScopeChannel(2, (int)nunchuck_data[5] & 1);
  //setScopeChannel(3, ((int)nunchuck_data[5] >> 1) & 1);
}
