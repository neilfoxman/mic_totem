# File Structure Tour

TBD

# Firmware

## State Machine

The program is responsible for both filtering microphone samples and updating LEDs based on the filtered signals. This behavior is achieved by cycling through a loop in a state machine shown below.

```mermaid
---
title: State Machine in main.c
---
stateDiagram-v2

classDef state_style text-align:left

config_mic_s : config_mic_s
config_mic_s:::state_style
[*] --> config_mic_s
config_mic_s : <b>ENTER/</b>
config_mic_s : - Disable timer, DMA, and LED Output
config_mic_s : - Update timer period
config_mic_s : - Calculate filter params
config_mic_s : - Enable DMA, ADC, and Timer

config_mic_s --> mic_s : TIM_UE
mic_s:::state_style
mic_s : mic_s
mic_s : <b>ENTER/</b>
mic_s : <b>MIC_DMA_COMPLETE/</b>
mic_s : - Called after all ADC channels sampled once.
mic_s : - Apply filters to mic signal.

mic_s --> config_led_s : num_samples_taken >= num_samples_before_LED
config_led_s : config_led_s
config_led_s:::state_style
config_led_s : <b>ENTER/</b>
config_led_s : - Disable timer, ADC, and DMA
config_led_s : - Calculate new LED Values and generate CCR values in array
config_led_s : - Update timer period
config_led_s : - Enable DMA and timer.

config_led_s --> led_s : TIM_UE
led_s : led_s
led_s:::state_style
led_s : <b>ENTER/</b>

led_s --> config_mic_s : LED_DMA_COMPLETE



```

The loops takes advantage of persistence of vision i.e. we can take a number of microphone samples then pause periodically to update LED values.  Note that the timer is used both for triggering samples of the microphone, then repurposed in output compare mode to communicate with the LEDs.

## Microphone Filtering

