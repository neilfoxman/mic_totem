/*
 * sm.h
 *
 *  Created on: Oct 4, 2024
 *      Author: NeilF
 */

#ifndef INC_SM_H_
#define INC_SM_H_

typedef enum{
	ENTER,
    TIM_UE,
    ADC_OVR,
	MIC_DMA_COMPLETE,
	LED_DMA_COMPLETE,
	NUM_EVENTS
} Event;

typedef void (*State)(Event evt);

extern State current_state;

void transition(State state);

#endif /* INC_SM_H_ */
