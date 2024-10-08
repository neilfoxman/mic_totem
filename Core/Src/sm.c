/*
 * sm.c
 *
 *  Created on: Oct 6, 2024
 *      Author: NeilF
 */

#include "sm.h"

State current_state;

void transition(State state){
	current_state = state;
	current_state(ENTER);
}
