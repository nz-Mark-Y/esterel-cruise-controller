#include <stdbool.h>
#include "cruiseController.h"

/*
DESCRIPTION: Saturate the throttle command to limit the acceleration.
PARAMETERS: throttleIn - throttle input
            saturate - true if saturated, false otherwise
RETURNS: throttle output (ThrottleCmd)
*/
float saturateThrottle(float throttleIn, bool* saturate) {
	if (throttleIn > THROTTLESATMAX) {
		*saturate = true;
		return THROTTLESATMAX;
	} else if (throttleIn < 0) {
		*saturate = true;
		return 0;
	} else {
		*saturate = false;
		return throttleIn;
	}
}

/*
DESCRIPTION: Regulate the throttle using the KP and KI terms
PARAMETERS: isGoingOn - true if the cruise control has just gone into the ON state 
                        from another state; false otherwise
            cruiseSpeed - target speed
			vehicleSpeed - current speed of vehicle
RETURNS: throttle output (ThrottleCmd)
*/
float regulateThrottle(int isGoingOn, float cruiseSpeed, float vehicleSpeed) {
	static bool saturate = true;
	static float iterm = 0;
	
	if (isGoingOn == 1) {
		iterm = 0;	// reset the integral action
		saturate = true;	
	}
	float error = cruiseSpeed - vehicleSpeed;
	float proportionalAction = error * KP;
	if (saturate) {
		error = 0;	// integral action is hold when command is saturated
	}
	iterm = iterm + error;
	float integralAction = KI * iterm;
	return saturateThrottle(proportionalAction + integralAction, &saturate);
}
