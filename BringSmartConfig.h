#ifndef BRING_SMART_CONFIG_H
#define BRING_SMART_CONFIG_H

	#define SPEED_RATIO				0.00816
	#define ENCODER_POLLING_RATE	50

	#define V_KP	6 // 10
	#define V_KI	60 // 111
	#define V_KD	0

	#define P_KP	0	//7		//7
	#define P_KI	0	//2     //15
	#define P_KD	0 //0     //0.2

	#define MIN_PWM	-254
	#define MAX_PWM	254
	#define UPPER_BAR_PWN	0 //25
	#define BOTTOM_BAR_PWM	0 //25

#define PWM_Freq 20000 //25

#define AFK_timer 5000

#endif
