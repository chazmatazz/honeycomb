// servo.h

#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define QUICK_SERVO 0
#define SMART_SERVO 1

#define SERVO_MAX_POS_DEG 90		// +/- degrees
#define SERVO_MIN_POS_DEG -90		// +/- degrees			

extern uint8_t SERVO_MODE;
extern int16_t smart_servo_pos_deg;

void init_servo(void);

//uint16_t servo_CNT_compare_from_postion(int16_t degrees);
uint16_t servo_CNT_compare_from_postion(float degrees);
//void set_servo_position(int16_t degrees);
void set_servo_position(float degrees);

#ifdef __cplusplus
}
#endif

#endif
