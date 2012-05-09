#if PROGMEM_SIZE > 0x010000
#define PGM_READ_BYTE pgm_read_byte_far
#else
#define PGM_READ_BYTE pgm_read_byte_near
#endif

#include "main.h"
#include "board_init.c"

#define prt_flt3(f)	(int16_t)f,(int16_t)((fabs(f)-(int16_t)(fabs(f)))*1000)			// use %i.%i inside print_f in place of %f

#define PI 3.14159
#define RADtoDEG 57.29578
#define NUM_NEIGHBORS 6

#define BOTTOM_LEFT 1	
#define BOTTOM_RIGHT 0  
#define LEFT_TOP 3      
#define LEFT_BOTTOM 2   
#define RIGHT_BOTTOM 4  
#define RIGHT_TOP 5    

// ============================================================================================
// prototype declaration
// ============================================================================================
void reset_myself(int);

void no_movement(void);

void swarm_initialization1(void);
void swarm_calculation1(void);
void swarm_interaction1(int);

void swarm_initialization2(void);
void swarm_calculation2(void);

void swarm_initialization3(void);
void swarm_calculation3(void);
void swarm_interaction3(int);

void communication(void);
void servo_motor_control(void);
void external_command(char);
// ============================================================================================

bool print_servo_info = false;
bool update_allowed = false;

bool sensor = false;

volatile unsigned long jiffies = 0;

uint8_t program_num = 1;

struct OBJ{
	float px, py, vx, vy, hd;
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
	bool open[NUM_NEIGHBORS];
} mchip;

struct point {
	float x, y;
	uint8_t pnum;	//for 8bit comm.
} mdata;

// ============================================================================================
// variables
// ============================================================================================
float dt = 0.05;		// time step for differential equation (no relation with real time)

float ld = 20.0;		// virtual distance between the nodes
float acc = 20.0;		// self-propelling force
float gmma =5.0;		// viscosity
float ka = 0.1;			// spring constant
float d = 0.5;			// strength of anisotropy
float cf = 100.0;		// strength of interaction with neighbors
float rc = 20.0;		// optimum distance between agents
float tau = 1.0;		// relaxation time of heading dynamics
float forcex, forcey;   // force

float ep = 0.2;
float dl = 0.5;
float alpha = 0.5;

// ============================================================================================
// Packet
// ============================================================================================
void rx_pkt(Xgrid::Packet *pkt)
{
		//fprintf_P(&usart_stream, PSTR("%d\r\n"), pkt->rx_node); //check received port
		mchip.open[pkt->rx_node] = true;

		point* pt_ptr = (point*) pkt->data;
		mchip.neix[pkt->rx_node] = pt_ptr->x;
		mchip.neiy[pkt->rx_node] = pt_ptr->y;

		program_num = pt_ptr->pnum;

		//over 50 = reset + program_num
		if(program_num >= 50){
			program_num -= 50;
			reset_myself(program_num);
		}
			
		LED_PORT.OUTTGL = LED_USR_2_PIN_bm;	//green LED
}

// ============================================================================================
// Timer tick ISR (1 kHz)
// ============================================================================================
ISR(TCC0_OVF_vect)
{
        jiffies++;	// Timers
	    xgrid.process();
}

// ============================================================================================
// MAIN FUNCTION
// ============================================================================================
int main(void)
{      
	char input_char;	
	int end_time;
	
	_delay_ms(50);
       
    init();

    xgrid.rx_pkt = &rx_pkt;

    LED_PORT.OUT = LED_USR_0_PIN_bm;
	fprintf_P(&usart_stream, PSTR("avr-xgrid build %ld\r\n"), (unsigned long) &__BUILD_NUMBER);

	// ##### Initialization of swarm dynamics #####
	switch(program_num){
		case 1: swarm_initialization1(); break;
		case 2: swarm_initialization2(); break;
		case 3: swarm_initialization3(); break;
	}
	// ############################################

	while (1){		

		end_time = jiffies + 100;

		if (usart.available()) input_char = usart.get();
							
		// main loop
		if (input_char == 0x1b) xboot_reset();

		else if(input_char != 0x00){
			fprintf_P(&usart_stream, PSTR("CPU: %c\r\n"), input_char);
			external_command(input_char);
			input_char = 0x00;	// clear the most recent computer input
		}

		communication();		// send position data to neighbors

		// ##### Processing for swarm dynamics #####
		switch(program_num){	// select program
			case 1: swarm_calculation1(); break;	// Ken's swarm dynamics
			case 2: swarm_calculation2(); break;	// two rythms
			case 3: swarm_calculation3(); break;	// Van der Pol Oscillator
			default: no_movement();
		}
		// #########################################

		servo_motor_control();	// servo control

		if(jiffies>end_time){
			LED_PORT.OUT = LED_USR_1_PIN_bm;		// turn on yellow LED
		}

		while(jiffies<end_time);

		if(jiffies>60000) jiffies = 0;	//reset jiffies in every 60 sec
	}
	return 0;
}

// ============================================================================================
// send messege
// ============================================================================================
void communication()
{
	mdata.x = mchip.px;
	mdata.y = mchip.py;
	mdata.pnum = program_num;

	Xgrid::Packet pkt;
	pkt.type = 0;
	pkt.flags = 0;	
	pkt.radius = 1;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);
									
	xgrid.send_packet(&pkt,0b00111111);		// send to all neighbors
}

// ============================================================================================
// reset messege to all
// ============================================================================================
void reset_myself(int num)
{
	switch(num){
		case 1: swarm_initialization1(); break;
		case 2: swarm_initialization2(); break;
		case 3: swarm_initialization3(); break;
	}
}

void reset(int num)
{
	reset_myself(num);
	init_servo();

	//send reset messege
	mdata.x = 0.0;	mdata.y = 0.0;	mdata.pnum = 50 + num;

	Xgrid::Packet pkt;
	pkt.type = 0;
	pkt.flags = 0;	
	pkt.radius = 14;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);
	xgrid.send_packet(&pkt,0b00111111);		// send to all neighbors
}

// ============================================================================================
// external command
// ============================================================================================
void external_command(char command)
{
	switch(command){	// change program
		case '0':	reset(0);	program_num=0;	break;
		case '1':	reset(1);	program_num=1;	break;
		case '2':	reset(2);	program_num=2;	break;
		case '3':	reset(3);	program_num=3;	break;

		case ' ':		// simulate sensor on/off 
			if(!sensor){
				sensor = true; 
				fprintf_P(&usart_stream, PSTR("Sensor: detect!\n\r"));
			}
			else{
				sensor = false;
				fprintf_P(&usart_stream, PSTR("Sensor: no detect\n\r"));
			}
			break;

		case 's':		// monitor servo angle
			if(print_servo_info)	print_servo_info = false;
			else					print_servo_info = true;
			break;

		case 'v':		// display program version
			fprintf_P(&usart_stream, PSTR("avr-xgrid build %ld\r\n"), (unsigned long) &__BUILD_NUMBER);
			break;
					
		default:
			fprintf_P(&usart_stream, PSTR("UNKNOWN command received: %c\n\r"), command);
	}
}

// ============================================================================================
// SERVO MOTOR
// ============================================================================================
void servo_motor_control()
{
	float servo_pos_flt, speed = 8.0;
	
	servo_pos_flt = 90*cos(mchip.hd * speed);
	
	if(print_servo_info)
		fprintf_P(&usart_stream, PSTR("%i.%i, deg: %i.%i\n\r"), prt_flt3(mchip.hd), prt_flt3(servo_pos_flt));

	set_servo_position(servo_pos_flt);
}

// ============================================================================================
// for initialization of swarm dynamics function
// ============================================================================================
void init_common()
{
	int i;
	init_servo();
	for(i=0;i<NUM_NEIGHBORS;i++){
		mchip.neix[i] = 0.0;
		mchip.neiy[i] = 0.0;
		mchip.open[i] = false;
	}
}

// ############################################################################################
// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 1 --- Ken's Swarm Dynamics ---
// ============================================================================================
// ============================================================================================
void swarm_initialization1()
{
	init_common();

	mchip.px = 0.0;
	mchip.py = 0.0;
	mchip.vx = 1.0;
	mchip.vy = 1.0;
	mchip.hd = PI/2.0;
}
// --------------------------------------------------------------------------------------------
void swarm_calculation1()
{
	int i;
	float dvx, dvy, lx, ly, vabs, ds, fx, fy;
	float dir = mchip.hd;
	float cvx = mchip.vx;
	float cvy = mchip.vy;

	// ===== calculation of forces =====
	//self-propel force and viscosity
	dvx = acc * cos(dir) - gmma * cvx;
	dvy = acc * sin(dir) - gmma * cvy;

	//interaction force with 6 neighbors
	forcex=0; forcey=0;
	for(i=0;i<6;i++){
		if(mchip.open[i]) swarm_interaction1(i);
	}

	dvx = dvx + forcex;
	dvy = dvy + forcey;

	//spring term (Zero-length spring). fixed point = (0,0)
	lx = - mchip.px;
	ly = - mchip.py;
	dvx = dvx + ka * lx;
	dvy = dvy + ka * ly;

	// ===== update =====
	//direction
	vabs = sqrt(cvx * cvx + cvy * cvy);
	fx = cvx / vabs;
	fy = cvy / vabs;
	ds = -1.0 / tau * (sin(dir)*fx-cos(dir)*fy); // get sin value by cross product
	mchip.hd += ds * dt;

	if(mchip.hd > 2.0 * PI)		mchip.hd -= 2.0 * PI;
	if(mchip.hd < 0 ) 			mchip.hd += 2.0 * PI;

	//velocity
	mchip.vx += dvx * dt;
	mchip.vy += dvy * dt;

	//position
	mchip.px += mchip.vx * dt;
	mchip.py += mchip.vy * dt;
}
// --------------------------------------------------------------------------------------------
void swarm_interaction1(int nei)
{
	float dirx, diry, disx, disy, dis1, dis2, alph, force;
	float di, dj;
	bool flag=true;

	dirx = cos(mchip.hd);
	diry = sin(mchip.hd);

	switch(nei){
		case BOTTOM_RIGHT: di= 0.866; dj= 0.500; break;
		case BOTTOM_LEFT : di=-0.866; dj= 0.500; break;
		case LEFT_BOTTOM : di=-1.000; dj= 0.000; break;
		case LEFT_TOP    : di=-0.866; dj=-0.500; break;
		case RIGHT_BOTTOM: di= 1.000; dj= 0.000; break;
		case RIGHT_TOP   : di= 0.866; dj=-0.500; break;
		default: flag=false;
	}

	if(flag){
		disx = mchip.neix[nei] + ld * di - mchip.px;
		disy = mchip.neiy[nei] + ld * dj - mchip.py;

		dis2 = disx * disx + disy * disy;
		dis1 = sqrt(dis2);

		alph = 1.0 + d * (disx * dirx + disy * diry) / dis1; //inner product
		force = -cf * (rc / dis1 - 1.0) * rc * rc / dis2;
		forcex += alph * force * disx / dis1;
		forcey += alph * force * disy / dis1;
	}
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 2 --- two rhythms ---
// ============================================================================================
// ============================================================================================
void swarm_initialization2()
{
	init_common();
	mchip.px = 1.0;
	mchip.py = 0.0; 
	mchip.hd = 0.0;
}

// --------------------------------------------------------------------------------------------
void swarm_calculation2()
{
	int i, cnt1 = 0, cnt2 = 0, cnt;
	float dp = 0;

	for(i=0;i<NUM_NEIGHBORS;i++){
		if(mchip.open[i]){
			if(mchip.neix[i]==1.0) cnt1++; else cnt2++;
		}
	}
	if(cnt1<cnt2){
		mchip.px = 1.0;
		dp = 0.01;
	}
	else{
		mchip.px = 2.0;
		dp = 0.05;
	}

	mchip.hd+=dp;
	if(mchip.hd>2.0*PI) mchip.hd-=2.0*PI;
	if(mchip.hd<0)		mchip.hd+=2.0*PI;
}

// ============================================================================================
// ============================================================================================
// SWARM DYNAMICS 3 --- Coupled Van Der Pol ---
// ============================================================================================
// ============================================================================================
void swarm_initialization3()
{
	init_common();
	mchip.px = 1.7;
	mchip.py = 0.0;
}

// --------------------------------------------------------------------------------------------
void swarm_calculation3()
{
	float dx, dy, sign, sum = mchip.px;
	int i, cnt = 1;

	for(i=0;i<NUM_NEIGHBORS;i++){
		if(mchip.open[i]){
			sum+=mchip.neix[i];
			cnt++;
		}
	}

	dx = mchip.py * dt;

	if(!sensor) sign = 1.0; else sign = -1.0;
	dy = (-alpha * mchip.px + ep*(1-mchip.px * mchip.px) * mchip.py + sign * dl * sum / cnt) * dt;
		
	mchip.px = mchip.px + dx * dt;
	mchip.py = mchip.py + dy * dt;
}

// ============================================================================================
// ============================================================================================
// NO MOVEMENT
// ============================================================================================
// ============================================================================================
void no_movement()
{
}
// ############################################################################################
