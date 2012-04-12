/************************************************************************/
/* honeycomb                                                            */
/*                                                                      */
/* user.cpp                                                             */
/*                                                                      */
/* Ken Sugawara                                                         */
/*                                                                      */
/* Copyright (c) 2012 Ken Sugawara                                      */
/*                                                                      */
/* Permission is hereby granted, free of charge, to any person          */
/* obtaining a copy of this software and associated documentation       */
/* files(the "Software"), to deal in the Software without restriction,  */
/* including without limitation the rights to use, copy, modify, merge, */
/* publish, distribute, sublicense, and/or sell copies of the Software, */
/* and to permit persons to whom the Software is furnished to do so,    */
/* subject to the following conditions:                                 */
/*                                                                      */
/* The above copyright notice and this permission notice shall be       */
/* included in all copies or substantial portions of the Software.      */
/*                                                                      */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,      */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF   */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND                */
/* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS  */
/* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   */
/* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN    */
/* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE     */
/* SOFTWARE.                                                            */
/*                                                                      */
/************************************************************************/

#include "honeycomb.h"

// prototypes
void swarm_communication();
void swarm_calculation();
void swarm_interaction(int i, int j, int nei);
void servo_motor_control();


float ld = 20.0;		// virtual distance between the nodes
float dt = 0.05;		// time step for difference equation
float acc = 10.0;		// self-propelling force
float gmma =5.0;		// viscosity
float ka = 0.1;			// spring constant
float d = 0.5;			// strength of anisotropy
float cf = 100.0;		// strength of interaction with neighbors
float rc = 20.0;		// optimum distance between agents
float tau = 1.0;		// relaxation time of heading dynamics
float forcex, forcey;   // force

int i, j, t;	// moved from swarm_initialization() to global

#define PI 3.14159
#define RADtoDEG 57.29578
#define delayX 10
#define SIZEX 10
#define SIZEY 10
#define NUM_NEIGHBORS 4

struct OBJ{
	// px, py point
	// vx, vy velocity of ?
	// hd heading (radians)
	// dt delta time
	// vpx, vpy velocity of point
	float px, py, vx, vy, hd, dt, vpx, vpy;
	// ?
	float mempx[delayX], mempy[delayX];
	// neighbors
	float neix[NUM_NEIGHBORS], neiy[NUM_NEIGHBORS];
	int mempt;
} mchip;


/**
 * Data transmitted to neighbors
 */
struct point {
	float x, y;
};

point mdata;

void loop() {
	unsigned long jiffies = getJiffies();

    if (jiffies % 50 == 0)
	{
		LED_PORT.OUTTGL = LED_USR_0_PIN_bm;		// toggle red light every 50 ms (20 Hz)
		servo_motor_control();
	}

	if (jiffies % 100 == 0)
	{
		swarm_communication();
		swarm_calculation();
	}
}

void rx_pkt(Xgrid::Packet *pkt)
{
		point* pt_ptr = (point*) pkt->data;
		mchip.neix[pkt->source_id] = pt_ptr->x;
		mchip.neiy[pkt->source_id] = pt_ptr->y;
        LED_PORT.OUTTGL = LED_USR_2_PIN_bm;		// toggle green light when receive packet?
}

void keyPressed(char key) {
	usart_stream_fprintf_P(PSTR("CPU: %c\r\n"), key);

	if (key == 'a')
	{
		char str[] = "A";
		Xgrid::Packet pkt;		// Packet is defined on line 72 of xgrid.h
		pkt.type = 0;
		pkt.flags = 0;
		pkt.radius = 1;
		pkt.data = (uint8_t *)str;
		pkt.data_len = 4;

		xgrid_send_packet(&pkt);
	}

	else if(key == 'y')
	{
		LED_PORT.OUTTGL = LED_USR_1_PIN_bm;
	}
}

void setup()
{

	mchip.px=0;
	mchip.py=0;
	mchip.vpx=0.0;
	mchip.vpy=0.0;

	for(t=0;t<delayX;t++){
		mchip.mempx[t]=0;
		mchip.mempy[t]=0;
	}
	mchip.mempt=0;

	mchip.vx = 1.0;
	mchip.vy = 1.0;
	mchip.hd = PI/4.0;

	for(t=0;t<NUM_NEIGHBORS;t++){
		mchip.neix[t]=0;
		mchip.neiy[t]=0;
	}
}

/*
 * Just sending
 */
void swarm_communication()
{
	/*
	Here I assume:
		Send: the value of 'mchip.vpx' and 'mchip.vpy' to the neighbors
		Receive and store:
			(mchip.neix[0], mchip.neiy[0]) <- bottom Chip's (.vpx, .vpy)
			(mchip.neix[1], mchip.neiy[1]) <- left Chip's (.vpx, .vpy)
			(mchip.neix[2], mchip.neiy[2]) <- right Chip's (.vpx, .vpy)
			(mchip.neix[3], mchip.neiy[3]) <- top Chip's (.vpx, .vpy)
	*/
	mdata.x = mchip.vpx;
	mdata.y = mchip.vpy;
	Xgrid::Packet pkt;
	pkt.type = 0;
	pkt.flags = 0;
	pkt.radius = 1;
	pkt.data = (uint8_t *)&mdata;
	pkt.data_len = sizeof(point);

	xgrid_send_packet(&pkt);
}

void swarm_calculation()
{
	//int check;	// unused
	// dvx, dvy delta velocity
	// lx, ly current position
	// ds ?
	// fx, fy ?
	float dvx, dvy, lx, ly, vabs, ds, fx, fy;
	// dir new direction
	float dir = mchip.hd;
	// cvx, cvy new velocity (?)
	float cvx = mchip.vx;
	float cvy = mchip.vy;

	// ===== calculation of forces =====
	//self-propel force and viscosity
	dvx = acc * cos(dir) - gmma * cvx;
	dvy = acc * sin(dir) - gmma * cvy;

	//interaction force with 4 neighbors
	//It contains the process for boundaries.
	forcex=0; forcey=0;
	if(i!=0)  			swarm_interaction(i,j,1);
	if(i!=SIZEX-1) 		swarm_interaction(i,j,2);
	if(j!=0)  			swarm_interaction(i,j,3);
	if(j!=SIZEY-1)		swarm_interaction(i,j,0);
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

	//This memory is for intentional delay effect
	mchip.mempx[mchip.mempt] = mchip.px;
	mchip.mempy[mchip.mempt] = mchip.py;

	mchip.vpx = mchip.mempx[mchip.mempt];
	mchip.vpy = mchip.mempy[mchip.mempt];

	mchip.mempt++;
	if(mchip.mempt == delayX)	mchip.mempt = 0;

}

//==============================================
/**
 * i, j current position
 * nei neighbor index
 */
void swarm_interaction(int i, int j, int nei)
{
	// dirx, diry direction (?)
	// disx, disy ?
	// dis1
	// dis2
	// alph
	// force
	float dirx, diry, disx, disy, dis1, dis2, alph, force;
	int di,dj;

	dirx = cos(mchip.hd);
	diry = sin(mchip.hd);

	switch(nei){
		case 0: di=0; dj=+1; break;
		case 1: di=-1; dj=0; break;
		case 2: di=+1; dj=0; break;
		case 3: di=0; dj=-1; break;
	}
	
	disx = mchip.neix[nei] + ld * di - mchip.px;
	disy = mchip.neiy[nei] + ld * dj - mchip.py;

	dis2 = disx * disx + disy * disy;
	dis1 = sqrt(dis2);

	alph = 1.0 + d * (disx * dirx + disy * diry) / dis1; //inner product
	force = -cf * (rc / dis1 - 1.0) * rc * rc / dis2;
	forcex = forcex + alph * force * disx / dis1;
	forcey = forcey + alph * force * disy / dis1;
}

void servo_motor_control()
{
	float servo_pos_flt;
	
	// Angle of axis Aa is calculated by
	// Aa = 2.0 * cos(mchip.hd)

	// This is just an example. There are many ways to translate
	// virtual agent's movement to the servo's angle.

	// mchip.hd (range: 0 - 2*PI)

	//servo_pos_flt = (mchip.hd-PI)*RADtoDEG;
	
	servo_pos_flt = 90*cos(mchip.hd);		// x = r*cos(theta)
	
	usart_stream_fprintf_P(PSTR("hd: %i.%i, deg: %i.%i\r\n"), prt_flt3(mchip.hd), prt_flt3(servo_pos_flt));

	set_servo_position(servo_pos_flt);
}


