/*
 * Acrobat_V1.c
 *
 * Created: 2012/10/27 11:06:36
 *  Author: Edward
 */ 
//##########################################################################
// this is the fourth version of our code, with comments
// this code is modified last by Edward on 11/5/2012 Mon.
// this code is compiled last on 11/7/2012 wed. and succeed 
// Changes from the last version:
// 1) add all the part for encoder, but the parameters are not perfect
// 2) actually some behavior of the robot prove that I maybe write something wrong in angle calibration
// Please describe all the function you added 
// The functions that already exist are:
// void TimerSetup(void); initialize the timer
// void systemInitial(void); initialize some function of the system
// int main(void); the main function 
// ISR(PCINT0_vect ) input change interrupt
// ISR(TIMER3_COMPA_vect) periodical interrupt
//##########################################################################


#include "m_general.h"
#include "m_usb.h"
#include "wireless.h"
#include "Balance_Kalman.h"

//******************** system configuration ********************************
//constant about period interrupt
#define PERIOD 5	//ms period1 which 
#define PERIOD2 100 //ms
#define PERIOD3 10	//ms
#define INTERRUPT10MS PERIOD3/PERIOD-1
#define INTERRUPT1S PERIOD2/PERIOD-1
//constant about dealing output data of the sensor
#define ACOFFSET 42
#define GYOFFSET -30
//parameter about kalman complementary
#define ACCPART 1
#define GYRPART 2-ACCPART
//Initial value of PID
#define KPINI 1000
#define KIINI 0
#define KDINI 5000
//************************************************************************
//****************** global variables ************************************
volatile int Timer3Flag=0;
volatile int Timer3Flag2=0;
volatile int Timer3Flag3=0;
volatile int ConvertFinishFlag=0;
volatile char buffer[packLength]={0};
// variables for encoder
volatile int numPulse=0;
volatile int LeftIn = 0;
volatile int RightIn = 0;
volatile int direction=1;
//************************************************************************

//================= system initialize ===================================
void TimerSetup(void){
	//Timer1 is used to generate the PWM
	OCR1A = 4000;
	OCR1B = 2000;
	OCR1C = 2000;
	set(DDRB,6);
	set(DDRB,7);
	TCCR1A = 0b00101011;
	TCCR1B = 0b00011001;
	//Timer3 is used to generate periodical interrupt
	OCR3A = (int)(15625*((float)PERIOD/1000));		//The equation to calculate the period of interrupt wich determined by PERIOD
	TCCR3A = 0b00000000;
	TCCR3B = 0b00001101;// /1024
	clear(TIMSK3 , OCIE3A);
	// The setup for the input capture should be here
}
void systemInitial(void){
	m_clockdivide(0);
	wirelessinitial();
	TimerSetup();
	//m_usb_init();
	//while(!m_imu_init(1,0));
	//sei();
	//while(!m_usb_isconnected());
}
//==========================================================================

int main(void)
{
	//original data
	int Data[9]={0};
	int RealinputACC = 0;
	int RealinputGyr = 0;
	//variables of flags
	int switchNum = 0;
	int paraDeter = 0;
	//temporary variables for test
	int tmp=0;
	int getValue=0;
	//variables for PID control
	int output;
	int angleHis1=0,angleHis2=0;
	int inputK=0,inputI=0,inputD=0;
	int Output=0;
	int OutputHis1=2000;
	int OutputHis2=2000;
	int KpIni=KPINI,KiIni=KIINI,KdIni=KDINI;
	float Kp = 1;
	float Ki = 0;
	float Kd = 0.5;
	int AngleCali = 0;
	int AngleActual=0;
	// Parameters for speed control
	int speed = 0;
	// PID variables for speed controller
	int speedHis1 = 0, speedHis2 = 0;
	int speedI = 0, speedD = 0;
	int outputSpeed =0;
	
	systemInitial();
	// send back comfirm information to prove that wireless has been connected
	char haha[5]="Hello";
	wireless_string(haha,5);
	wireless_char('\n');
	clear(DDRB,0);
	clear(PORTB,0);
	// using wireless to change parameters of PID controller, and if DIP switch 1 is ON, this can be skipped
	if (!check(PINB,0))
	{
		while(1)
		{
			//m_green(ON);
			int success;
			success = m_rf_read(buffer, packLength);
			if ((success==1)&&(buffer[0]>=45))
			{
				m_green(ON);
				ConvertFinishFlag=1;
			}
			else{
				m_green(OFF);
			}
			if (ConvertFinishFlag==1)
			{
				if (switchNum==0)
				{
					// send back current value of parameter and determine which one should be changed
					wireless_string(buffer,packLength);
					wireless_char(':');
					if ((buffer[0]=='k')&&(buffer[1]=='p'))
					{
						paraDeter = 1;
						wireless_int(KpIni);
					}
					if ((buffer[0]=='k')&&(buffer[1]=='i'))
					{
						paraDeter = 2;
						wireless_int(KiIni);
					}
					if ((buffer[0]=='k')&&(buffer[1]=='d'))
					{
						paraDeter = 3;
						wireless_int(KdIni);
					}
					m_wait(50);
					wireless_char('\n');
					switchNum++;
				}else{
					// change the parameter, and if the input is not a value, it will send back "SayAgain" and wait untill the input is a value
					getValue = atoi(buffer);
					if ((getValue==0)&&(buffer[0]!='0'))
					{
						char Sorry1[8] = "SayAgain";
						wireless_string(Sorry1,8);
						m_wait(50);
						wireless_char('\n');
						m_wait(50);
						switchNum=1;
					}else{
						wireless_string(buffer,packLength);
						m_wait(50);
						wireless_char('\n');
						m_wait(50);
						switchNum=0;
						switch (paraDeter)
						{
							case 1:KpIni=getValue;break;
							case 2:KiIni=getValue;break;
							case 3:KdIni=getValue;break;
						}
					}
				}
				// when the input is "start", the robot will start
				if ((buffer[0]=='s')&&(buffer[1]=='t')&&(buffer[2]=='a')&&(buffer[3]=='r')&&(buffer[4]=='t'))
				{
					break;
				}
				ConvertFinishFlag=0;
				memset(buffer,0,packLength);
				m_wait(100);
			}
		}
	}
	m_green(OFF);
	// Initializing IMU
	while(!m_imu_init(1,0));
	int AngleHis=0;
	// calibrate the balance angle value, the code here is trying to find out offset of the angle
	for (int numpoint=0;numpoint<499;numpoint++)
	{
		if(m_imu_raw(Data))
		{
			RealinputACC = ACCPART*(Data[1]-ACOFFSET);
			RealinputGyr = GYRPART*(GYOFFSET-Data[3]);
			Kalman_Filter(RealinputACC,RealinputGyr);
		}
		AngleHis+=angle;
	}
	AngleCali = AngleHis/500;
	// Enable timer interrupt and global interrupt
	set(TIMSK3 , OCIE3A);
	// Initializing the pin change interrupt which will capture the phase of the signal input of the encoder
	set(PCMSK0 , PCINT4);
	set(PCICR , PCIE0);
	clear(DDRB,4);
	clear(DDRB,5);
	sei();
	// Initializing all the parameters will be used in the PID control of the speed ( or we could say 'displacement')
	Kp = (float)KpIni/1000;
	Ki = (float)KiIni/1000;
	Kd = (float)KdIni/1000;
	float Kps, Kds, Kis;
	int OutputSpeedActual = 0;
	Kps = 4;
	Kds = 5;
	Kis = 0.5;
    while(1)
    {
		if (Timer3Flag==1)		//Here is the control loop, all the data sample process and output should be here
		{
			cli();	//Try to reduce the possibility of changing the data read from IIC, cause multiple device will use the same line and there also different interrupt in the program
			// read data from IMU
			if(m_imu_raw(Data))
			{
				m_red(TOGGLE);
			}
			// make the input value get rid of the offset
			RealinputACC = ACCPART*(Data[1]-ACOFFSET);	//The acceleration input without offset
			RealinputGyr = GYRPART*(Data[3]-GYOFFSET);	//The anglar velocity input without offset
			Kalman_Filter(RealinputACC,RealinputGyr);	//Using the Kalman Filter to get the reliable output of the angle

			// read the changed parameter, because the wireless cannot send float, so change it to float here
			Kp = (float)KpIni/1000;
			Ki = (float)KiIni/1000;
			Kd = (float)KdIni/1000;
			// calibrate the angle value by using the balance offset of the angle
			AngleActual = angle-AngleCali;
			inputK = AngleActual-angleHis1;
			inputI +=AngleActual;

			// PID controller, which you will find that Ki always be zero
			Output = Kp*AngleActual + Ki*inputI + Kd*inputK;
			
			// dead region, which means the value in this region won't give the wheels a speed, so remove it from output to make the output speed linear with the input angle
			if (AngleActual>0)
			{
				OutputHis1 =1823-Output;
			}
			if (AngleActual<0)
			{
				OutputHis1 =2193-Output;
			}
			if (AngleActual==0)
			{
				if (angleHis1>0)
				{
					OutputHis1 =1823;
				}
				if (angleHis1<0)
				{
					OutputHis1 =2193;
				}
				if (angleHis1=0)
				{
					OutputHis1 =OutputHis1;
				}
			}
			if (AngleActual>0)
			{
				OutputHis2 =1857-Output;
			}
			if (AngleActual<0)
			{
				OutputHis2 =2148-Output;
			}
			if (AngleActual==0)
			{
				if (angleHis1>0)
				{
					OutputHis2 =1857;
				}
				if (angleHis1<0)
				{
					OutputHis2 =2148;
				}
				if (angleHis1=0)
				{
					OutputHis2 =OutputHis2;
				}
			}
			// Try to check whether the output value in the limitation
			if (OutputHis1>4000)		//Detect the limitation of the output value
			{
				OutputHis1=4000;
			}else{
				if (OutputHis1<3)
				{
					OutputHis1=3;
				}
			}
			if (OutputHis2>4000)		//Detect the limitation of the output value
			{
				OutputHis2=4000;
			}else{
				if (OutputHis2<3)
				{
					OutputHis2=3;
				}
			}
			// set the output duty cycle
			OCR1B = OutputHis1;	// for B6
			OCR1C = OutputHis2;	// for B7
			
			angleHis2 = angleHis1;	//record value of the angle which will be used in PID controller(differential)
			angleHis1 = AngleActual;//record value of the angle which will be used in PID controller(differential)
			
			Timer3Flag=0;
		}

		if (Timer3Flag2==INTERRUPT1S)	//this condition used to send the wireless data or usb data, cause the frequency here is below 10Hz
		{
			cli();	// same concern with above
			
			speed += numPulse;	//red the the value of holes (net value, which means one direction is positive and the other is negative, here is just a summation)
			
			speedI += speed;
			speedD = speed - speedHis1;
			// PID controller of the speed(displacement actually) 
			outputSpeed = Kps*speed + Kis*speedI + Kds*speedD;
			
			//if (speed>0)
			//{
				//OutputSpeedActual =1800-outputSpeed;
			//}
			//if (speed<0)
			//{
				//OutputSpeedActual =2200-outputSpeed;
			//}
			//if (speed==0)
			//{
				//if (speed>0)
				//{
					//OutputSpeedActual =1800;
				//}
				//if (speed<0)
				//{
					//OutputSpeedActual =2200;
				//}
				//if (speed=0)
				//{
					//OutputSpeedActual =OutputSpeedActual;
				//}
			//}
			// For stability concern, I decide to limit this value, which means the maximum change of duty cycle is 500/4000 = 12.5%
			if (outputSpeed>500)
			{
				outputSpeed=500;
			}
			if (outputSpeed<-500)
			{
				outputSpeed=-500;
			}
			OCR1B = OutputHis1+outputSpeed;
			OCR1C = OutputHis2+outputSpeed;
			
			speedHis2 = speedHis1;
			speedHis1 = speed;
			// all the wireless receiving and sending codes should be wrote here 
			wireless_int(RealinputACC);	// send back value from IMU of acceleration of Y
			wireless_char('\t');
			wireless_int(RealinputGyr);	// send back value from IMU of angular speed around X
			wireless_char('\t');
			wireless_int(AngleActual);	// send back the current angle value
			wireless_char('\n');
			m_green(TOGGLE);
			Timer3Flag2=0;
			// clear the counter
			numPulse = 0;
			sei();
		}
		//if (Timer3Flag3==INTERRUPT10MS)
		//{
		//}				
    }
}
// The interrupt for the encoder, when the PORTB4 call the interrupt, if the PORTB5 is high, the wheels turn one direction
// and if the PORTB5 is low, the wheels turn the other direction, which can be observed by a oscillation scope
ISR(PCINT0_vect )
{
	LeftIn = check(PINB,4);
	RightIn = check(PINB,5);
	if (LeftIn==1)
	{
		//numPulse++;
		if (RightIn==0)
		{
			numPulse++;
			direction = 1;
		}else{
			numPulse--;
			direction = -1;
		}
	}
}
ISR(TIMER3_COMPA_vect) //ms interrupt
{
	Timer3Flag = 1;
	if (Timer3Flag2<INTERRUPT1S)
	{
		Timer3Flag2++;
	}
	//if (Timer3Flag3<INTERRUPT10MS)
	//{
		//Timer3Flag3++;
	//}
}