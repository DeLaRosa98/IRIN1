#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_

#define NUM_STATES 5

#define AVOID_PRIORITY   100.0
#define CHARGE_PRIORITY  10.0
#define GOHOSP_PRIORITY  5.0
#define YELLOW_PRIORITY  1.0
#define BLUE_PRIORITY  1.0

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/

typedef struct robot_state {
	double angle;
	bool active;
	double priority;
} robot_state_t;

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
	void avoid();
	void charge();
	void gohosp();
	void yellow();
	void blue();
    CEpuck* m_pcEpuck;
    
		CWheelsActuator* m_acWheels;
    	CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CRealBlueLightSensor* m_seBlueLight;
		CRealRedLightSensor* m_seRedLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;  
		CBlueBatterySensor* m_seBlueBattery;  
		CRedBatterySensor* m_seRedBattery;  
		CEncoderSensor* m_seEncoder;  
		CCompassSensor* m_seCompass;  

    	float m_fOrientation; 
    	dVector2 m_vPosition;

		int m_nWriteToFile;

		robot_state_t states[NUM_STATES];
		bool goToHosp;
		double speed;
		double maxB;
		bool charge_inhibitor;
		bool yellow_inhibitor;
		bool finish_inhibitor;
		int numB;
		int numY;

};

#endif
