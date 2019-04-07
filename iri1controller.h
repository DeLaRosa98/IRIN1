#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_


/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/

typedef struct robot_state {
	double angle;
	bool active;
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

		robot_state_t states[5];
		bool charging;
		bool goToHospY;
		bool goToHospB;
		double speed;

};

#endif
