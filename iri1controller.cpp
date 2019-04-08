/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

//#define DEBUG_ALL
//#define DEBUG_STATES

CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{

	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set blue battery Sensor */
	m_seBlueBattery = (CBlueBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BLUE_BATTERY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor (SENSOR_ENCODER);
  m_seEncoder->InitEncoderSensor(m_pcEpuck);
	/* Set compass Sensor */
	m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor (SENSOR_COMPASS);

	charging = false;
	goToHospY = false;
	goToHospB = false;
	speed = 500;
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::avoid()
{
	double angle;
	double boolean;

	const double* directions = m_seProx->GetSensorDirections();
	const double* data = m_seProx->GetSensorReading(m_pcEpuck);

	double angleX = 0;
	double angleY = 0;
	double max = 0;

	for(int i= 0; i<m_seProx->GetNumberOfInputs(); i++)
	{
		angleX += data[i]*cos(directions[i]);
		angleY += data[i]*sin(directions[i]);
		if(data[i] > max)
		{
			max = data[i];
		}
	}

	angle = atan2(angleY, angleX) - M_PI;

	if(max > 0.65)
	{
		boolean = true;
	}
	else
	{
		boolean = false;
	}
	robot_state_t state = {angle, boolean};
	states[0] = state;		
	
}

void CIri1Controller::charge()
{
	double angle;
	double boolean;

	const double* directions = m_seRedLight->GetSensorDirections();
	const double* data = m_seRedLight->GetSensorReading(m_pcEpuck);

	double angleX = 0;
	double angleY = 0;

	for(int i= 0; i<m_seRedLight->GetNumberOfInputs(); i++)
	{
		angleX += data[i]*cos(directions[i]);
		angleY += data[i]*sin(directions[i]);
	}

	angle = atan2(angleY, angleX);

	if(m_seRedBattery->GetBatteryLevel() <= 0.15 || charging)
	{
		if(m_seRedBattery->GetBatteryLevel() >= 0.95)
		{
			charging = 1;
			boolean = 1;
		}
		else
		{
			charging = 0;
			boolean = 0;
		}
	}

	robot_state_t state = {angle, boolean};
	states[1] = state;		
	
}

void CIri1Controller::yellow()
{
	double angle;
	double boolean;

	const double* directions = m_seLight->GetSensorDirections();
	const double* data = m_seLight->GetSensorReading(m_pcEpuck);

	double angleX = 0;
	double angleY = 0;
	double max = 0;

	for(int i= 0; i<m_seLight->GetNumberOfInputs(); i++)
	{
		angleX += data[i]*cos(directions[i]);
		angleY += data[i]*sin(directions[i]);
		if(data[i] > max)
		{
			max = data[i];
		}
	}

	if(max > 0.95 && !goToHospY)
	{
		m_seLight->SwitchNearestLight(0);
		goToHospY = true;
	}

	angle = atan2(angleY, angleX);

	if(max == 0 || goToHospY)
	{
		boolean = false;
	}
	else
	{
		boolean = true;
	}
	//printf("YellowMax: %f YellowAngle: %f \n",max, angle);
	robot_state_t state = {angle, boolean};
	states[3] = state;
}

void CIri1Controller::blue()
{
	double angle;
	double boolean;

	const double* directions = m_seBlueLight->GetSensorDirections();
	const double* data = m_seBlueLight->GetSensorReading(m_pcEpuck);

	double angleX = 0;
	double angleY = 0;
	double max = 0;

	for(int i= 0; i<m_seBlueLight->GetNumberOfInputs(); i++)
	{
		angleX += data[i]*cos(directions[i]);
		angleY += data[i]*sin(directions[i]);
		if(data[i] > max)
		{
			max = data[i];
		}
	}

	if(max > 0.9 && (!goToHospY && !states[3].active))
	{
		m_seBlueLight->SwitchNearestLight(0);
	}

	if(max == 0)
	{
		goToHospB = true;
	}

	angle = atan2(angleY, angleX);

	if((max == 0 || goToHospB) && (!goToHospY && !states[3].active))
	{
		boolean = false;
	}
	else
	{
		boolean = true;
	}
	robot_state_t state = {angle, boolean};
	states[4] = state;
}

void CIri1Controller::gohosp()
{
	double angle;
	double boolean;

	const double* directions = m_seRedLight->GetSensorDirections();
	const double* data = m_seRedLight->GetSensorReading(m_pcEpuck);

	double angleX = 0;
	double angleY = 0;
	double max = 0;

	for(int i= 0; i<m_seRedLight->GetNumberOfInputs(); i++)
	{
		angleX += data[i]*cos(directions[i]);
		angleY += data[i]*sin(directions[i]);
		if(data[i] > max)
		{
			max = data[i];
		}
	}

	angle = atan2(angleY, angleX);

	if(max > 0.95)
	{
		goToHospB = false;
		goToHospY = false;
	}

	if(goToHospY || goToHospB)
	{
		boolean = true;
	}
	else
	{
		boolean = false;
	}
	robot_state_t state = {angle, boolean};
	states[2] = state;		
}

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{


	/* FASE 1: LECTURA DE SENSORES */

	/* Leer Sensores de Contacto */
	double* contact = m_seContact->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Luz Azul*/
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Luz Roja*/
	double* redlight = m_seRedLight->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Suelo */
	double* ground = m_seGround->GetSensorReading(m_pcEpuck);
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	/* Leer Battery Sensores de Suelo Memory */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);
	/* Leer Blue Battery Sensores de Suelo Memory */
	double* bluebattery = m_seBlueBattery->GetSensorReading(m_pcEpuck);
	/* Leer Red Battery Sensores de Suelo Memory */
	double* redbattery = m_seRedBattery->GetSensorReading(m_pcEpuck);
	/* Leer Encoder */
	double* encoder = m_seEncoder->GetSensorReading(m_pcEpuck);
	/* Leer Compass */
	double* compass = m_seCompass->GetSensorReading(m_pcEpuck);

	
	/* FASE 2: CONTROLADOR */
	
	/* Inicio Incluir las ACCIONES/CONTROLADOR a implementar */
	#ifdef DEBUG_ALL
	printf("CONTACT: ");
	for ( int i = 0 ; i < m_seContact->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", contact[i]);
	}
	printf("\n");
	
	printf("PROX: ");
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", prox[i]);
	}
	printf ("\n");
	
	printf("LIGHT: ");
	for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", light[i]);
	}
	printf ("\n");
	
	printf("BLUE LIGHT: ");
	for ( int i = 0 ; i < m_seBlueLight->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", bluelight[i]);
	}
	printf ("\n");
	
	printf("RED LIGHT: ");
	for ( int i = 0 ; i < m_seRedLight->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", redlight[i]);
	}
	printf ("\n");
	
	printf("GROUND: ");
	for ( int i = 0 ; i < m_seGround->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", ground[i]);
	}
	printf("\n");

	printf("GROUND MEMORY: ");
	for ( int i = 0 ; i < m_seGroundMemory->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", groundMemory[i]);
	}
	printf("\n");
	
	printf("BATTERY: ");
	for ( int i = 0 ; i < m_seBattery->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", battery[i]);
	}
	printf("\n");
	
	printf("BLUE BATTERY: ");
	for ( int i = 0 ; i < m_seBlueBattery->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", bluebattery[i]);
	}
	printf("\n");
	printf("RED BATTERY: ");
	for ( int i = 0 ; i < m_seRedBattery->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.3f ", redbattery[i]);
	}
	printf("\n");
	
  printf("ENCODER: ");
	for ( int i = 0 ; i < m_seEncoder->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.5f ", encoder[i]);
	}
	printf("\n");
  
  printf("COMPASS: ");
	for ( int i = 0 ; i < m_seCompass->GetNumberOfInputs() ; i ++ )
	{
		printf("%1.5f ", compass[i]);
	}
	printf("\n");

	/* Fin: Incluir las ACCIONES/CONTROLADOR a implementar */


	FILE* filePosition = fopen("outputFiles/robotPosition", "a");
	fprintf(filePosition," %2.4f %2.4f %2.4f %2.4f\n",
	f_time, m_pcEpuck->GetPosition().x,
	m_pcEpuck->GetPosition().y,
	m_pcEpuck->GetRotation());
	fclose(filePosition);
	#endif

	avoid();
	charge();
	gohosp();
	yellow();
	blue();
	
	#ifdef DEBUG_STATES
	printf("%%%%%%%%%%Información de estados%%%%%%%%%%\n");
	#endif

	robot_state_t state = {0 , false};

	for(int i= 0; i < 5; i++)
	{
		#ifdef DEBUG_STATES
		printf("Estado: %d; angle: %f; active: %d\n", i, states[i].angle, states[i].active);
		#endif
		if(states[i].active)
		{
		state = states[i];
		break;
		}
	}

	
	double rWheel = 0;
	double lWheel = 0;


	
	if(state.active)
	{
		if(state.angle >0)
		{
			rWheel = speed;
			lWheel = speed*cos(state.angle);
		}
		else if(state.angle <= 0)
		{
			rWheel = speed*cos(state.angle);
			lWheel = speed;
		}
	}

	m_acWheels->SetSpeed(lWheel,rWheel);

	#ifdef DEBUG_STATES
	printf("TotalAngle: %f VelX: %f VelY: %f \n", state.angle, rWheel, lWheel);
	#endif


	
}

/******************************************************************************/
/******************************************************************************/

