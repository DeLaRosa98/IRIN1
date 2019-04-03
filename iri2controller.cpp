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
#include "iri2controller.h"


extern gsl_rng* rng;
extern long int rngSeed;

using namespace std;

CIri2Controller::CIri2Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

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

}

/******************************************************************************/
/******************************************************************************/

CIri2Controller::~CIri2Controller()
{
}


/******************************************************************************/
/******************************************************************************/

void CIri2Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
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
	
	
	caso = 0;
	numAm = 0;
	numBl = 0;
	m_nLightObjectNumber = 5; //Idealmente se sacaría del archivo de parámetros
	m_nBlueLightObjectNumber = 4;
	totalLight = light[7] + light[0];
	totalBlueLight = bluelight[7] + bluelight[0];
	redSpeed[0] = redlight[0] + redlight[1] + redlight[2] + redlight[3]; //Sensores de luz del lado izquierdo
	redSpeed[1] = redlight[4] + redlight[5] + redlight[6] + redlight[7]; //Sensores de luz del lado derecho
	
	if(redbattery[0] <= 0.1) //Si ha pasado el turno se dirige a realizar el cambio de turno
	{
		m_acWheels->SetOutput(0, redSpeed[1]); //Para dirigirse a la luz que carga la bateria (realiza el cambio de turno) hacemos que las ruedas se muevan
  		m_acWheels->SetOutput(1, redSpeed[0]); //a la velocidad a la que les dicen los sensores del lado contrario.
	}
	else if(redbattery[0] >= 0.9) //Si ha realizado el cambio de turno se dirige a recoger a los pacientes
	{
		if(caso == 0) //Recoge uno a uno a los pacientes de mayor importancia
		{
			if ( totalLight >= 0.9)
			{
				m_seLight->SwitchNearestLight(0);
				numAm = numAm + 1;
				caso = 1;
			}
			if ( light[0] * light[7] == 0.0 )
			{
				double LightLeft 	= light[0] + light[1] + light[2] + light[3];
				double LightRight = light[4] + light[5] + light[6] + light[7];
				if ( LightLeft > LightRight )
				{
					m_acWheels->SetSpeed(-500,500);
				}
				else
				{
					m_acWheels->SetSpeed(500,-500);
				}
			}
			else
			{
				m_acWheels->SetSpeed(500,500);
			}	
		}
		else if (caso == 1) //Tras recoger un unico paciente va al hospital
		{
			if(compass[0] < 5.5 || compass[0] > 5.7)
			{
				m_acWheels->SetSpeed(-50,50);
			}
			else if((compass[0] > 5.5 || compass[0] < 5.7) &&(m_pcEpuck->GetPosition().x < 1.2) && (m_pcEpuck->GetPosition().y > -1.2))
			{
				m_acWheels->SetSpeed(500,500);
			}
			else if((compass[0] > 5.5 || compass[0] < 5.7) &&(m_pcEpuck->GetPosition().x >= 1.2) && (m_pcEpuck->GetPosition().y < -1.2))
			{
				if(numAm == m_nLightObjectNumber)
				{
					caso = 2; //Si ya ha recogido todos los pacientes va a recoger a los pacientes de menor importancia
				}
				else
				{
					caso = 0; //Si aun quedan pacientes de esta importancia vuelve al estado de recoger a los pacientes
				}
			}
		}
		else if (caso == 2) //Recoge a los pacientes de menor importancia
		{
			if ( totalBlueLight >= 0.9)
			{
				m_seLight->SwitchNearestLight(0);
				numBl = numBl + 1;
				if(numBl == m_nBlueLightObjectNumber)
				{
					caso = 3;
				}
			}
			if ( bluelight[0] * redlight[7] == 0.0 )
			{
				double blueLightLeft 	= bluelight[0] + bluelight[1] + bluelight[2] + bluelight[3];
				double blueLightRight = bluelight[4] + bluelight[5] + bluelight[6] + bluelight[7];
				if ( blueLightLeft > blueLightRight )
				{
					m_acWheels->SetSpeed(-500,500);
				}
				else
				{
					m_acWheels->SetSpeed(500,-500);
				}
			}
			else
			{
				m_acWheels->SetSpeed(500,500);
			}	
		}
		else if (caso == 3) // Tras recoger a todos los pacientes de menor importancia va al hospital
		{
			if(compass[0] < 5.5 || compass[0] > 5.7)
			{
				m_acWheels->SetSpeed(-50,50);
			}
			else if((compass[0] > 5.5 || compass[0] < 5.7) &&(m_pcEpuck->GetPosition().x < 1.2) && (m_pcEpuck->GetPosition().y > -1.2))
			{
				m_acWheels->SetSpeed(500,500);
			}
			else if((compass[0] > 5.5 || compass[0] < 5.7) &&(m_pcEpuck->GetPosition().x >= 1.2) && (m_pcEpuck->GetPosition().y < -1.2))
			{
				caso = 4;
			}	
		}
		else //ya ha recogido a todos los pacientes de todas las importancias y se queda parado en el hospital
		{
			printf("La ambulancia ha atendido a todos los pacientes");
			m_acWheels->SetSpeed(0,0);
		}
	}
	//m_acWheels->SetSpeed(500,500);

	/* Option 2: Speed between 0,1*/
	//m_acWheels->SetOutput(0,0.5);
	//m_acWheels->SetOutput(1,0.5);
	
}

/******************************************************************************/
/******************************************************************************/

