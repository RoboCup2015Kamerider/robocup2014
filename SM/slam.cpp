#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  

#include "slam.h"

void SlamController::MOVEFORWARD(int distance)
{
	_SET_VEL_(distance);
	_SLEEP_();
	STOP();
}

void SlamController::MOVEBACKWARD(int distance)
{
	TURNBACK();
	_SET_VEL_(distance);
	TURNBACK();
	_SLEEP_();
	STOP();
}

void SlamController::TURNLEFT()
{
	m_robot->setWheelVelocity(-PI / 4, PI / 4);
	_SLEEP_();
	STOP();
}

void SlamController::TURNRIGHT()
{
	m_robot->setWheelVelocity(PI / 4, -PI / 4);
	_SLEEP_();
	STOP();
}

void SlamController::TURNBACK()
{
	m_robot->setWheelVelocity(PI / 2, -PI / 2);
	_SLEEP_();
	STOP();
}

void SlamController::MOVELEFT()
{
	TURNLEFT();
	MOVEFORWARD(2.0);
	TURNRIGHT();
}

void SlamController::MOVERIGHT()
{
	TURNRIGHT();
	MOVEFORWARD(2.0);
	TURNLEFT();
}

void SlamController::STOP()
{
	m_robot->setWheelVelocity(0.0, 0.0);
}

void SlamController::_SET_VEL_(int distance)
{
	Vector3d* pV3d = GETHPOINTA(GETHLINE(3));
	if(pV3d != NULL)
	{
		double min_y = 1024.0;
		for(int i = 0; i < 320; i++)
		{
			if(fabs(pV3d[i].x()) <= 20)
			{
				if(min_y >  pV3d[i].z())
				{
					min_y = pV3d[i].z();
				}
			}
		}
		cout << "min_y = " << min_y << endl;
		if(min_y - 30 > distance)
		{
			m_robot->setWheelVelocity(distance / 10.0, distance / 10.0);
		}
		else
		{
			int v = floor(min_y - 30) / 10;
			m_robot->setWheelVelocity(v, v);
		}
	}
	else
	{
		m_robot->setWheelVelocity(0.0, 0.0);
	}
}

void SlamController::_SLEEP_()
{
	sleep(1.0);
}

int* SlamController::GETHLINE(int camID)
{
	if(m_view != NULL) 
	{ 
        	ViewImage *dis_img = m_view->distanceSensor1D(0.0, 255.0, camID, DEPTHBIT_8, IMAGE_320X1);
                char *dis_buf = dis_img->getBuffer(); 
                int width = dis_img->getWidth();
		int* data = new int[width];
		for(int i = 0; i < width; i++)
		{
			data[i] = int(dis_buf[i]);
		}
		return data;
	}
	return NULL;
}

Vector3d* SlamController::GETHPOINTA(int* data)
{
	if(data != NULL)
	{
		double fovy = m_robot->getCamFOV() * PI / 180.0; 
                double fovx = 2 * atan(tan(fovy * 0.5) * m_robot->getCamAS());
		int width = 320;

		Vector3d* result = new Vector3d[width];
		for(int i = 0; i < width; i++)
		{
                	double theta = fovx * i / (width - 1.0) - fovx / 2.0;
			result[i] = Vector3d(data[i] * sin(theta), 42, data[i] * cos(theta));
		}
		return result;
	}
	return NULL;
}

extern "C" Controller * createController() 
{  
  	return new SlamController;  
}  

