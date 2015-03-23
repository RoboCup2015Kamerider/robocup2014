#include "ControllerEvent.h"  
#include "Controller.h"  
#include "Logger.h"  

#include <iostream>
#include <iomanip>
#include <vector>

#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>

#define	PI	3.14159265357978624
#define SCALE	20
#define GSCALE	2
#define DEG2RAD(DEG) ( (PI) * (DEG) / 180.0 )

using namespace std;

class GPoint
{
public:
	int x;
	int z;

	int f_state;	// 0 -> undetect, 1 -> posed, 2 -> no obs, 3 -> obs, 4 -> target, trash -> 5
public:
	GPoint()
	{
		x = 0;
		z = 0;
		f_state = 0;
	}
	GPoint(int x, int z, int f = 0)
	{
		this->x = x;
		this->z = z;
		f_state = f;
	}
};
class cordinate
{
public:
	int i;
	int j;
	
public:
	cordinate();
	cordinate(int i, int j);
	bool operator == (cordinate c);
	friend cordinate Cordinate(int i, int j);
};

class Dijkstra
{
protected:
	
	int height;
	int width;
	
	int **data;			//原始数据
	double **extdata;		//扩展数据
	
	cordinate destination;
	cordinate startpoint;
	
	std::vector<cordinate> OPEN;
	std::vector<cordinate> PATH;
	std::vector<cordinate> TRANS;
	
public:

	Dijkstra();
	Dijkstra(int h, int w);
	
	bool SetData(int **d);
	void SetDestination(int i, int j);
	void SetStart(int i, int j);
	std::vector<cordinate> GetPath();
	std::vector<cordinate> GetTrans();
	bool Calculate();

	~Dijkstra();
	
private:

	inline double& EXT(cordinate c);
	inline double& EXT(cordinate c, int a, int b);
};

cordinate::cordinate()
{
}
cordinate::cordinate(int i, int j)
{
	this->i = i;
	this->j = j;
}
bool cordinate::operator == (cordinate c)
{
	return (this->i == c.i && this->j == c.j);
}

cordinate Cordinate(int i, int j)
{
	cordinate c(i, j);
	return c;
}

//class Dijkstra
Dijkstra::Dijkstra()
{
}

Dijkstra::Dijkstra(int h, int w)
{
	this->height = h;
	this->width = w;
}

Dijkstra::~Dijkstra()
{
	for(int i= 0; i < height; i++)
	{
		delete[] extdata[i];
	}
	delete[] extdata;
}

double& Dijkstra::EXT(cordinate c)
{
	return extdata[c.i][c.j];
}

double& Dijkstra::EXT(cordinate c, int a, int b)
{
	return extdata[c.i + a][c.j + b];
}

bool Dijkstra::SetData(int **d) 
{
	if(data == NULL)
	{
		return false;
	}
	int i, j;
	this->data = d;
	extdata = new double*[height];
	for(i = 0; i < height; i++)
	{
		extdata[i] = new double[width];
		for(j = 0; j < width; j++)
		{
			extdata[i][j] = 0;
		}
	}
/*	for(i = 1; i < height - 1; i++)
	{
		for(j = 1; j < width - 1; j++)
		{
			if(data[i][j])
			{
				extdata[i][j] = -3;
				extdata[i][j - 1] = -3;
				extdata[i][j + 1] = -3;
				extdata[i - 1][j] = -3;
				extdata[i - 1][j - 1] = -3;
				extdata[i - 1][j + 1] = -3;
				extdata[i + 1][j] = -3;
				extdata[i + 1][j - 1] = -3;
				extdata[i + 1][j + 1] = -3;
			}
		}
	}*/
	for(i = 2; i < height - 2; i++)
	{
		for(j = 2; j < width - 2; j++)
		{
			if(data[i][j])
			{
				for(int m = -2; m <= 2; m++)
				{
					for(int n = -2; n <= 2; n++)
					{
						extdata[i + m][j + n] = -3;
					}
				}
				/*extdata[i][j] = -3;
				extdata[i][j - 1] = -3;
				extdata[i][j + 1] = -3;
				extdata[i - 1][j] = -3;
				extdata[i - 1][j - 1] = -3;
				extdata[i - 1][j + 1] = -3;
				extdata[i + 1][j] = -3;
				extdata[i + 1][j - 1] = -3;
				extdata[i + 1][j + 1] = -3;*/
			}
		}
	}
	/*for(i = 0; i < height; i++)
	{
		for(j = 0; j < width; j++)
		{
			if(extdata[i][j] == -3)
			{
				data[i][j] = 1;
			}
			else
			{
				data[i][j] = 0;
			}
		}
	}
	for(i = 1; i < height - 1; i++)
	{
		for(j = 1; j < width - 1; j++)
		{
			if(data[i][j])
			{
				extdata[i][j] = -3;
				extdata[i][j - 1] = -3;
				extdata[i][j + 1] = -3;
				extdata[i - 1][j] = -3;
				extdata[i - 1][j - 1] = -3;
				extdata[i - 1][j + 1] = -3;
				extdata[i + 1][j] = -3;
				extdata[i + 1][j - 1] = -3;
				extdata[i + 1][j + 1] = -3;
			}
		}
	}*/
	
	return true;
}

void Dijkstra::SetDestination(int i, int j)
{
	destination = Cordinate(i, j);
}

void Dijkstra::SetStart(int i, int j)
{
	startpoint = Cordinate(i, j);
	EXT(startpoint) = 1;
}

std::vector<cordinate> Dijkstra::GetPath()
{
	return PATH;
}

std::vector<cordinate> Dijkstra::GetTrans()
{
	return TRANS;
}

bool Dijkstra::Calculate()
{
	OPEN.push_back(startpoint);
	
	int i;
	/*for(i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if(extdata[i][j] == 0)
			{
				cout << ".";
			}
			else
			{
				cout << "X";
			}
		}
		cout << endl;
	}*/


	bool flag = true;
	while(flag && OPEN.size() < 8000)
	{
		int nsize = OPEN.size();
		//cout << "OPEN.size() = " << nsize << endl;
		for(i = 0; i < nsize; i++)
		{
			if(OPEN[i].i - 1 >= 0)
			{
				if(OPEN[i].j - 1 >= 0)
				{
					if(extdata[OPEN[i].i - 1][OPEN[i].j - 1] != -3)
					{
						cordinate t(OPEN[i].i - 1, OPEN[i].j - 1);
						
						double nv = EXT(OPEN[i]) + 2.5;
						
						if(EXT(t) == 0 || nv < EXT(t))
						{
							EXT(t) = nv;
							OPEN.push_back(t);
						}
					}
				}
				if(OPEN[i].j + 1 < width)
				{
					if(EXT(OPEN[i], -1, 1) != -3)
					{
						cordinate t(OPEN[i].i - 1, OPEN[i].j + 1);
						double nv = EXT(OPEN[i]) + 2.5;
						
						if(EXT(t) == 0 || nv < EXT(t))
						{
							EXT(t) = nv;
							OPEN.push_back(t);
						}
					}
				}
				if(EXT(OPEN[i], -1, 0) != -3)
				{	
					cordinate t(OPEN[i].i - 1, OPEN[i].j);
					
					double nv = EXT(OPEN[i]) + 1;
					
					if(EXT(t) == 0 || nv < EXT(t))
					{
						EXT(t) = nv;
						OPEN.push_back(t);
					}
				}
			}
			if(OPEN[i].i + 1 < height)
			{
				if(OPEN[i].j - 1 >= 0)
				{
					if(EXT(OPEN[i], 1, -1) != -3)
					{
						cordinate t(OPEN[i].i + 1, OPEN[i].j - 1);
						
						double nv = EXT(OPEN[i]) + 2.5;
						
						if(EXT(t) == 0 || nv < EXT(t))
						{
							EXT(t) = nv;
							OPEN.push_back(t);
						}
					}
				}
				if(OPEN[i].j + 1 < width)
				{
					if(extdata[OPEN[i].i + 1][OPEN[i].j + 1] != -3)
					{
						cordinate t(OPEN[i].i + 1, OPEN[i].j + 1);
						
						double nv = extdata[OPEN[i].i][OPEN[i].j] + 2.5;
						
						if(EXT(t) == 0 || nv < EXT(t))
						{
							EXT(t) = nv;
							OPEN.push_back(t);
						}
					}
				}
				if(EXT(OPEN[i], 1, 0) != -3)
				{
					cordinate t(OPEN[i].i + 1, OPEN[i].j);
					
					double nv = extdata[OPEN[i].i][OPEN[i].j] + 1;
					
					if(EXT(t) == 0 || nv < EXT(t))
					{
						EXT(t) = nv;
						OPEN.push_back(t);
					}
				}
			}
			if(OPEN[i].j - 1 >= 0)
			{
				if(EXT(OPEN[i], 0, -1) != -3)
				{
					cordinate t(OPEN[i].i, OPEN[i].j - 1);
					
					double nv = extdata[OPEN[i].i][OPEN[i].j] + 1;
					
					if(EXT(t) == 0 || nv < EXT(t))
					{
						EXT(t) = nv;
						OPEN.push_back(t);
					}
				}
			}
			if(OPEN[i].j + 1 < width)
			{
				if(EXT(OPEN[i], 0, 1) != -3)
				{
					cordinate t(OPEN[i].i, OPEN[i].j + 1);
					double nv = extdata[OPEN[i].i][OPEN[i].j] + 1;
					if(EXT(t) == 0 || nv < EXT(t))
					{
						EXT(t) = nv;
						OPEN.push_back(t);
					}
					
				}
			}
		}
		
		for(i = 0; i < OPEN.size(); i++)
		{
			if(OPEN[i] == destination)
			{
				flag = false;
				break;
			}
		}
	}
	if(flag == true)
	{
		cout << "flag = true" << endl;
		return false;
	}
	double best = EXT(destination);
	flag = true;
	PATH.push_back(destination);
	int index = 0;
	double min = best + 1;
	double value;
	while(flag)
	{
		cordinate ic;
		if(PATH[index].i - 1 >= 0)
		{
			if(PATH[index].j - 1 >= 0)
			{
				value = EXT(PATH[index], -1, -1);
				if(value < min && value != 0 && value != -3)
				{
					min = value;
					ic.i = PATH[index].i - 1;
					ic.j = PATH[index].j - 1;
				}
			}
			if(PATH[index].j + 1 < width)
			{
				value = EXT(PATH[index], -1, 1);
				if(value < min && value != 0 && value != -3)
				{
					min = value;
					ic.i = PATH[index].i - 1;
					ic.j = PATH[index].j + 1;
				}
			}
			value = EXT(PATH[index], -1, 0);
			if(value < min && value != 0 && value != -3)
			{
				min = value;
				ic.i = PATH[index].i - 1;
				ic.j = PATH[index].j;
			}
		}
		if(PATH[index].i + 1 < height)
		{
			if(PATH[index].j - 1 >= 0)
			{
				value = EXT(PATH[index], 1, -1);
				if(value < min && value != 0 && value != -3)
				{
					min = value;
					ic.i = PATH[index].i + 1;
					ic.j = PATH[index].j - 1;
				}
			}
			if(PATH[index].j + 1 < width)
			{
				value = EXT(PATH[index], 1, 1);
				if(value < min && value != 0 && value != -3)
				{
					min = value;
					ic.i = PATH[index].i + 1;
					ic.j = PATH[index].j + 1;
				}
			}
			value = EXT(PATH[index], 1, 0);
			if(value < min && value != 0 && value != -3)
			{
				min = value;
				ic.i = PATH[index].i + 1;
				ic.j = PATH[index].j;
			}
		}
		if(PATH[index].j - 1 >= 0)
		{
			value = EXT(PATH[index], 0, -1);
			if(value < min && value != 0 && value != -3)
			{
				min = value;
				ic.i = PATH[index].i;
				ic.j = PATH[index].j - 1;
			}
		}
		if(PATH[index].j + 1 < width)
		{
			value = EXT(PATH[index], 0, 1);
			if(value < min && value != 0 && value != -3)
			{
				min = value;
				ic.i = PATH[index].i;
				ic.j = PATH[index].j + 1;
			}
		}
		PATH.push_back(ic);
		index++;
		if(min == 1)
		{
			flag = false;
			break;
		}
	}
	std::vector<double> v;
	for(i = PATH.size() - 1; i >= 0; i--)
	{
		if(i > 0)
		{
			v.push_back(EXT(PATH[i]) - EXT(PATH[i - 1]));
		}
		data[PATH[i].i][PATH[i].j] = -4;
	}
	value = v[0];
	for(i = 1; i < v.size(); i++)
	{
		if(v[i] != value)
		{
			value = v[i];
			data[PATH[PATH.size() - 1 - i].i][PATH[PATH.size() -  1 - i].j] = -1;
			TRANS.push_back(PATH[PATH.size() - 1 - i]);
		}
	}
	cout << "flag = false" << endl;
	return true;
}

class GGraphic
{
public:
	IplImage* map;
	int i_shift;
	int j_shift;
	int scale;

	int height;
	int width;

	unsigned char* imagedata;
	int step;

public:
	void Add(int gi, int gj, CvScalar cvs)
	{
		for(int i = gi * scale + i_shift; i < gi * scale + i_shift + scale; i++)
		{
			for(int j = gj * scale + j_shift; j < gj * scale + j_shift + scale; j++)
			{
				//cout << i << "\t" << j << "\t" << cvs.val[0] << endl;
				imagedata[i * step + j * 3] = cvs.val[0];
				imagedata[i * step + j * 3 + 1] = cvs.val[1];
				imagedata[i * step + j * 3 + 2] = cvs.val[2];
			}
		}
	}
	void clear()
	{
		for(int i = 0; i < height; i++)
		{
			for(int j = 0; j < width; j++)
			{
				imagedata[i * step + j * 3] = 255;
				imagedata[i * step + j * 3 + 1] = 255;
				imagedata[i * step + j * 3 + 2] = 255;
			}
		}
	}
	GGraphic()
	{
		width = 600;
		height = 400;
		map = cvCreateImage(cvSize(width, height), 8, 3);
		imagedata = (unsigned char*)map->imageData;
		step = map->widthStep / sizeof(unsigned char);

		i_shift = height / 2;
		j_shift = width / 2;
		scale = GSCALE;
		clear();
		
	}
	void GDisplay()
	{	
		//cvSaveImage("1.bmp", map);
		cvNamedWindow("GGraphic");
		cvShowImage("GGraphic", map);
		cvWaitKey(100);
		//cvDestroyWindow("GGraphic");
	}
};

class GridController : public Controller 
{  
public:  
  	void onInit(InitEvent &evt);  
  	double onAction(ActionEvent&);  
  	void onRecvMsg(RecvMsgEvent &evt); 
  	void onCollision(CollisionEvent &evt); 

public:
	RobotObj	*m_robot;
	ViewService	*m_view;

public:
	void MOVEFORWARD();
	void MOVEBACKWARD();
	void TURNLEFT();
	void TURNRIGHT();
	void STOP();

public:
	bool b_move;
	bool b_grasp;
	bool b_coll;
	int m_state;
	int m_target;
	
	bool b_TrashRedD;
	bool b_TrashGreenD;
	bool b_TrashBlueD;
	bool b_WagonD;
	
public:
	int f_dirX;
	int f_dirZ;

	int f_posX;
	int f_posZ;

	int d_posX;
	int d_posZ;

	int d_cX;
	int d_cZ;
	int amount_c;

	int d_ncX;
	int d_ncZ;
	
	int d_tX;
	int d_tZ;

	int d_tX0;
	int d_tZ0;
	int amount_t0;
	
	int d_tX1;
	int d_tZ1;
	int amount_t1;
	
	int d_tX2;
	int d_tZ2;
	int amount_t2;
	
	int d_tX3;
	int d_tZ3;
	int amount_t3;
	
	int DIS;
	bool is_near;

	vector<GPoint> map_d;
	GGraphic g_map;
	
	vector<int> prc_m;
	vector<int> prc_d;

public:
	int* GETHLINE(int camID);
	Vector3d* GETHPOINTA(int* data);
	Vector3d* GETPLANT(int camID);
	int GETPOINTDIS(CvPoint cpt, cordinate& cor, int camID);
	int GETPOINTDIS2(CvPoint cpt, cordinate& cor, int camID);
	int FindObs(int camID);
	void Detect();
	void MapInit();

public:
	vector<cordinate> GetPath(int pX, int pZ);
	void MOVETO(int pX, int pZ);
	void MOVEAROUND();
	bool MOVEPATH(int X, int Z);
	void GETNEARBY(int& dx, int& dz);
	bool MOVENEARBY(int x, int z, int allow);
	void MOVETOWARD(int dx, int dz);

public:
	bool DetectTrashRed(CvPoint& cpt, int camID);
	bool DealDetectTrashRed();
	bool DetectTrashGreen(CvPoint& cpt, int camID);
	bool DealDetectTrashGreen();
	bool DetectTrashBlue(CvPoint& cpt, int camID);
	bool DealDetectTrashBlue();
	bool DetectCup(CvPoint& cpt,int camID);
	bool DealDetectCup();
	
	
	bool DetectCococup(CvPoint& cpt, int camID);
	bool DealDetectCococup();
	void TRANS(cordinate orgin, cordinate& world);
	
	bool DetectWagon(CvPoint& cpt, int camID);
	bool DealDetectWagon();

public:
	void TOUCH();
	void STOPTOUCH();
	void HOLDARM(int deg);
	void HOLDHAND(int deg);
	void HOLDBACKARM();
	void MOVEHAND();
	void INITARM();
	void THROWTRASH();
	void PREPARETHROW(int deg);
	
public:
	void DIRECTIONCORRECTION(int dir);
	void POSITIONCORRECTION(int pos);
	
	void BackToGridMap();
	bool preCorrection(int x, int dis);
	bool correction(int x, int dis);
};  
void GridController::MapInit()
{
	m_target = 3;
	
	b_TrashRedD = false;
	b_TrashGreenD = false;
	b_TrashBlueD = false;
	b_WagonD = false;

	f_dirX = 0;
	f_dirZ = 1;

	f_posX = 0;
	f_posZ = 0;

	d_posX = 0;
	d_posZ = 0;

	// 识别到的位置
	d_cX = 0;
	d_cZ = 0;
	amount_c = 0;

	// 附近的位置
	d_ncX = 0;
	d_ncZ = 0;

	d_tX0 = 0;
	d_tZ0 = 0;
	amount_t0 = 0;
	
	d_tX1 = 0;
	d_tZ1 = 0;
	amount_t1 = 0;
	
	d_tX2 = 0;
	d_tZ2 = 0;
	amount_t2 = 0;
	
	d_tX3 = 0;
	d_tZ3 = 0;
	amount_t3 = 0;

	m_state = 0;
	is_near = false;


	map_d.push_back(GPoint(0, 0, 1));

	g_map.Add(0, 0, cvScalar(0, 0, 255));
	Detect();
	
}
void GridController::MOVETOWARD(int dx, int dz)
{
	if(fabs(dx) > fabs(dz))
	{
		if(dz != 0)
		{
			while(dz * f_dirZ <= 0)
			{
				TURNLEFT();
			}
			MOVEFORWARD();
		}
		while(dx * f_dirX <= 0)
		{
			TURNRIGHT();
		}
	}
	else
	{
		if(dx != 0)
		{
			while(dx * f_dirX <= 0)
			{
				TURNRIGHT();
			}
		}
		MOVEFORWARD();
		while(dz * f_dirZ <= 0)
		{
			TURNLEFT();
		}
	}
	MOVEBACKWARD();
}
void GridController::onInit(InitEvent &evt) 
{  
	m_robot = getRobotObj(myname());
	m_view = (ViewService*)connectToService("SIGViewer");
	
	b_move = false;
	b_grasp = true;
	b_coll = false;

	MapInit();
	INITARM();
	DIS = 200;

	m_robot->setWheel(10.0, 10.0);
}  

double GridController::onAction(ActionEvent &evt) 
{  
	static int y = 120;
	static int nearTime = 0;
	cout << "\t\tf_pos = " << f_posX << ", " << f_posZ << 
		"\tf_dir = " << f_dirX << ", " << f_dirZ <<
		"\td_pos = " << d_posX << ", " << d_posZ <<
		"\tm_state = " << m_state << endl;
	static int fg = 0;
	if(b_move == false)
	{
		fg = 0;
		cout << "b_move false" << endl;
		return 1.0;
	}
	if(m_state == 0)
	{
		TURNRIGHT();
		fg++;
		if(fg < 4)
		{
			m_state = 10;
		}
		else
		{
			m_state = 5;
		}
	}
	else if(m_state == 5)
	{
		MOVEAROUND();
		m_state = 10;
	}
	else if(m_state == 10)
	{
		if(DealDetectCup())/////
		{
			d_posX = d_cX;
			d_posZ = d_cZ;
			m_state = 11;
		}
		else if(MOVEPATH(d_posX, d_posZ))
		{
			if(d_cX == 0 && d_cZ == 0)
			{
				m_state = 0;
			}
		}
		b_TrashBlueD = DealDetectTrashBlue();
		b_TrashGreenD = DealDetectTrashGreen();
		b_TrashRedD = DealDetectTrashRed();
		b_WagonD = DealDetectWagon();
	}
	else if(m_state == 11)
	{
		DIS = 5;
		DealDetectCup();////
		if(MOVEPATH(d_posX, d_posZ))
		{
			cout << "come" << endl;
			if(MOVENEARBY(d_cX, d_cZ, 17))
			{
				cout << "NEAR BY" << endl;
				int dx = d_cX - f_posX;
				int dz = d_cZ - f_posZ;
				MOVETOWARD(dx, dz);
				cout << "TOWARD" << endl;
				
				m_state = 20;
			}
			else
			{
				nearTime++;
			}
			if((nearTime + 1) % 12 == 0)
			{
				int dx = d_cX - f_posX;
				int dz = d_cZ - f_posZ;
				MOVETOWARD(dx, dz);
				MOVEFORWARD();
				MOVEFORWARD();
			}
		}
	}
	// 粗调
	else if(m_state == 20)
	{
	        CvPoint cpt;
		cout << "in 20" << endl;
		is_near = true;
		if(DetectCup(cpt, 1))////
		{     
			cordinate cor;
			int dis = GETPOINTDIS2(cpt, cor, 1);  
			if(preCorrection(cpt.x, dis) == true)
			{
				
				m_state = 30;
			}
		}
		else
		{
			m_state = 11;
		}
	}
	// 微调
	else if(m_state == 30)
	{
		CvPoint cpt;
		cout << "\tin 30" << endl;
		
		if(DetectCup(cpt, 1))////
		{
			cordinate cor;
			int dis = GETPOINTDIS2(cpt, cor, 1);  
		        if(correction(cpt.x, dis) == true)
		        {
		        	y = cpt.y;
		        	m_state = 40;
		        }
		}
		else
		{
			m_state = 20;
		}
	}
	// 手部抓取
	else if(m_state == 40)
	{
		if(b_grasp == true)
		{
		        HOLDARM(0);
		        HOLDHAND((y - 120) / 5);
		        cout << "h h = " << y << "\t" << (y - 120) / 5 << endl;
		        sleep(3.0);
		        if(b_grasp == true)
		        {
		        	MOVEHAND();
		        	sleep(3.0);
		        }
		}
		else
		{
			HOLDBACKARM();
			// 回调到网格地图
			BackToGridMap();
			MOVEBACKWARD();
			
			m_state = 50;
		}
	}
	// 重设目标
	else if(m_state == 50)
	{
		cout << "suc grasp " << m_target << endl;
		if(b_TrashBlueD == true && m_target == 2)
		{
			d_posX = d_tX2;
			d_posZ = d_tZ2;
			d_tX = d_tX2;
			d_tZ = d_tZ2;
			m_state = 60;
		}
		else if(b_TrashRedD == true && m_target == 1)
		{
			d_posX = d_tX1;
			d_posZ = d_tZ1;
			d_tX = d_tX1;
			d_tZ = d_tZ1;
			m_state = 60;
		}
		else if(b_TrashGreenD == true && m_target == 0)
		{
			d_posX = d_tX0;
			d_posZ = d_tZ0;
			d_tX = d_tX0;
			d_tZ = d_tZ0;
			m_state = 60;
		}
		else if(b_WagonD == true && m_target == 3)
		{
			d_posX = d_tX3;
			d_posZ = d_tZ3;
			d_tX = d_tX3;
			d_tZ = d_tZ3;
			m_state = 60;
		}
		else
		{
			MOVEAROUND();
			m_state = 65;
		}
	}	
	else if(m_state == 60)
	{
		if(MOVEPATH(d_posX, d_posZ))
		{
			if(MOVENEARBY(d_tX, d_tZ, 29))
			{
				cout << "\tNear Trash" << endl;
				m_state = 70;
			}
		}
	}
	else if(m_state == 65)
	{
		MOVEPATH(d_posX, d_posZ);
		b_TrashBlueD = DealDetectTrashBlue();
		b_TrashGreenD = DealDetectTrashGreen();
		b_TrashRedD = DealDetectTrashRed();
		b_WagonD = DealDetectWagon();
		m_state = 50;
	}
	else if(m_state == 70)
	{
		int dx = d_tX - f_posX;
		int dz = d_tZ - f_posZ;
		MOVETOWARD(dx, dz);
		m_state = 80;
	}
	else if(m_state == 80)
	{
		CvPoint cpt;
		cout << "in 80" << endl;
		if(m_target == 0)
		{
			if(DetectTrashGreen(cpt, 3))
			{     
				 cordinate cor;
				 int dis = GETPOINTDIS2(cpt, cor, 3);  
				 if(preCorrection(cpt.x, dis) == true)
				 {
				 	m_state = 90;
				 }
			}
		}
		else if(m_target == 1)
		{
			if(DetectTrashRed(cpt, 3))
			{     
				 cordinate cor;
				 int dis = GETPOINTDIS2(cpt, cor, 3);  
				 if(preCorrection(cpt.x, dis) == true)
				 {
				 	m_state = 90;
				 }
			}
		}
		else if(m_target == 2)
		{
			if(DetectTrashBlue(cpt, 3))
			{     
				 cordinate cor;
				 int dis = GETPOINTDIS2(cpt, cor, 3);  
				 if(preCorrection(cpt.x, dis) == true)
				 {
				 	m_state = 90;
				 }
			}
		}
		else if(m_target == 3)
		{
			if(DetectWagon(cpt, 3))
			{     
				 cordinate cor;
				 int dis = GETPOINTDIS2(cpt, cor, 3);  
				 if(preCorrection(cpt.x, dis - 50) == true)
				 {
				 	m_state = 90;
				 }
			}
		}
	}
	else if(m_state == 90)
	{
		PREPARETHROW(0);
		THROWTRASH();
		INITARM();
		MOVEBACKWARD();
		MOVEBACKWARD();
		//std::string msg = "Task_finish";
		//broadcastMsg(msg);
		is_near = false;
		MapInit();
		fg = 0;
		//b_move = false;
		m_state = 0;
	}

	//g_map.GDisplay();
  	return 1.0;      
}  
  
void GridController::onRecvMsg(RecvMsgEvent &evt) 
{  
	std::string sender = evt.getSender();
	std::string msg = evt.getMsg();
	if(strcmp(msg.c_str(), "Task_start") == 0)
	{
		m_state = 0;
		b_move = true;
	}
	else if(strcmp(msg.c_str(), "Task_end") == 0)
	{
		STOP();
		b_move = false;
	}
	else if(strcmp(msg.c_str(), "Time_over") == 0)
	{
		STOP();
		MapInit();
		b_move = false;
	}
	
	else if(strcmp(msg.c_str(), "mf") == 0)
	{
		MOVEFORWARD();
	}
	else if(strcmp(msg.c_str(), "mb") == 0)	
	{
		MOVEBACKWARD();
	}
	else if(strcmp(msg.c_str(), "tl") == 0)	
	{
		TURNLEFT();
	}
	else if(strcmp(msg.c_str(), "tr") == 0)
	{
		TURNRIGHT();
	}
	else if(strcmp(msg.c_str(), "map") == 0)
	{
		for(int i = 0; i < map_d.size(); i++)
		{
			cout << i << "\t" << map_d[i].x << "\t" << map_d[i].z << "\t" << map_d[i].f_state << endl;
		}
	}
	else if(strcmp(msg.c_str(), "ma") == 0)
	{
		MOVEAROUND();
	}
	else if(strcmp(msg.c_str(), "frt") == 0)
	{
		CvPoint cpt;
		if(DetectTrashRed(cpt, 3))
		{
			cout << "cpt = " << cpt.x << "\t" << cpt.y << endl;
			cordinate cor;
			int dis = GETPOINTDIS2(cpt, cor, 3);
			cout << "dis = " << dis << "\t" << cor.i << "\t" << cor.j << endl;
			cordinate w;
			TRANS(cor, w);
			cout << "world = " << w.i << "\t" << w.j << endl;
			g_map.Add(w.j, w.i, cvScalar(0, 255, 255));
			map_d.push_back(GPoint(w.i, w.j, 5));
			d_posX = w.i;
			d_posZ = w.j;
		}
	}
	else if(strcmp(msg.c_str(), "fcc") == 0)
	{
		CvPoint cpt;
		if(DetectCococup(cpt, 1))
		{
			cout << "cpt = " << cpt.x << "\t" << cpt.y << endl;
			cordinate cor;
			int dis = GETPOINTDIS2(cpt, cor, 1);
			cout << "dis = " << dis << "\t" << cor.i << "\t" << cor.j << endl;
			cordinate w;
			TRANS(cor, w);
			cout << "world = " << w.i << "\t" << w.j << endl;
			g_map.Add(w.j, w.i, cvScalar(255, 0, 255));
			map_d.push_back(GPoint(w.i, w.j, 4));
			d_posX = w.i;
			d_posZ = w.j;
		}
	}
	else if(strcmp(msg.c_str(), "dw") == 0)
	{
		CvPoint cpt;
		DetectWagon(cpt, 3);
	}
	else if(strcmp(msg.c_str(), "m") == 0)
	{
		b_move = ! b_move;
	}
	else if(strcmp(msg.c_str(), "c") == 0)
	{   
		if(m_view != NULL) 
		{   
			ViewImage *img = m_view->captureView(1, COLORBIT_24, IMAGE_320X240);   
			if (img != NULL) 
			{   
				img->saveAsWindowsBMP("cap.bmp");  
  
				delete img;   
      			}
			ViewImage *dimg = m_view->distanceSensor2D(0.0, 1023.0, 1, DEPTHBIT_8, IMAGE_320X240);
			if (dimg != NULL) 
			{   
				dimg->saveAsWindowsBMP("dcap.bmp");  
  
				delete dimg;   
      			}

   		}
	}
	else if(strcmp(msg.c_str(), "c3") == 0)
	{   
		if(m_view != NULL) 
		{   
			ViewImage *img = m_view->captureView(3, COLORBIT_24, IMAGE_320X240);   
			if (img != NULL) 
			{   
				img->saveAsWindowsBMP("cap3.bmp");  
  
				delete img;   
      			}
			ViewImage *dimg = m_view->distanceSensor2D(0.0, 1023.0, 3, DEPTHBIT_8, IMAGE_320X240);
			if (dimg != NULL) 
			{   
				dimg->saveAsWindowsBMP("dcap3.bmp");  
  
				delete dimg;   
      			}

   		}
	}
	else if(strcmp(msg.c_str(), "ha") == 0)
	{
		HOLDARM(55);
	}
	else if(strcmp(msg.c_str(), "ts") == 0)
	{
		TOUCH();
	}
	else if(strcmp(msg.c_str(), "te") == 0)
	{
		STOPTOUCH();
	}
	else if(strcmp(msg.c_str(), "ht") == 0)
	{
		m_robot->setJointVelocity("LARM_JOINT6", 1.0, 1.0);
		sleep(1.0);
		m_robot->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
	}
	
}  

void GridController::onCollision(CollisionEvent &evt) 
{
	if(b_grasp)
	{		
		
		const vector<string> & with = evt.getWith(); 
		const vector<string> & mparts = evt.getMyParts();

		for(int i = 0; i < with.size(); i++)
		{   
			if(mparts[i] == "LARM_LINK7" || mparts[i] == "LARM_LINK4")
			{ 
				CParts * parts = m_robot->getParts("LARM_LINK7");
				if(parts->graspObj(with[i])) 
				{
					if(with[i] == "petbottle_0")
					{
						m_target = 3;
					}
					else if(with[i] == "petbottle_1")
					{
						m_target = 0;
					}
					else if(with[i] == "petbottle_2")
					{
						m_target = 3;
					}
					else if(with[i] == "petbottle_3")
					{
						m_target = 0;
					}
					else if(with[i] == "petbottle_4")
					{
						m_target = 3;
					}
					else if(with[i] == "petbottle_5")
					{
						m_target = 0;
					}
					else if(with[i] == "can_0")
					{
						m_target = 2;
					}
					else if(with[i] == "can_1")
					{
						m_target = 2;
					}
					else if(with[i] == "can_2")
					{
						m_target = 2;
					}
					else if(with[i] == "can_3")
					{
						m_target = 2;
					}
					else if(with[i] == "mayonaise_0")
					{
						m_target = 3;
					}
					else if(with[i] == "mayonaise_1")
					{
						m_target = 0;
					}
					else if(with[i] == "mugcup")
					{
						m_target = 3;
					}
					else if(with[i] == "banana")
					{
						m_target = 1;
					}
					else if(with[i] == "apple")
					{
						m_target = 1;
					}
					b_grasp = false;
				}
			}
		}
		
	}
}
bool GridController::DealDetectWagon()
{
	if(amount_t3 > 10)
	{
		return false;
	}
	CvPoint cpt;
	if(DetectWagon(cpt, 3))
	{
		cout << "cpt = " << cpt.x << "\t" << cpt.y << endl;
		cordinate cor;
		int dis = GETPOINTDIS2(cpt, cor, 3);
		if(dis > 500)
		{
			return false;
		}
		cout << "dis = " << dis << "\t" << cor.i << "\t" << cor.j << endl;
		cordinate w;
		TRANS(cor, w);
		cout << "world = " << w.i << "\t" << w.j << endl;
		g_map.Add(w.j, w.i, cvScalar(0, 255, 255));
		map_d.push_back(GPoint(w.i, w.j, 5));
		d_tX3 = (d_tX3 * amount_t3 + w.i) / (amount_t3 + 1);
		d_tZ3 = (d_tZ3 * amount_t3 + w.j) / (amount_t3 + 1);
		amount_t3++;
		cout << "TRASHRED = " << d_tX3 << "\t" << d_tZ3 << endl;
		return true;
	}
	return false;
}
bool GridController::DealDetectTrashBlue()
{
	if(amount_t2 > 10)
	{
		return false;
	}
	CvPoint cpt;
	if(DetectTrashBlue(cpt, 3))
	{
		cout << "cpt = " << cpt.x << "\t" << cpt.y << endl;
		cordinate cor;
		int dis = GETPOINTDIS2(cpt, cor, 3);
		cout << "dis = " << dis << "\t" << cor.i << "\t" << cor.j << endl;
		cordinate w;
		TRANS(cor, w);
		cout << "world = " << w.i << "\t" << w.j << endl;
		g_map.Add(w.j, w.i, cvScalar(0, 255, 255));
		map_d.push_back(GPoint(w.i, w.j, 5));
		d_tX2 = (d_tX2 * amount_t2 + w.i) / (amount_t2 + 1);
		d_tZ2 = (d_tZ2 * amount_t2 + w.j) / (amount_t2 + 1);
		amount_t2++;
		cout << "TRASHRED = " << d_tX2 << "\t" << d_tZ2 << endl;
		return true;
	}
	return false;
}
bool GridController::DealDetectTrashGreen()
{
	if(amount_t0 > 10)
	{
		return false;
	}
	CvPoint cpt;
	if(DetectTrashGreen(cpt, 3))
	{
		cout << "cpt = " << cpt.x << "\t" << cpt.y << endl;
		cordinate cor;
		int dis = GETPOINTDIS2(cpt, cor, 3);
		cout << "dis = " << dis << "\t" << cor.i << "\t" << cor.j << endl;
		cordinate w;
		TRANS(cor, w);
		cout << "world = " << w.i << "\t" << w.j << endl;
		g_map.Add(w.j, w.i, cvScalar(0, 255, 255));
		map_d.push_back(GPoint(w.i, w.j, 5));
		d_tX0 = (d_tX0 * amount_t0 + w.i) / (amount_t0 + 1);
		d_tZ0 = (d_tZ0 * amount_t0 + w.j) / (amount_t0 + 1);
		amount_t0++;
		cout << "TRASHRED = " << d_tX0 << "\t" << d_tZ0 << endl;
		return true;
	}
	return false;
}

bool GridController::DealDetectTrashRed()
{
	if(amount_t1 > 10)
	{
		return false;
	}
	CvPoint cpt;
	if(DetectTrashRed(cpt, 3))
	{
		cout << "cpt = " << cpt.x << "\t" << cpt.y << endl;
		cordinate cor;
		int dis = GETPOINTDIS2(cpt, cor, 3);
		cout << "dis = " << dis << "\t" << cor.i << "\t" << cor.j << endl;
		cordinate w;
		TRANS(cor, w);
		cout << "world = " << w.i << "\t" << w.j << endl;
		g_map.Add(w.j, w.i, cvScalar(0, 255, 255));
		map_d.push_back(GPoint(w.i, w.j, 5));
		d_tX1 = (d_tX1 * amount_t1 + w.i) / (amount_t1 + 1);
		d_tZ1 = (d_tZ1 * amount_t1 + w.j) / (amount_t1 + 1);
		amount_t1++;
		cout << "TRASHRED = " << d_tX1 << "\t" << d_tZ1 << endl;
		return true;
	}
	return false;
}

bool GridController::DealDetectCococup()
{
	CvPoint cpt;
	if(DetectCococup(cpt, 1))
	{
		cout << "COCOCUP DETECTED =====" << endl;
		cout << "cpt = " << cpt.x << "\t" << cpt.y << endl;
		cordinate cor;
		int dis = GETPOINTDIS2(cpt, cor, 1);
		cout << "dis = " << dis << "\t" << cor.i << "\t" << cor.j << endl;
		cordinate w;
		TRANS(cor, w);
		cout << "world = " << w.i << "\t" << w.j << endl;
		g_map.Add(w.j, w.i, cvScalar(255, 0, 255));
		map_d.push_back(GPoint(w.i, w.j, 4));
		d_cX = (d_cX * amount_c + w.i) / (amount_c + 1);
		d_cZ = (d_cZ * amount_c + w.j) / (amount_c + 1);
		amount_c++;
		cout << "COCOCUP = " << d_cX << "\t" << d_cZ << endl;
		return true;
	}
	return false;
}

void GridController::THROWTRASH()
{
	CParts *parts = m_robot->getParts("LARM_LINK7");
	
	parts->releaseObj();
	sleep(3.0);
	
	b_grasp = true;

}

void GridController::INITARM()
{	
	double j_larm_joint4 = m_robot->getJointAngle("LARM_JOINT4");
	double j_rarm_joint4 = m_robot->getJointAngle("RARM_JOINT4");
	double v_j_larm_joint4 = DEG2RAD(-135) - j_larm_joint4;
	double v_j_rarm_joint4 = DEG2RAD(-135) - j_rarm_joint4;
	m_robot->setJointVelocity("LARM_JOINT4", v_j_larm_joint4, v_j_larm_joint4);
	m_robot->setJointVelocity("RARM_JOINT4", v_j_rarm_joint4, v_j_rarm_joint4);
	sleep(1.0);
	m_robot->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
	m_robot->setJointVelocity("RARM_JOINT4", 0.0, 0.0);
}
void GridController::PREPARETHROW(int deg)
{
	double j_larm_joint0 = m_robot->getJointAngle("LARM_JOINT0");
	double j_larm_joint1 = m_robot->getJointAngle("LARM_JOINT1");
	double j_larm_joint4 = m_robot->getJointAngle("LARM_JOINT4");
	
	double v_j_larm_joint0 = DEG2RAD(-20) - j_larm_joint0;
	double v_j_larm_joint1 = DEG2RAD(-90) - j_larm_joint1;
	double v_j_larm_joint4 = DEG2RAD(deg) - j_larm_joint4;
	
	m_robot->setJointVelocity("LARM_JOINT0", v_j_larm_joint0, v_j_larm_joint0);
	m_robot->setJointVelocity("LARM_JOINT1", v_j_larm_joint1, v_j_larm_joint1);
	m_robot->setJointVelocity("LARM_JOINT4", v_j_larm_joint4, v_j_larm_joint4);
	sleep(1.0);
	m_robot->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
	m_robot->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
	m_robot->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
}
void GridController::HOLDHAND(int deg)
{
	double j_larm_joint6 = m_robot->getJointAngle("LARM_JOINT6");
	double v_j_larm_joint6 = DEG2RAD(deg) - j_larm_joint6;
	sleep(1.0);
	m_robot->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
}
void GridController::HOLDARM(int deg)
{
	double j_larm_joint0 = m_robot->getJointAngle("LARM_JOINT0");
	double j_larm_joint1 = m_robot->getJointAngle("LARM_JOINT1");
	double j_larm_joint4 = m_robot->getJointAngle("LARM_JOINT4");
	double j_larm_joint6 = m_robot->getJointAngle("LARM_JOINT6");
	
	double v_j_larm_joint0 = DEG2RAD(-20) - j_larm_joint0;
	double v_j_larm_joint1 = DEG2RAD(-85) - j_larm_joint1;
	double v_j_larm_joint4 = DEG2RAD(deg) - j_larm_joint4;
	double v_j_larm_joint6 = DEG2RAD(90 - deg) - j_larm_joint6;
	
	m_robot->setJointVelocity("LARM_JOINT0", v_j_larm_joint0, v_j_larm_joint0);
	m_robot->setJointVelocity("LARM_JOINT1", v_j_larm_joint1, v_j_larm_joint1);
	m_robot->setJointVelocity("LARM_JOINT4", v_j_larm_joint4, v_j_larm_joint4);
	m_robot->setJointVelocity("LARM_JOINT6", v_j_larm_joint6, v_j_larm_joint6);
	sleep(1.0);
	m_robot->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
	m_robot->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
	m_robot->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
	m_robot->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
}

void GridController::MOVEHAND()
{
	static int sign = 1;
	static int num = 2;
	m_robot->setJointVelocity("LARM_JOINT0", DEG2RAD(sign * num), DEG2RAD(sign * num));
	sleep(1.0);
	m_robot->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
	num+=2;
	sign = -sign;
}

void GridController::HOLDBACKARM()
{
	double j_larm_joint6 = m_robot->getJointAngle("LARM_JOINT6");
	double j_larm_joint1 = m_robot->getJointAngle("LARM_JOINT1");
	double j_larm_joint4 = m_robot->getJointAngle("LARM_JOINT4");
	
	double v_j_larm_joint6 = DEG2RAD(0) - j_larm_joint6;
	double v_j_larm_joint1 = DEG2RAD(-45) - j_larm_joint1;
	double v_j_larm_joint4 = DEG2RAD(-135) - j_larm_joint4;
	m_robot->setJointVelocity("LARM_JOINT1", v_j_larm_joint1, v_j_larm_joint1);
	m_robot->setJointVelocity("LARM_JOINT4", v_j_larm_joint4, v_j_larm_joint4);
	m_robot->setJointVelocity("LARM_JOINT6", v_j_larm_joint6, v_j_larm_joint6);
	sleep(1.0);
	m_robot->setJointVelocity("LARM_JOINT1", 0.0, 0.0);
	m_robot->setJointVelocity("LARM_JOINT6", 0.0, 0.0);
	m_robot->setJointVelocity("LARM_JOINT4", 0.0, 0.0);
}

void GridController::TOUCH()
{
	m_robot->setJointVelocity("LARM_JOINT0", 0.5, 0.5);
}

void GridController::STOPTOUCH()
{
	m_robot->setJointVelocity("LARM_JOINT0", 0.0, 0.0);
}

bool GridController::MOVENEARBY(int x, int z, int allow)
{
	if(pow(f_posX - x, 2) + pow(f_posZ - z, 2) > allow)
	{
		int dx, dz;
		GETNEARBY(dx, dz);

		d_posX = x + dx;
		d_posZ = z + dz;
		return false;
	}
	return true;
}

void GridController::GETNEARBY(int& dx, int& dz)
{
	static int index = 0;
	switch(index)
	{
	case 0:
		dx = 0;
		dz = 1;
		break;
	case 1:
		dx = 0;
		dz = -1;
		break;
	case 2:
		dz = 0;
		dx = 1;
		break;
	case 3:
		dz = 0;
		dx = -1;
		break;
	case 4:
		dx = 0;
		dz = 2;
		break;
	case 5:
		dx = 0;
		dz = -2;
		break;
	case 6:
		dz = 0;
		dx = 2;
		break;
	case 7:
		dz = 0;
		dx = -2;
		break;
	case 8:
		dx = 0;
		dz = 3;
		break;
	case 9: 
		dx = 0;
		dz = -3;
		break;
	case 10:
		dz = 0;
		dx = 3;
		break;
	case 11:
		dz = 0;
		dx = -3;
		break;
	case 12:
		index = -1;
	}
	index++;
}

void GridController::TRANS(cordinate orgin, cordinate& world)
{
	orgin.i = -orgin.i;
	int zd = orgin.i * 0 + orgin.j * 1;
	int xd = orgin.i * 1 + orgin.j * 0;

	int dx = f_dirX * zd + f_dirZ * xd;
	int dz = f_dirZ * zd + -f_dirX * xd;

	world.i = dx + f_posX;
	world.j = dz + f_posZ;
}

int GridController::GETPOINTDIS(CvPoint cpt, cordinate& cor, int camID)
{
	ViewImage *dis_img = m_view->distanceSensor1D(0.0, 1023.0, camID);  
        char *dis_buf = dis_img->getBuffer();
	int distance = int((unsigned char)dis_buf[cpt.x]) * 4;

	double fovy = m_robot->getCamFOV() * PI / 180.0; 
        double fovx = 2 * atan(tan(fovy * 0.5) * m_robot->getCamAS());
	int width = 320;
	double theta = fovx * cpt.x / (width - 1.0) - fovx / 2.0;

	double x = distance * sin(theta);
	double z = distance * cos(theta) + 10;

	cor.i = floor(x) / SCALE;
	cor.j = floor(z) / SCALE;

	return distance;
}
int GridController::GETPOINTDIS2(CvPoint cpt, cordinate& cor, int camID)
{
	ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1023.0, camID);  
        char *dis_buf = dis_img->getBuffer();
        int width = dis_img->getWidth();   
	int height = dis_img->getHeight();  
	 
	int distance = int((unsigned char)dis_buf[cpt.y * width + cpt.x]) * 4;

	double fovy = m_robot->getCamFOV() * PI / 180.0; 
        double fovx = 2 * atan(tan(fovy * 0.5) * m_robot->getCamAS());
        
        double phi   = fovx * cpt.x / (width - 1.0) - fovx/2.0;   
        double theta = fovy * cpt.y / (height - 1.0) - fovy/2.0;
        
	double z = distance / sqrt(1 + pow(tan(phi), 2) + pow(tan(theta), 2));
	double x = z * tan(phi);

	cor.i = floor(x) / SCALE;
	cor.j = floor(z) / SCALE;

	return distance;
}
 /*
bool GridController::DetectCococup(CvPoint& cpt, int camID)
{
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *red = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(red->imageData, buf, red->imageSize);

	ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1023.0, camID, DEPTHBIT_8, IMAGE_320X240);  
        char *dis_buf = dis_img->getBuffer();

	IplImage *src = cvCreateImage(cvSize(320, 240), 8, 1);
	memcpy(src->imageData, dis_buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);


	unsigned char* rimagedata = (unsigned char*)red->imageData;
	int rstep = red->widthStep / sizeof(unsigned char);

	long average = 0;
	int i;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			average += imagedata[i * step + j];
		}
	}
	average /= (src->height * src->width);
	cout << average << endl;

	cvThreshold(src, src, average, 255, CV_THRESH_BINARY);
	
	int mi = 0;
	int mj = 0;
	int amount = 0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			if(imagedata[i * step + j] == 0)
			{
				unsigned char B = rimagedata[i * rstep + j * 3];
				unsigned char G = rimagedata[i * rstep + j * 3 + 1];
				unsigned char R = rimagedata[i * rstep + j * 3 + 2];
				if(R > 80 && R / G >= 2 && R / B >= 4)
				{
					mi += i;
					mj += j;
					amount++;
				}
			}
		}
	}
	if(amount > 0)
	{
		mi /= amount;
		mj /= amount;
		cvCircle(red, cvPoint(mj, mi), 10, cvScalar(255, 255, 255), 3);
		cpt.x = mj;
		cpt.y = mi;

		cvNamedWindow("CO");
		cvShowImage("CO", red);
		cvWaitKey(1000);
		cvDestroyWindow("CO");
		cvReleaseImage(&src);
		cvReleaseImage(&red);
		return true;
		
	}
	return false;
}*/

bool GridController::DetectCococup(CvPoint& cpt, int camID)
{
	cout << "In DetectCococup" << endl;
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *red = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(red->imageData, buf, red->imageSize);

	ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1023.0, camID, DEPTHBIT_8, IMAGE_320X240);  
        char *dis_buf = dis_img->getBuffer();

	IplImage *src = cvCreateImage(cvSize(320, 240), 8, 1);
	memcpy(src->imageData, dis_buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);


	unsigned char* rimagedata = (unsigned char*)red->imageData;
	int rstep = red->widthStep / sizeof(unsigned char);

	long average = 0;
	int i;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			average += imagedata[i * step + j];
		}
	}
	average /= (src->height * src->width);
	cout << average << endl;

	cvThreshold(src, src, average, 255, CV_THRESH_BINARY);
	
	int mi = 0;
	int mj = 0;
	int amount = 0;
	int else_count=0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			if(imagedata[i * step + j] == 0)
			{
				unsigned char B = rimagedata[i * rstep + j * 3];
				unsigned char G = rimagedata[i * rstep + j * 3 + 1];
				unsigned char R = rimagedata[i * rstep + j * 3 + 2];
				if(R > 80 && R / G >= 2 && R / B >= 4)
				{
					mi += i;
					mj += j;
					amount++;
				}
				else
				{
				        else_count++;
				}
			}
		}
	}
	cout << "amount = " << amount << endl;
	if(amount > 0)
	{
	        
		mi /= amount;
		mj /= amount;
		cvCircle(red, cvPoint(mj, mi), 10, cvScalar(255, 255, 255), 3);
		cpt.x = mj;
		cpt.y = mi;
                
                //
                CvPoint pt;
		pt.x=cpt.x+(-cpt.x)/2;
		pt.y=cpt.y+(-cpt.y)/2;
		int dis=(unsigned char)dis_buf[cpt.y*step+cpt.x];
		int else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		if(fabs(double(dis-else_dis))/dis>0.1)
		{
			cout << "Not plant" << endl;
		        if(double(else_count)/amount>DIS)
                        {
                                std::cout<<"amount= "<<amount<<std::endl;
                                std::cout<<"else_mount= "<<else_count<<std::endl;
                                std::cout<<"distance of cup= "<<dis<<std::endl;
                                
		                /*cvNamedWindow("CO");
		                cvShowImage("CO", red);
		                cvWaitKey(1000);
		                cvDestroyWindow("CO");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*//////
		                return true;
		        }
		}
		
	}
	return false;
}
/*
bool GridController::DetectTrashRed(CvPoint& cpt, int camID)
{
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(src->imageData, buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);

	IplImage *red = cvCreateImage(cvGetSize(src), 8, 1);
	unsigned char* rimagedata = (unsigned char*)red->imageData;
	int rstep = red->widthStep / sizeof(unsigned char);

	int i;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char B = imagedata[i * step + j * 3];
			unsigned char G = imagedata[i * step + j * 3 + 1];
			unsigned char R = imagedata[i * step + j * 3 + 2];

			if(B < 120 && G < 100 && R > 140 && fabs(B - G) < 10)
			{
				rimagedata[i * rstep + j] = 255;
			}
			else
			{
				rimagedata[i * rstep + j] = 0;
			}
		}
	}
	cvDilate(red, red, NULL, 3);

	int amount = 0;
	int mi = 0;
	int mj = 0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char r = rimagedata[i * rstep + j];
			if(r == 255)
			{
				unsigned char B = imagedata[i * step + j * 3];
				unsigned char G = imagedata[i * step + j * 3 + 1];
				unsigned char R = imagedata[i * step + j * 3 + 2];
				if(B > 0 && B < 150 && G > 165 && G < 190 && R > 165 && R < 190 && fabs(R - G) < 10)
				{
					mi += i;
					mj += j;
					amount++;
				}
			}
		}
	}
	if(amount != 0)
	{
		mi /= amount;
		mj /= amount;
		cvCircle(src, cvPoint(mj, mi), 10, cvScalar(255, 255, 255), 3);
		cpt.x = mj;
		cpt.y = mi;

		cvNamedWindow("EX");
		cvShowImage("EX", src);
		cvWaitKey(1000);
		cvDestroyWindow("EX");
		cvReleaseImage(&src);
		cvReleaseImage(&red);
		return true;
	}
	return false;
}*/
bool GridController::DetectTrashBlue(CvPoint& cpt, int camID)
{
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(src->imageData, buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);

	IplImage *red = cvCreateImage(cvGetSize(src), 8, 1);
	unsigned char* rimagedata = (unsigned char*)red->imageData;
	int rstep = red->widthStep / sizeof(unsigned char);

        ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1023.0, camID, DEPTHBIT_8, IMAGE_320X240);  
        char *dis_buf = dis_img->getBuffer();
        
	int i;
	int trashBox_count=0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char B = imagedata[i * step + j * 3];
			unsigned char G = imagedata[i * step + j * 3 + 1];
			unsigned char R = imagedata[i * step + j * 3 + 2];

			if(B > 100 && G < 150 && G > 50 && R < 50 && fabs(double(B - G)) < 50)
			{
				rimagedata[i * rstep + j] = 255;
				trashBox_count++;
			}
			else
			{
				rimagedata[i * rstep + j] = 0;
			}
		}
	}
	cvDilate(red, red, NULL, 3);

	int amount = 0;
	int mi = 0;
	int mj = 0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char r = rimagedata[i * rstep + j];
			if(r == 255)
			{
				unsigned char B = imagedata[i * step + j * 3];
				unsigned char G = imagedata[i * step + j * 3 + 1];
				unsigned char R = imagedata[i * step + j * 3 + 2];
				if(B > 0 && B < 150 && G > 165 && G < 190 && R > 165 && R < 190 && fabs(R - G) < 10)
				{
					mi += i;
					mj += j;
					amount++;
				}
			}
		}
	}
	
	if(amount != 0)
	{
	        
		mi /= amount;
		mj /= amount;
		CvPoint pt;
		pt.x=mj/2;
		pt.y=mi/2;
		double dis=(int)(unsigned char)dis_buf[mi*step+mj];
		double else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		if(fabs(dis-else_dis)>10 || dis < 60)
		{
		        std::cout<<"trashBox_count"<<trashBox_count<<std::endl;
		        std::cout<<"dis"<<dis<<std::endl;
		        std::cout<<"trashBox_count/dis"<<trashBox_count/dis<<std::endl;
	                if(trashBox_count>50)
	                {
		                cvCircle(src, cvPoint(mj, mi), 10, cvScalar(255, 255, 255), 3);
		                cpt.x = mj;
		                cpt.y = mi;

		                /*cvNamedWindow("EX");
		                cvShowImage("EX", src);
		                cvWaitKey(1000);
		                cvDestroyWindow("EX");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*/////
		                return true;
		        }
		}
	}
	return false;
}
bool GridController::DetectTrashGreen(CvPoint& cpt, int camID)
{
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(src->imageData, buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);

	IplImage *red = cvCreateImage(cvGetSize(src), 8, 1);
	unsigned char* rimagedata = (unsigned char*)red->imageData;
	int rstep = red->widthStep / sizeof(unsigned char);

        ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1023.0, camID, DEPTHBIT_8, IMAGE_320X240);  
        char *dis_buf = dis_img->getBuffer();
        
	int i;
	int trashBox_count=0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char B = imagedata[i * step + j * 3];
			unsigned char G = imagedata[i * step + j * 3 + 1];
			unsigned char R = imagedata[i * step + j * 3 + 2];

			if((R > 70 && R < 90 && G > 120 && G < 140 && B > 70 && B < 90))// || (R < 10 && G < 10 && B < 10))
			{
				rimagedata[i * rstep + j] = 255;
				trashBox_count++;
			}
			else
			{
				rimagedata[i * rstep + j] = 0;
			}
		}
	}
	cvDilate(red, red, NULL, 3);

	int amount = 0;
	int mi = 0;
	int mj = 0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char r = rimagedata[i * rstep + j];
			if(r == 255)
			{
				unsigned char B = imagedata[i * step + j * 3];
				unsigned char G = imagedata[i * step + j * 3 + 1];
				unsigned char R = imagedata[i * step + j * 3 + 2];
				if(B > 0 && B < 150 && G > 165 && G < 190 && R > 165 && R < 190 && fabs(R - G) < 10)
				{
					mi += i;
					mj += j;
					amount++;
				}
			}
		}
	}
	
	if(amount != 0)
	{
	        
		mi /= amount;
		mj /= amount;
		CvPoint pt;
		pt.x=mj/2;
		pt.y=mi/2;
		double dis=(int)(unsigned char)dis_buf[mi*step+mj];
		double else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		if(fabs(dis-else_dis)>10 || dis < 60)
		{
		        std::cout<<"trashBox_count"<<trashBox_count<<std::endl;
		        std::cout<<"dis"<<dis<<std::endl;
		        std::cout<<"trashBox_count/dis"<<trashBox_count/dis<<std::endl;
	                if(trashBox_count>50)
	                {
		                //cvCircle(src, cvPoint(mj, mi), 10, cvScalar(255, 255, 255), 3);
		                cpt.x = mj;
		                cpt.y = mi;

		                /*cvNamedWindow("EX");
		                cvShowImage("EX", src);
		                cvWaitKey(1000);
		                cvDestroyWindow("EX");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*//////
		                return true;
		        }
		}
	}
	return false;
}

bool GridController::DetectTrashRed(CvPoint& cpt, int camID)
{
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(src->imageData, buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);

	IplImage *red = cvCreateImage(cvGetSize(src), 8, 1);
	unsigned char* rimagedata = (unsigned char*)red->imageData;
	int rstep = red->widthStep / sizeof(unsigned char);

        ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1023.0, camID, DEPTHBIT_8, IMAGE_320X240);  
        char *dis_buf = dis_img->getBuffer();
        
	int i;
	int trashBox_count=0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char B = imagedata[i * step + j * 3];
			unsigned char G = imagedata[i * step + j * 3 + 1];
			unsigned char R = imagedata[i * step + j * 3 + 2];

			if(B < 120 && G < 100 && R > 140 && fabs(B - G) < 10)
			{
				rimagedata[i * rstep + j] = 255;
				trashBox_count++;
			}
			else
			{
				rimagedata[i * rstep + j] = 0;
			}
		}
	}
	cvDilate(red, red, NULL, 3);

	int amount = 0;
	int mi = 0;
	int mj = 0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char r = rimagedata[i * rstep + j];
			if(r == 255)
			{
				unsigned char B = imagedata[i * step + j * 3];
				unsigned char G = imagedata[i * step + j * 3 + 1];
				unsigned char R = imagedata[i * step + j * 3 + 2];
				if(B > 0 && B < 150 && G > 165 && G < 190 && R > 165 && R < 190 && fabs(R - G) < 10)
				{
					mi += i;
					mj += j;
					amount++;
				}
			}
		}
	}
	
	if(amount != 0)
	{
	        
		mi /= amount;
		mj /= amount;
		CvPoint pt;
		pt.x=mj/2;
		pt.y=mi/2;
		double dis=(int)(unsigned char)dis_buf[mi*step+mj];
		double else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		if(fabs(dis-else_dis)>10 || dis < 60)
		{
		        std::cout<<"trashBox_count"<<trashBox_count<<std::endl;
		        std::cout<<"dis"<<dis<<std::endl;
		        std::cout<<"trashBox_count/dis"<<trashBox_count/dis<<std::endl;
	                if(trashBox_count>50)
	                {
		                //cvCircle(src, cvPoint(mj, mi), 10, cvScalar(255, 255, 255), 3);
		                cpt.x = mj;
		                cpt.y = mi;

		                /*cvNamedWindow("EX");
		                cvShowImage("EX", src);
		                cvWaitKey(1000);
		                cvDestroyWindow("EX");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*/////
		                return true;
		        }
		}
	}
	return false;
}
bool GridController::DetectWagon(CvPoint& cpt, int camID)
{
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *src = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(src->imageData, buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);

	int i;
	int mi = 0;
	int mj = 0;
	int amount = 0;
	for(i = 0; i < src->height / 2; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			unsigned char B = imagedata[i * step + j * 3];
			unsigned char G = imagedata[i * step + j * 3 + 1];
			unsigned char R = imagedata[i * step + j * 3 + 2];

			if(B > 30 && B < 60 && G > 50 && G < 80 && R > 80 && R < 130)
			{
				//imagedata[i * step + j * 3] = 255;
				//imagedata[i * step + j * 3 + 1] = 255;
				//imagedata[i * step + j * 3 + 2] = 255;
				mi += i;
				mj += j;
				amount++;
			}
		}
	}
	int ni = 0;
	int nj = 0;
	int n_amount = 0;
	if(amount > 50)
	{
		mi /= amount;
		mj /= amount;
		for(i = mi; i < src->height / 2; i++)
		{
			for(int j = 0; j < src->width; j++)
			{
				unsigned char B = imagedata[i * step + j * 3];
				unsigned char G = imagedata[i * step + j * 3 + 1];
				unsigned char R = imagedata[i * step + j * 3 + 2];

				if(B > 30 && B < 60 && G > 50 && G < 80 && R > 80 && R < 130)
				{
					imagedata[i * step + j * 3] = 255;
					imagedata[i * step + j * 3 + 1] = 255;
					imagedata[i * step + j * 3 + 2] = 255;
					ni += i;
					nj += j;
					n_amount++;
				}
			}
		}
		if(n_amount > 0)
		{
			ni /= n_amount;
			nj /= n_amount;
		
			//cvCircle(src, cvPoint(nj, ni), 10, cvScalar(255, 255, 255), 3);
			cpt.x = nj;
			cpt.y = ni;
			/*cvNamedWindow("EX");
			cvShowImage("EX", src);
			cvWaitKey(1000);
			cvDestroyWindow("EX");
			cvReleaseImage(&src);*//////
			return true;
		}
	}
	return false;
}
bool GridController::MOVEPATH(int X, int Z)
{
	if(f_posX != X || f_posZ != Z)
	{
		vector<cordinate> res = GetPath(X, Z);
		if(res.size() == 1 && res[0].i == 0 && res[0].j == 0)
		{
			cout << "No Result" << endl;
			d_posX = f_posX;
			d_posZ = f_posZ;
			return true;
		}
		int rX = res[res.size() - 2].j;
		int rZ = res[res.size() - 2].i;

		cout << rX << "\t" << rZ << endl;
		MOVETO(rX, rZ);
		return false;
	}
	return true;
}

void GridController::MOVEAROUND()
{
	srand(time(NULL));
	int amount = map_d.size();
	int cs = rand() % amount;
	if(map_d[cs].f_state == 2)
	{
		d_posX = map_d[cs].x;
		d_posZ = map_d[cs].z;
	}
}

void GridController::MOVETO(int pX, int pZ)
{
	int dx = pX - f_posX;
	int dz = pZ - f_posZ;
	cout << "dx = " << dx << "\t" << dz << endl;	
	cout << "dir = " << f_dirX << "\t" << f_dirZ << endl;
	cout << "pos = " << f_posX << "\t" << f_posZ << endl;


	if(dx != 0)
	{
		while(dx != f_dirX)
		{
			TURNRIGHT();
			if(m_state == 10 || m_state == 65)
				return;
			
		}
		MOVEFORWARD();
		if(m_state == 10 || m_state == 65)
			return;
		
	}
	if(dz != 0)
	{
		while(dz != f_dirZ)
		{
			TURNLEFT();
			if(m_state == 10 || m_state == 65)
				return;
		}
		MOVEFORWARD();
	}
}
 
vector<cordinate> GridController::GetPath(int pX, int pZ)
{
	int h = 600 / GSCALE;
	int w = 800 / GSCALE;
	Dijkstra djk(h, w);
	int** predata = new int*[h];
	int i;
	for(i = 0; i < h; i++)
	{
		predata[i] = new int[w];
		for(int j = 0; j < w; j++)
		{
			predata[i][j] = 0;
		}
	}
	for(i = 0; i < map_d.size(); i++)
	{
		if(map_d[i].f_state == 3)
		{
			int y = map_d[i].z + h / 2;
			int x = map_d[i].x + w / 2;

			predata[y][x] = 1;
		}
	}
	djk.SetData(predata);
	djk.SetStart(f_posZ + h / 2, f_posX + w / 2);
	djk.SetDestination(pZ + h / 2, pX + w / 2);
	vector<cordinate> result;
	if(!djk.Calculate())
	{
		result.push_back(cordinate(0, 0));
		return result;
	}
		
	vector<cordinate> cpath = djk.GetPath();
	
	for(int j = 0; j < cpath.size(); j++)
	{
		//cout << "-- " << cpath[j].i - h / 2 << "\t" << cpath[j].j  - w / 2 << endl;
		result.push_back(cordinate(cpath[j].i - h / 2, cpath[j].j  - w / 2));
	}
	return result;
}

void GridController::Detect()
{
	int fd = FindObs(3);
	//cout << "fd = " << fd << endl;
	if(fd > 0)
	{
		for(int i = 1; i < fd; i++)
		{
			int nX = f_posX + i * f_dirX;
			int nZ = f_posZ + i * f_dirZ;
			int amount = 0;
			for(int j = 0; j < map_d.size(); j++)
			{
				if(nX == map_d[j].x && nZ == map_d[j].z)
				{
					if(map_d[j].f_state == 0)
					{
						map_d[j].f_state = 2;
						g_map.Add(nZ, nX, cvScalar(0, 255, 0));
					}
					break;
				}
				else
				{
					amount++;
				}
			}
			//cout << "amount = " << amount << endl;
			if(amount == map_d.size())
			{
				map_d.push_back(GPoint(nX, nZ, 2));
				g_map.Add(nZ, nX, cvScalar(0, 255, 0));
			}
		}
	}
	if(fd < 200 / SCALE)
	{
		int nX = f_posX + fd * f_dirX;
		int nZ = f_posZ + fd * f_dirZ;
		int amount = 0;
		for(int j = 0; j < map_d.size(); j++)
		{
			if(nX == map_d[j].x && nZ == map_d[j].z)
			{
				if(map_d[j].f_state == 0 || map_d[j].f_state == 2)
				{
					map_d[j].f_state = 3;
					g_map.Add(nZ, nX, cvScalar(255, 0, 0));
				}
				break;
			}
			else
			{
				amount++;
			}
		}
		//cout << "amount = " << amount << endl;
		if(amount == map_d.size())
		{
			map_d.push_back(GPoint(nX, nZ, 3));
			g_map.Add(nZ, nX, cvScalar(255, 0, 0));
		}
	}
	fd = FindObs(2);
	if(fd > 0)
	{
		for(int i = 1; i < fd; i++)
		{
			int fdx = f_dirZ;
			int fdz = -f_dirX;

			int nX = f_posX + i * fdx;
			int nZ = f_posZ + i * fdz;

			int amount = 0;
			for(int j = 0; j < map_d.size(); j++)
			{
				if(nX == map_d[j].x && nZ == map_d[j].z)
				{
					if(map_d[j].f_state == 0)
					{
						map_d[j].f_state = 2;
						g_map.Add(nZ, nX, cvScalar(0, 255, 0));
					}
					break;
				}
				else
				{
					amount++;
				}
				if(amount == map_d.size())
				{
					map_d.push_back(GPoint(nX, nZ, 2));
					g_map.Add(nZ, nX, cvScalar(0, 255, 0));
				}
			}
		}
	}
	if(fd < 200 / SCALE)
	{
		int nX = f_posX + fd * f_dirZ;
		int nZ = f_posZ - fd * f_dirX;
		int amount = 0;
		for(int j = 0; j < map_d.size(); j++)
		{
			if(nX == map_d[j].x && nZ == map_d[j].z)
			{
				if(map_d[j].f_state == 0 || map_d[j].f_state == 2)
				{
					map_d[j].f_state = 3;
					g_map.Add(nZ, nX, cvScalar(255, 0, 0));
				}
				break;
			}
			else
			{
				amount++;
			}
		}
		//cout << "amount = " << amount << endl;
		if(amount == map_d.size())
		{
			map_d.push_back(GPoint(nX, nZ, 3));
			g_map.Add(nZ, nX, cvScalar(255, 0, 0));
		}
	}
	fd = FindObs(4);
	if(fd > 0)
	{
		for(int i = 1; i < fd; i++)
		{
			int fdx = -f_dirZ;
			int fdz = f_dirX;

			int nX = f_posX + i * fdx;
			int nZ = f_posZ + i * fdz;

			int amount = 0;
			for(int j = 0; j < map_d.size(); j++)
			{
				if(nX == map_d[j].x && nZ == map_d[j].z)
				{
					if(map_d[j].f_state == 0)
					{
						map_d[j].f_state = 2;
						g_map.Add(nZ, nX, cvScalar(0, 255, 0));
					}
					break;
				}
				else
				{
					amount++;
				}
				if(amount == map_d.size())
				{
					map_d.push_back(GPoint(nX, nZ, 2));
					g_map.Add(nZ, nX, cvScalar(0, 255, 0));
				}
			}
		}
	}
	if(fd < 200 / SCALE)
	{
		int nX = f_posX - fd * f_dirZ;
		int nZ = f_posZ + fd * f_dirX;
		int amount = 0;
		for(int j = 0; j < map_d.size(); j++)
		{
			if(nX == map_d[j].x && nZ == map_d[j].z)
			{
				if(map_d[j].f_state == 0 || map_d[j].f_state == 2)
				{
					map_d[j].f_state = 3;
					g_map.Add(nZ, nX, cvScalar(255, 0, 0));
				}
				break;
			}
			else
			{
				amount++;
			}
		}
		//cout << "amount = " << amount << endl;
		if(amount == map_d.size())
		{
			map_d.push_back(GPoint(nX, nZ, 3));
			g_map.Add(nZ, nX, cvScalar(255, 0, 0));
		}
	}
}

int GridController::FindObs(int camID)
{
	int* data = GETHLINE(camID);
	if(data == NULL)
	{
		//cout << "data = NULL" << endl;
		return -1;
	}
	Vector3d* pv3d = GETHPOINTA(data);//*///
	//Vector3d* pv3d = GETPLANT(camID);
	
	if(pv3d == NULL)
	{
		return -2;
	}
	int height = 240;
	int width = 320;
	int min_dis = 255;
	for(int i = 0; i < width; i++)
	{
		if(fabs(pv3d[i].x()) < 25)// && pv3d[i].y() > 0)
		{
			if(min_dis > pv3d[i].z())
			{
				min_dis = pv3d[i].z();
			}
		}
	}
	/*for(int i = 0; i < height; i++)
	{
		for(int j = 0; j < width; j++)
		{
			if(fabs(pv3d[i * width + j].x()) < 25 && pv3d[i * width + j].y() > 20)
			{
				if(min_dis > pv3d[i].z())
				{
					min_dis = pv3d[i].z();
				}
			}
		}
	}*/
	//cout << "min_dis " << camID << " = " << min_dis << endl;
	return min_dis / SCALE;
}

int* GridController::GETHLINE(int camID)
{
	if(m_view != NULL) 
	{ 
		//cout << "In" << endl;
        	ViewImage *dis_img = m_view->distanceSensor1D(0.0, 255.0, camID, DEPTHBIT_8, IMAGE_320X1);
                char *dis_buf = dis_img->getBuffer(); 
                int width = dis_img->getWidth();
		//cout << "width = " << width << endl;
		int* data = new int[width];
		for(int i = 0; i < width; i++)
		{
			data[i] = int((unsigned char)dis_buf[i]);
		}
		//cout << "data[0] = " << data[0] << endl;
		return data;
	}
	//cout << "Out" << endl;
	return NULL;
}

Vector3d* GridController::GETHPOINTA(int* data)
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
			result[i] = Vector3d(data[i] * sin(theta), 42, 10 + data[i] * cos(theta));
		}
		return result;
	}
	return NULL;
}

Vector3d* GridController::GETPLANT(int camID) 
{
	if(m_view != NULL) 
	{ 
       		ViewImage *dis_img = m_view->distanceSensor2D(0.0, 255.0, camID, DEPTHBIT_8, IMAGE_320X240);   
 
  		double fovy = m_robot->getCamFOV() * PI / 180.0;
  		double ar = m_robot->getCamAS();  

                double fovx = 2 * atan(tan(fovy * 0.5) * ar);  

                char *dis_buf = dis_img->getBuffer();  
                  
		int width = dis_img->getWidth();
                int height = dis_img->getHeight();
	          
		Vector3d *p = new Vector3d[height * width];
                double *theta = new double[width]; 
		double *phi = new double[height];

	        unsigned char *distance = new unsigned char[height*width];

		double *y = new double[height * width];
		double *x = new double[height * width];
		double *z = new double[height * width];
		//ou << "====" << endl;
		int index;
		for(int i = 0; i < height; i++)
		{
			phi[i] = fovy / 2.0 - fovy * i / (height - 1.0);  
		    	for(int j = 0; j < width; j++)
			{
                        	theta[j] = fovx * j / (width - 1.0) - fovx / 2.0;  
		  		index = i * width + j;
		  		distance[index] = dis_buf[index]; 
				//ou << int(distance[index]) << "\t";
		    	}  
			//ou << endl;
		}
		int *dis = new int[height * width];
		int *disv = new int[height];
		int *dish = new int[width];
		double *angle = new double[height * width];

		
		for(int i = 0; i < height; i++)
		{
		  	disv[i] = distance[i * width + width / 2];
		  	//disv[i] = disv[i];
		  	for(int j = 0;j < width; j++)
			{
				index = i * width + j;
				dis[index]=distance[index];
				//dis[index]=4*dis[index];
		        	dish[j]=distance[width*height/2+j];
		        	//dish[j]=4*dish[j];
				y[index]=disv[i]*sin(phi[i])+30;
				x[index]=dish[j]*sin(theta[j]);
				angle[index]=asin((dish[0]*sin(fovx/2)*(width/2-j))/(dis[index]*width/2));
			
				z[index]=dis[index]*cos(angle[index])*cos(phi[i])+10;
		  	}
		}
		for(int i=0;i<height;i++)
		{
		  	for(int j=0;j<width;j++)
			{
				index=i*width+j;
				p[index]=Vector3d(x[index],y[index],z[index]);
		  	}
		}

		delete theta;
		delete phi;
		delete distance;
		delete dis;
		delete disv;
		delete dish;
		delete x;
		delete y;
		delete z;

		return p;
	}
	return NULL;
}  

void GridController::BackToGridMap()
{
	vector<int> cpy_prc_m = prc_m;
	vector<int> cpy_prc_d = prc_d;
			
	for(int i = cpy_prc_m.size() - 1; i >= 0 ; i--)
	{
		if(cpy_prc_m[i] == 2)
		{
			DIRECTIONCORRECTION(-cpy_prc_d[i]);
		}
		else if(cpy_prc_m[i] == -2)
		{
			DIRECTIONCORRECTION(-cpy_prc_d[i]);
		}
		else if(cpy_prc_m[i] == 1)
		{
			POSITIONCORRECTION(-cpy_prc_d[i]);
		}
		else if(cpy_prc_m[i] == -1)
		{
			POSITIONCORRECTION(-cpy_prc_d[i]);
		}
	}
	prc_m.clear();
	prc_d.clear();
}

bool GridController::preCorrection(int x, int dis)
{
	if(x < 80)
	{
		TURNLEFT();
		MOVEFORWARD();
		TURNRIGHT();
		return false;
	}
	else if(x > 240)
	{
		TURNRIGHT();
		MOVEFORWARD();
		TURNLEFT();
		return false;
	}
	else if(dis > 50)
	{
		MOVEFORWARD();
		return false;
	}
	else
	{
		return true;
	}
}

bool GridController::correction(int x, int dis)
{
	if(x < 150)
	{
		DIRECTIONCORRECTION((x - 150) / 10);
		return false;
	}
	else if(x > 170)
	{
		DIRECTIONCORRECTION((x - 170) / 10);
		return false;
	}
	else if(dis < 20)
	{
		POSITIONCORRECTION((dis - 20) / 3);
		return false;
	}
	else if(dis > 25)
	{
		POSITIONCORRECTION((dis - 25) / 3);
		return false;
	}
	return true;
}
void GridController::POSITIONCORRECTION(int pos)
{
	if(pos > 0)
	{
		m_robot->setWheelVelocity(0.1 * pos + 0.1, 0.1 * pos + 0.1);
		sleep(1.0);
		m_robot->setWheelVelocity(0.0, 0.0);
		prc_m.push_back(1);
		prc_d.push_back(pos);
	}
	else
	{
		m_robot->setWheelVelocity(0.1 * pos - 0.1, 0.1 * pos - 0.1);
		sleep(1.0);
		m_robot->setWheelVelocity(0.0, 0.0);
		prc_m.push_back(-1);
		prc_d.push_back(pos);
	}
}

void GridController::DIRECTIONCORRECTION(int dir)
{
	if(dir > 0)
	{
		m_robot->setWheelVelocity(DEG2RAD(dir + 1), -DEG2RAD(dir + 1));
		sleep(1.0);
		m_robot->setWheelVelocity(0.0, 0.0);
		prc_m.push_back(2);
		prc_d.push_back(dir);
	}
	else
	{
		m_robot->setWheelVelocity(DEG2RAD(dir - 1), -DEG2RAD(dir - 1));
		sleep(1.0);
		m_robot->setWheelVelocity(0.0, 0.0);
		prc_m.push_back(-2);
		prc_d.push_back(dir);
	}
}

void GridController::MOVEFORWARD()
{
	m_robot->setWheelVelocity(SCALE / 10.0, SCALE / 10.0);
	sleep(1.0);
	m_robot->setWheelVelocity(0.0, 0.0);
	
	f_posX = f_posX + f_dirX;
	f_posZ = f_posZ + f_dirZ;
	
	map_d.push_back(GPoint(f_posX, f_posZ, 1));
	g_map.Add(f_posZ, f_posX, cvScalar(0, 0, 255));
	Detect();
}

void GridController::MOVEBACKWARD()
{
	m_robot->setWheelVelocity(-SCALE / 10.0, -SCALE / 10.0);
	sleep(1.0);
	m_robot->setWheelVelocity(0.0, 0.0);
	
	f_posX = f_posX - f_dirX;
	f_posZ = f_posZ - f_dirZ;
	
	map_d.push_back(GPoint(f_posX, f_posZ, 1));
	g_map.Add(f_posZ, f_posX, cvScalar(0, 0, 255));
	Detect();
}

void GridController::TURNLEFT()
{
	m_robot->setWheelVelocity(-PI / 4.0, PI / 4.0);
	sleep(1.0);
	m_robot->setWheelVelocity(0.0, 0.0);

	int dz = f_dirZ;
	int dx = f_dirX;

	f_dirX = dz;
	f_dirZ = -dx;
	Detect();
}

void GridController::TURNRIGHT()
{
	m_robot->setWheelVelocity(PI / 4.0, -PI / 4.0);
	sleep(1.0);
	m_robot->setWheelVelocity(0.0, 0.0);
	
	int dz = f_dirZ;
	int dx = f_dirX;

	f_dirX = -dz;
	f_dirZ = dx;
	Detect();
}

void GridController::STOP()
{
	m_robot->setWheelVelocity(0.0, 0.0);
}

/*bool GridController::DetectCup(CvPoint& cpt,int camID)
{
	cout << "In DetectCup" << endl;
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *red = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(red->imageData, buf, red->imageSize);

	ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1023.0, camID, DEPTHBIT_8, IMAGE_320X240);  
        char *dis_buf = dis_img->getBuffer();

	IplImage *src = cvCreateImage(cvSize(320, 240), 8, 1);
	memcpy(src->imageData, dis_buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);


	unsigned char* rimagedata = (unsigned char*)red->imageData;
	int rstep = red->widthStep / sizeof(unsigned char);

	long average = 0;
	int i;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			average += imagedata[i * step + j];
		}
	}
	average /= (src->height * src->width);
	cout << average << endl;

	cvThreshold(src, src, average, 255, CV_THRESH_BINARY);
	
	int mi_red = 0;
	int mj_red = 0;
	int amount_red = 0;
	int else_count_red=0;
	int mi_green = 0;
	int mj_green = 0;
	int amount_green = 0;
	int else_count_green=0;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			if(imagedata[i * step + j] == 0)
			{
				unsigned char B = rimagedata[i * rstep + j * 3];
				unsigned char G = rimagedata[i * rstep + j * 3 + 1];
				unsigned char R = rimagedata[i * rstep + j * 3 + 2];
				//red
				if(R > 80 && R / G >= 2 && R / B >= 4)
				{
					mi_red += i;
					mj_red += j;
					amount_red++;
					else_count_green++;
				}
				else if(R < 180 && R > 130 && G > 200 && G < 240 && B < 180)//green
				{
				        mi_green += i;
					mj_green += j;
					amount_green++;  
					else_count_red++;      
				}
				else
				{
				        else_count_red++;
				        else_count_green++;
				}
			}
		}
	}
	cout << "amount_red = " << amount_red << endl;
	cout << "amount_green = " << amount_green << endl;
	
	if(amount_red > 0)
	{
	        
		mi_red /= amount_red;
		mj_red /= amount_red;
		cvCircle(red, cvPoint(mj_red, mi_red), 10, cvScalar(255, 255, 255), 3);
		cpt.x = mj_red;
		cpt.y = mi_red;
                
                CvPoint pt;
		pt.x=cpt.x/2;
		pt.y=cpt.y/2;
		int dis=(unsigned char)dis_buf[cpt.y*step+cpt.x];
		int else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		cout << " else_count_red= " << else_count_red << endl;
		
		if(fabs(double(dis-else_dis))/dis>0.1) 
		{
			cout << "Not plant" << endl;
		        if(double(else_count_red)/amount_red>DIS)
                        {
                                std::cout<<"distance of COCO= "<<dis<<std::endl;
                                
		                cvNamedWindow("CO");
		                cvShowImage("CO", red);
		                cvWaitKey(1000);
		                cvDestroyWindow("CO");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);
		                return true;
		        }
		}
		
	}
	if(amount_green > 0)
	{
	        
		mi_green /= amount_green;
		mj_green /= amount_green;
		cvCircle(red, cvPoint(mj_green, mi_green), 10, cvScalar(255, 255, 255), 3);
		cvCircle(red, cvPoint(mj_green/2, mi_green/2), 10, cvScalar(255, 255, 0), 3);
		cpt.x = mj_green;
		cpt.y = mi_green;
                
                CvPoint pt;
		pt.x=cpt.x/2;
		pt.y=cpt.y/2;
		int dis=(unsigned char)dis_buf[cpt.y*step+cpt.x];
		int else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		cout << " else_count_green= " << else_count_green << endl;
		cout << "dis = " << dis << "\t" << else_dis << endl; 
		if(fabs(double(dis-else_dis))/dis>0.1)
		{
			cout << "Not plant" << endl;
		        if(double(else_count_green)/amount_green>DIS)
                        {  
                                cout<<"distance of CAN= "<<dis<<endl;
                                
		                cvNamedWindow("CO");
		                cvShowImage("CO", red);
		                cvWaitKey(1000);
		                cvDestroyWindow("CO");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);
		                return true;
		        }
		}
		
	}
	cout << " nothing detected " << endl;
	
	return false;
}*/
bool GridController::DetectCup(CvPoint& cpt,int camID)
{
	cout << "In DetectCup" << endl;
	ViewImage *img = m_view->captureView(camID, COLORBIT_24, IMAGE_320X240); 
	char *buf = img->getBuffer();

        IplImage *red = cvCreateImage(cvSize(320, 240), 8, 3);
        memcpy(red->imageData, buf, red->imageSize);

	ViewImage *dis_img = m_view->distanceSensor2D(0.0, 1023.0, camID, DEPTHBIT_8, IMAGE_320X240);  
        char *dis_buf = dis_img->getBuffer();

	IplImage *src = cvCreateImage(cvSize(320, 240), 8, 1);
	memcpy(src->imageData, dis_buf, src->imageSize);

	unsigned char* imagedata = (unsigned char*)src->imageData;
	int step = src->widthStep / sizeof(unsigned char);


	unsigned char* rimagedata = (unsigned char*)red->imageData;
	int rstep = red->widthStep / sizeof(unsigned char);

	long average = 0;
	int i;
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			average += imagedata[i * step + j];
		}
	}
	average /= (src->height * src->width);
	//cout << average << endl;

	cvThreshold(src, src, average, 255, CV_THRESH_BINARY);
	
	int mi_red = 0;
	int mj_red = 0;
	int amount_red = 0;
	int else_count_red=0;
	
	int mi_can_green = 0;
	int mj_can_green = 0;
	int amount_can_green = 0;
	int else_can_count_green=0;
	
	int mi_mug_green = 0;
	int mj_mug_green = 0;
	int amount_mug_green = 0;
	int else_mug_count_green=0;
	
	int mi_blue = 0;
	int mj_blue = 0;
	int amount_blue = 0;
	int else_count_blue=0;
	
	int mi_yellow = 0;
	int mj_yellow = 0;
	int amount_yellow = 0;
	int else_count_yellow=0;
	
	for(i = 0; i < src->height; i++)
	{
		for(int j = 0; j < src->width; j++)
		{
			if(imagedata[i * step + j] == 0)
			{
				unsigned char B = rimagedata[i * rstep + j * 3];
				unsigned char G = rimagedata[i * rstep + j * 3 + 1];
				unsigned char R = rimagedata[i * rstep + j * 3 + 2];
				
				if( R > 60 && R < 150 && G > 180 && G < 210 && B > 200 && B < 205)//blue
				{
				        mi_blue += i;
					mj_blue += j;
					amount_blue++; 
					
					else_count_red++;
				        else_can_count_green++;
				        else_count_yellow++;
				        else_mug_count_green++;
				
				}
				else if(//(R > 125 && R < 140 && G > 30 && G < 35 && B > 30 && B < 35)
					(R > 240 && G > 135 && G < 150 && B > 135 && B < 150))  //yellow
				{
				        mi_yellow += i;
					mj_yellow += j;
					amount_yellow++;
					
					else_count_red++;
				        else_can_count_green++;
				        else_count_blue++;
				        else_mug_count_green++;
				        
				}
				else if(R > 100 && R / G >= 2 && R / B >= 8)  //red
				{
					mi_red += i;
					mj_red += j;
					amount_red++;
					
					else_can_count_green++;
				        else_count_blue++;
				        else_count_yellow++;
				        else_can_count_green++;
				        else_mug_count_green++;
					
				}
				else if(R < 120 && R > 90 && G > 170 && G < 190 && B < 105 && B > 80)//mug_green
				{
				        mi_mug_green += i;	
				        mj_mug_green += j;
					amount_mug_green++;  
					
					else_count_red++; 
				        else_count_blue++;
				        else_count_yellow++;  
				           
				} 
				else if(R < 180 && R > 170 && G > 200 && G < 210 && B < 180 && B > 150)//can_green
				{
				        mi_can_green += i;	
				        mj_can_green += j;
					amount_can_green++;  
					
					else_count_red++; 
				        else_count_blue++;
				        else_count_yellow++;  
				           
				} 
				else
				{
				        else_count_red++;
				        else_can_count_green++;
				        else_mug_count_green++;
				        else_count_blue++;
				        else_count_yellow++;
				}
			}
		}
	}
	//cout << "amount_red = " << amount_red << endl;
	//cout << "amount_green = " << amount_green << endl;
	if(amount_blue > 0 && amount_blue < 240*56)
	{
	        
		mi_blue /= amount_blue;
		mj_blue /= amount_blue;
		
		cpt.x = mj_blue;
		cpt.y = mi_blue;
                
                CvPoint pt;
		pt.x=cpt.x/2;
		pt.y=cpt.y/2;
		int dis=(unsigned char)dis_buf[cpt.y*step+cpt.x];
		int else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		cout << "                          dis= " << dis << endl;
		cout << "                          else_dis= " << else_dis << endl;
		cout << "else_count_blue= " << else_count_blue << endl;
		if(fabs(double(dis-else_dis)) > 10||is_near)
		{
			cout << "Not plant" << endl;
		        if(double(else_count_blue)/amount_blue>DIS)
                        {  
                                cout<<"\t\t\t\t\t find blue "<<endl;
                                /*cvCircle(red, cvPoint(mj_blue, mi_blue), 10, cvScalar(255, 255, 255), 3);
		                cvNamedWindow("CO");
		                cvShowImage("CO", red);
		                cvWaitKey(1000);
		                cvDestroyWindow("CO");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*//////
		                return true;
		        }
		}
		
	}
	
	if(amount_yellow > 0 && amount_yellow < 240*36)
	{
	        
		mi_yellow /= amount_yellow;
		mj_yellow /= amount_yellow;
		
		cpt.x = mj_yellow;
		cpt.y = mi_yellow;
                
                CvPoint pt;
		pt.x=cpt.x/2;
		pt.y=cpt.y/2;
		int dis=(unsigned char)dis_buf[cpt.y*step+cpt.x];
		int else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		cout << "                          dis= " << dis << endl;
		cout << "                          else_dis= " << else_dis << endl;
		cout << "else_count_yellow= " << else_count_yellow << endl;
		if(fabs(double(dis-else_dis)) > 10||is_near)
		{
			cout << "Not plant" << endl;
		        if(double(else_count_yellow)/amount_yellow>DIS)
                        {  
                                cout<<"\t\t\t\t\t find yellow "<<endl;
                                /*cvCircle(red, cvPoint(mj_yellow, mi_yellow), 10, cvScalar(255, 255, 255), 3);
		                cvNamedWindow("CO");
		                cvShowImage("CO", red);
		                cvWaitKey(1000);
		                cvDestroyWindow("CO");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*//////
		                return true;
		        }
		}
		
	}
	cout << "amount_red = " << amount_red << endl;
	if(amount_red > 0 && amount_red < 240*20)
	{
	        
		mi_red /= amount_red;
		mj_red /= amount_red;
		
		cpt.x = mj_red;
		cpt.y = mi_red;
                
                CvPoint pt;
		pt.x=cpt.x/2;
		pt.y=cpt.y/2;
		int dis=(unsigned char)dis_buf[cpt.y*step+cpt.x];
		int else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		cout << " else_count_red= " << else_count_red << endl;
		cout << "                      dis= " << dis << endl;
		cout << "                      else_dis= " << else_dis << endl;
		
		if(fabs(double(dis-else_dis))>12||is_near) 
		{
			cout << "Not plant" << endl;
		        if(double(else_count_red)/amount_red>DIS)
                        {
                                cout<<"\t\t\t\t\t find red "<<endl;
                                /*cvCircle(red, cvPoint(mj_red, mi_red), 10, cvScalar(255, 255, 255), 3);
		                cvNamedWindow("CO");
		                cvShowImage("CO", red);
		                cvWaitKey(1000);
		                cvDestroyWindow("CO");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*/////
		                return true;
		        }
		}
		
	}
	if(amount_mug_green > 0 && amount_mug_green < 240*36)
	{
	        
		mi_mug_green /= amount_mug_green;
		mj_mug_green /= amount_mug_green;
		
		cpt.x = mj_mug_green;
		cpt.y = mi_mug_green;
                
                CvPoint pt;
		pt.x=cpt.x/2;
		pt.y=cpt.y/2;
		int dis=(unsigned char)dis_buf[cpt.y*step+cpt.x];
		int else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		cout << "                          dis= " << dis << endl;
		cout << "                          else_dis= " << else_dis << endl;
		cout << "else_count__green= " << else_mug_count_green << endl;
		if(fabs(double(dis-else_dis)) > 10||is_near)
		{
			cout << "Not plant" << endl;
		        if(double(else_mug_count_green)/amount_mug_green>DIS)
                        {  
                                cout<<"\t\t\t\t\t find green "<<endl;
                                /*cvCircle(red, cvPoint(mj_mug_green, mi_mug_green), 10, cvScalar(255, 255, 255), 3);
		                cvNamedWindow("CO");
		                cvShowImage("CO", red);
		                cvWaitKey(1000);
		                cvDestroyWindow("CO");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*/////
		                return true;
		        }
		}
		
	}
	if(amount_can_green > 0 && amount_can_green < 240*36)
	{
	        
		mi_can_green /= amount_can_green;
		mj_can_green /= amount_can_green;
		
		cpt.x = mj_can_green;
		cpt.y = mi_can_green;
                
                CvPoint pt;
		pt.x=cpt.x/2;
		pt.y=cpt.y/2;
		int dis=(unsigned char)dis_buf[cpt.y*step+cpt.x];
		int else_dis=(unsigned char)dis_buf[pt.y*step+pt.x];
		cout << "                          dis= " << dis << endl;
		cout << "                          else_dis= " << else_dis << endl;
		cout << "else_count_can_green= " << else_can_count_green << endl;
		if(fabs(double(dis-else_dis)) > 10||is_near)
		{
			cout << "Not plant" << endl;
		        if(double(else_can_count_green)/amount_can_green>DIS)
                        {  
                                cout<<"\t\t\t\t\t find green "<<endl;
                                /*cvCircle(red, cvPoint(mj_can_green, mi_can_green), 10, cvScalar(255, 255, 255), 3);
		                cvNamedWindow("CO");
		                cvShowImage("CO", red);
		                cvWaitKey(1000);
		                cvDestroyWindow("CO");
		                cvReleaseImage(&src);
		                cvReleaseImage(&red);*//////
		                return true;
		        }
		}
		
	}
	
	
	cout << " nothing detected " << endl;
	
	return false;
}
bool GridController::DealDetectCup()
{
	CvPoint cpt;
	if(DetectCup(cpt, 1))
	{
		cout << "CUP DETECTED =====" << endl;
		cout << "cpt = " << cpt.x << "\t" << cpt.y << endl;
		cordinate cor;
		int dis = GETPOINTDIS2(cpt, cor, 1);
		cout << "dis = " << dis << "\t" << cor.i << "\t" << cor.j << endl;
		cordinate w;
		TRANS(cor, w);
		cout << "world = " << w.i << "\t" << w.j << endl;
		g_map.Add(w.j, w.i, cvScalar(255, 0, 255));
		map_d.push_back(GPoint(w.i, w.j, 4));
		d_cX = (d_cX * amount_c + w.i) / (amount_c + 1);
		d_cZ = (d_cZ * amount_c + w.j) / (amount_c + 1);
		amount_c++;
		cout << "CUP = " << d_cX << "\t" << d_cZ << endl;
		return true;
	}
	return false;
}

extern "C" Controller * createController() 
{  
  	return new GridController;  
}  

