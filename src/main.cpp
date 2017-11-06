
/*
this is a program created by YangGang,ME,SJTU,CN

if you need to use any code from here ,please at least let me know 
using this email:1449381582@qq.com

good luck!

*/


/*OpenCV�ͼ���������ص�ͷ�ļ�*/
#include <opencv2/opencv.hpp>
#include "preprocessing.h" 
#include "cal_3d_coor.h"
#include "cal_area.h"




 //������ص�ͷ�ļ�  
#include<stdio.h>      /*��׼�����������*/  
#include<stdlib.h>     /*��׼�����ⶨ��*/  
#include<unistd.h>     /*Unix ��׼��������*/  
#include<sys/types.h>   
#include<sys/stat.h>     
#include<fcntl.h>      /*�ļ����ƶ���*/  
#include<termios.h>    /*PPSIX �ն˿��ƶ���*/  
#include<errno.h>      /*����Ŷ���*/  
#include<string.h>    

//�궨��  
#define FALSE  -1  
#define TRUE   0  



using namespace cv;
using namespace std;

float a, b, c, A, B, C; //abc��ax+by+cz+1=0����ʽ�Ĳ�����ABC��Z=Ax+By+C��ʽ�Ĳ���,��Ҫת��һ��

/*�ڲ�������֮ǰ�Ѿ��궨��*/
Mat K = (Mat_<float>(3, 3) << 518.8906422065566, 0, 294.5896218285499,
								0, 520.0230989700873, 226.11902054293,
								0, 0, 1);
								
VideoCapture capLeft(0);    //������
VideoCapture capRight(1);   //�ұ����

Mat leftFrame,rightFrame;    //�����������Ƭ

int main()
{
	/*�����ṹ�������֮ǰ�궨�õ�*/
	//FileStorage fs("C://test_pic//test.yml",FileStorage::READ);
	//fs["A"]>>A;
	//fs["B"]>>B;
	//fs["C"]>>C;
	//fs.release();

	a = 0;
	b = -0.0153846;
	c = -0.0068497;

	A = -a / c;
	B = -b / c;
	C = -1 / c;

	Vec3f coeff(A, B, C);

	//Mat Input = imread("../../res/right01.jpg",1);

	Mat leftMask = imread("../../res/left_mask.jpg", 0);
	Mat rightMask = imread("../../res/right_mask.jpg", 0);
	
	int width = 640;
	int height = 480;
	Size ImageSize(width, height);
	
	/*������ص�����*/
		int fd;                            //�ļ�������  
		int err;                           //���ص��ú�����״̬  
		int len;                          
		char send_buf[8]={0};  
		struct termios options; 
		
		fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY); //�򿪴��ڣ������ļ�������  
	
	while(1)
	{
		
		/*���򿪴���ʧ�ܣ����˳�*/
		if  ( tcgetattr( fd,&options)  !=  0)  
		   {  
			  perror("SetupSerial 1");      
			  return(FALSE);   
		   } 
		   
		   
		   
		//������������ұ����δ�򿪣����˳�
		if(!capLeft.isOpened() | !capRight.isOpened())
		{
			cout<<"open camera  failure"<<endl;
			return (FALSE);
		}
		
		capLeft>>leftFrame;
		capRight>>rightFrame;
		
	Mat left_skeleton(Size(width, height), CV_8UC1, Scalar(0));
	Mat right_skeleton(Size(width, height), CV_8UC1, Scalar(0));
	
	
	int countLeft = 0;
	int countRight = 0;
	
	//ͼ��Ԥ������,Input��ʾ����ͼ��uMask��ģͼ��skeleton������ĹǼܣ�count�����Ǽܵ�ĸ���
	preprocessing(leftFrame, leftMask, left_skeleton, ImageSize,countLeft);
	preprocessing(rightFrame, rightMask, right_skeleton, ImageSize,countRight);
	//imshow("left_skeleton", left_skeleton);
	//cout << "countLeft is " << countLeft << endl;

	//�����ĸ�������50������Ϊ�ǿհ�֡,Ӧ����������Ϊ0��continue��������һ��ѭ��

	if (countLeft<50 |countRight<50)
	{
		cout << "this frame is void \n " << endl;
		//continue;
	}
  
	

	/*����ֱ�������е���������������*/
	vector<Vec3f> allPointsLeft;
	vector<Vec3f> allPointsRight;
	
	int z_count_left = 0;  //��¼����z_threshold�ĵ�ĸ���
	int z_count_right = 0;  //��¼����z_threshold�ĵ�ĸ���
	
	int z_threshold = 20;
	
																																																																																	
	cal_3d_coor(left_skeleton, allPointsLeft, K, coeff, z_count_left, z_threshold);
	cal_3d_coor(right_skeleton, allPointsRight, K, coeff, z_count_right, z_threshold);
	
	
	cout << "number of left points that are larger than z_threshold is " << z_count_left << endl;
	cout << "the count of all left points is ��" << allPointsLeft.size() << endl;
	
	cout << "number of right points that are larger than z_threshold is " << z_count_left << endl;
	cout << "the count of all right points is ��" << allPointsLeft.size() << endl;	

	if (z_count_left<50 | z_count_right<50)
	{
		cout << "this frame is a board frame" << endl;
	}


	//�������,�ȷֱ�������������Ȼ�����,����ĵ�λ��ƽ������
	int distance=40, board_width = 190;
	float area_leftSide = cal_area(allPointsLeft, distance, board_width);
	float area_rightSide = cal_area(allPointsRight, distance, board_width);
	
	float area=area_leftSide+area_rightSide;
	cout << "area is " << area << endl;
	
	
	
	/*��������,�ٶ���330mm/s,��˼�������Ľ������������/s������1000,��λ����������/s��Ҳ����L/s,����ʱҪת��Ϊint����*/
	float velocity=330.0;
	float volume=velocity*area;
	
	int n_volume=(int)(volume/1000);
	
	
	send_buf[2]=n_volume%256;
    send_buf[3]=n_volume/256;
	
	
	
	
		
	/*���ڷ���*/  
	   cfsetospeed(&options,B115200);    
	  //�޸Ŀ���ģʽ����֤���򲻻�ռ�ô���  
		options.c_cflag |= CLOCAL;  
		//�޸Ŀ���ģʽ��ʹ���ܹ��Ӵ����ж�ȡ��������  
		options.c_cflag |= CREAD;  
		
		//��ʹ��������  
	   options.c_cflag &= ~CRTSCTS;  
	   
	   //��������λ��8λ
		options.c_cflag |= CS8; 

		//��У��λ
		 options.c_cflag &= ~PARENB;   
		 options.c_iflag &= ~INPCK;     
		
		//һ��ֹͣλ
		 options.c_cflag &= ~CSTOPB; 
		 
		 
		 //�����������������������ݣ����ǲ��ٶ�ȡ ˢ���յ������ݵ��ǲ���  
		tcflush(fd,TCIFLUSH);  
		 
		//�������� (���޸ĺ��termios�������õ������У�  
	   int test= tcsetattr(fd,TCSANOW,&options) ; 
			  

	  //printf("Set Port Exactly!\n");  
	  
	  write(fd,(send_buf),8);
	                     
	//waitKey();
	}
	
	  close(fd); 
}
