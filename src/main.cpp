
/*
this is a program created by YangGang,ME,SJTU,CN

if you need to use any code from here ,please at least let me know 
using this email:1449381582@qq.com

good luck!

*/


/*OpenCV和计算截面积相关的头文件*/
#include <opencv2/opencv.hpp>
#include "preprocessing.h" 
#include "cal_3d_coor.h"
#include "cal_area.h"




 //串口相关的头文件  
#include<stdio.h>      /*标准输入输出定义*/  
#include<stdlib.h>     /*标准函数库定义*/  
#include<unistd.h>     /*Unix 标准函数定义*/  
#include<sys/types.h>   
#include<sys/stat.h>     
#include<fcntl.h>      /*文件控制定义*/  
#include<termios.h>    /*PPSIX 终端控制定义*/  
#include<errno.h>      /*错误号定义*/  
#include<string.h>    

//宏定义  
#define FALSE  -1  
#define TRUE   0  



using namespace cv;
using namespace std;

float a, b, c, A, B, C; //abc是ax+by+cz+1=0的形式的参数，ABC是Z=Ax+By+C形式的参数,需要转换一下

/*内参数矩阵，之前已经标定好*/
Mat K = (Mat_<float>(3, 3) << 518.8906422065566, 0, 294.5896218285499,
								0, 520.0230989700873, 226.11902054293,
								0, 0, 1);
								
VideoCapture capLeft(0);    //左边相机
VideoCapture capRight(1);   //右边相机

Mat leftFrame,rightFrame;    //左右相机的照片

int main()
{
	/*读出结构光参数，之前标定好的*/
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
	
	/*串口相关的设置*/
		int fd;                            //文件描述符  
		int err;                           //返回调用函数的状态  
		int len;                          
		char send_buf[8]={0};  
		struct termios options; 
		
		fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY); //打开串口，返回文件描述符  
	
	while(1)
	{
		
		/*若打开串口失败，则退出*/
		if  ( tcgetattr( fd,&options)  !=  0)  
		   {  
			  perror("SetupSerial 1");      
			  return(FALSE);   
		   } 
		   
		   
		   
		//若左相机或者右边相机未打开，则退出
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
	
	//图像预处理函数,Input表示输入图像，uMask掩模图像，skeleton是输出的骨架，count计数骨架点的个数
	preprocessing(leftFrame, leftMask, left_skeleton, ImageSize,countLeft);
	preprocessing(rightFrame, rightMask, right_skeleton, ImageSize,countRight);
	//imshow("left_skeleton", left_skeleton);
	//cout << "countLeft is " << countLeft << endl;

	//如果点的个数少于50，则认为是空白帧,应该输出截面积为0，continue，进行下一次循环

	if (countLeft<50 |countRight<50)
	{
		cout << "this frame is void \n " << endl;
		//continue;
	}
  
	

	/*计算直线上所有点相对于相机的坐标*/
	vector<Vec3f> allPointsLeft;
	vector<Vec3f> allPointsRight;
	
	int z_count_left = 0;  //记录大于z_threshold的点的个数
	int z_count_right = 0;  //记录大于z_threshold的点的个数
	
	int z_threshold = 20;
	
																																																																																	
	cal_3d_coor(left_skeleton, allPointsLeft, K, coeff, z_count_left, z_threshold);
	cal_3d_coor(right_skeleton, allPointsRight, K, coeff, z_count_right, z_threshold);
	
	
	cout << "number of left points that are larger than z_threshold is " << z_count_left << endl;
	cout << "the count of all left points is ：" << allPointsLeft.size() << endl;
	
	cout << "number of right points that are larger than z_threshold is " << z_count_left << endl;
	cout << "the count of all right points is ：" << allPointsLeft.size() << endl;	

	if (z_count_left<50 | z_count_right<50)
	{
		cout << "this frame is a board frame" << endl;
	}


	//计算面积,先分别计算左右面积，然后相加,这里的单位是平方毫米
	int distance=40, board_width = 190;
	float area_leftSide = cal_area(allPointsLeft, distance, board_width);
	float area_rightSide = cal_area(allPointsRight, distance, board_width);
	
	float area=area_leftSide+area_rightSide;
	cout << "area is " << area << endl;
	
	
	
	/*计算流量,速度是330mm/s,因此计算出来的结果是立方毫米/s，除以1000,单位是立方分米/s，也就是L/s,发送时要转化为int类型*/
	float velocity=330.0;
	float volume=velocity*area;
	
	int n_volume=(int)(volume/1000);
	
	
	send_buf[2]=n_volume%256;
    send_buf[3]=n_volume/256;
	
	
	
	
		
	/*串口发送*/  
	   cfsetospeed(&options,B115200);    
	  //修改控制模式，保证程序不会占用串口  
		options.c_cflag |= CLOCAL;  
		//修改控制模式，使得能够从串口中读取输入数据  
		options.c_cflag |= CREAD;  
		
		//不使用流控制  
	   options.c_cflag &= ~CRTSCTS;  
	   
	   //设置数据位，8位
		options.c_cflag |= CS8; 

		//无校验位
		 options.c_cflag &= ~PARENB;   
		 options.c_iflag &= ~INPCK;     
		
		//一个停止位
		 options.c_cflag &= ~CSTOPB; 
		 
		 
		 //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读  
		tcflush(fd,TCIFLUSH);  
		 
		//激活配置 (将修改后的termios数据设置到串口中）  
	   int test= tcsetattr(fd,TCSANOW,&options) ; 
			  

	  //printf("Set Port Exactly!\n");  
	  
	  write(fd,(send_buf),8);
	                     
	//waitKey();
	}
	
	  close(fd); 
}
