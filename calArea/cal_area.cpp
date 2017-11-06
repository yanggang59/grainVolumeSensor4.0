
#include "cal_area.h"
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

/*用来计算截面积*/
/*diatance是相机距离 面板的距离,用来算点到刮板的实际距离，width是刮板的宽度,用来剔除不需要的点*/
float cal_area(vector<Vec3f> pointsAfterTransform, float distance, float width)
{

	/*找到谷物上的点*/
	vector<Vec3f> pointsOnGrain;
	for (int i = 0; i<pointsAfterTransform.size(); i++)
	{
		if (pointsAfterTransform[i][2] - distance<width / 2 && pointsAfterTransform[i][2] - distance>0)
			pointsOnGrain.push_back(pointsAfterTransform[i]);

	}

	//如果没有点，则面积为0
	if (0 == pointsOnGrain.size())
		return 0;



	/*找到x坐标的最大值和最小值，以及最值点的索引*/
	float max_x_coor = pointsOnGrain[1][0];
	float min_x_coor = pointsOnGrain[1][0];
	int x_max_index = 0, x_min_index = 0;

	for (int i = 0; i<pointsOnGrain.size(); i++)
	{
		if (max_x_coor<pointsOnGrain[i][0])
		{
			max_x_coor = pointsOnGrain[i][0];
			x_max_index = i;
		}
		if (min_x_coor>pointsOnGrain[i][0])
		{
			min_x_coor = pointsOnGrain[i][0];
			x_min_index = i;
		}
	}


	/*计算点之间的水平间距*/
	int count = pointsOnGrain.size();
	float gap = (max_x_coor - min_x_coor) / (count - 1);

	/*面积*/
	float area;

	/*z坐标之和*/
	float z_sum = 0;
	for (int i = 0; i<count; i++)
	{
		z_sum += pointsOnGrain[i][2] - distance;
	}
	area = (max_x_coor - min_x_coor)*(pointsOnGrain[x_max_index][2] + pointsOnGrain[x_min_index][2]) / 2 - z_sum*gap;

	return area;
}