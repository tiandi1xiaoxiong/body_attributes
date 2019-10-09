#include <iostream>
#include <string>
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iomanip>
#include <pcl/common/transforms.h> 
#include <cmath>
#include <pcl/filters/statistical_outlier_removal.h>
using namespace std;

// 定义点云类型

typedef pcl::PointXYZRGB PointT;

typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参

const double camera_factor = 100;

const double camera_cx = 253.684;

const double camera_cy = 207.153;

const double camera_fx = 368.091;

const double camera_fy = 368.091;


void cloudFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_cloud)//实参合法的指针给;
{
	//if(!filter_cloud->empty())
	//	filter_cloud->clear();//是全局变量,里面早有东西了,所以再放压要先清掉;
	// 创建滤波器对象;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Filter(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloud_Filter->reserve(src_cloud->size());

	/*filter_cloud->reserve(src_cloud->size());*/

	/*for(int i = 0; i < src_cloud->size(); i++)
	{
		if( src_cloud->points[i].x > 2.0)
			cloud_Filter->push_back(src_cloud->points[i]);
	}*/

	for (int i = 0; i < src_cloud->size(); i++)
	{
		/*if (src_cloud->points[i].z > 5.5 && src_cloud->points[i].z < 6.5)
			if (src_cloud->points[i].x > -1.5 && src_cloud->points[i].x < 1.5)
				if (src_cloud->points[i].y > -1.5 && src_cloud->points[i].y < 1.5)*/
		if (src_cloud->points[i].z < 2.5)//距离相机3-4m可以使用此滤去背景;
		{
			pcl::PointXYZRGB point;
			point.x = src_cloud->points[i].x;
			point.y = src_cloud->points[i].y;
			point.z = src_cloud->points[i].z;
			point.b = src_cloud->points[i].b;
			point.g = src_cloud->points[i].g;
			point.r = src_cloud->points[i].r;
			cloud_Filter->push_back(point);
			/*filter_cloud->push_back(point);*/
		}
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_staticFilter(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_staticFilter->reserve(cloud_Filter->size());
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud_Filter);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1.0);
	sor.filter(*filter_cloud);
}


void find_n_max_number(vector<double> * array, int number)
{
	int inner;
	int outer;
	double median;

	if (1 == array->empty() || 0 == array->size())
		return;

	if (number > array->size())
		return;

	for (outer = array->size() - 1; outer > (array->size() - 1 - number); outer--)
	{
		for (inner = 0; inner < outer; inner++)
		{
			if (array->at(inner) > array->at(inner + 1)) //">"冒泡找最大，“<”冒泡找最小；
			{
				median = array->at(inner);
				array->at(inner) = array->at(inner + 1);
				array->at(inner + 1) = median;
			}
		}
	}
}

void find_n_min_number(vector<double> * array, int number)
{
	int inner;
	int outer;
	double median;

	if (1 == array->empty() || 0 == array->size())
		return;

	if (number > array->size())
		return;

	for (outer = array->size() - 1; outer > (array->size() - 1 - number); outer--)
	{
		for (inner = 0; inner < outer; inner++)
		{
			if (array->at(inner) < array->at(inner + 1)) //">"冒泡找最大，“<”冒泡找最小；
			{
				median = array->at(inner);
				array->at(inner) = array->at(inner + 1);
				array->at(inner + 1) = median;
			}
		}
	}
}

int main(int argc, char** argv)

{
	cv::Mat rgb, depth;
	rgb = cv::imread(argv[1]);

	// rgb 图像是8UC3的彩色图像

	// depth 是8UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改

	depth = cv::imread(argv[2], -1);

	// 点云变量

	// 使用智能指针，创建一个空点云。这种指针用完会自动释放。

	PointCloud::Ptr cloud(new PointCloud);

	char pcdfilename[25];
	char pcdfilename_filter1[25];
	char pcdfilename_filter2[25];
	char pcdfilename_filter3[25];

	int row1 = atoi(argv[3]);
	int row2 = atoi(argv[4]);
	int col1 = atoi(argv[5]);
	int col2 = atoi(argv[6]);

	// 遍历深度图

	for (int m = row1; m < row2; m++)//depth.rows;

		for (int n = col1; n < col2; n++)//depth.cols;

		{
			// 获取深度图中(m,n)处的值

			int d = depth.ptr(m)[n]; //<ushort>;

			// d 可能没有值，若如此，跳过此点

			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			PointT p;

			// 计算这个点的空间坐标

			p.z = double(d) / camera_factor;

			p.x = -(n - camera_cx) * p.z / camera_fx;

			p.y = -(m - camera_cy) * p.z / camera_fy;

			// 从rgb图像中获取它的颜色

			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色

			p.b = rgb.ptr<uchar>(m)[n * 3];

			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];

			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// 把p加入到点云中

			cloud->points.push_back(p);

		}

	// 设置并保存点云

	cloud->height = 1;

	cloud->width = cloud->points.size();

	sprintf(pcdfilename, "%.2s%s", argv[1], "_full.pcd");//与原main函数相比就改了一个文件名字;

	pcl::io::savePCDFile(pcdfilename, *cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter1(new pcl::PointCloud<pcl::PointXYZRGB>);//滤除背景；

	cloudFilter(cloud, cloud_filter1);

	sprintf(pcdfilename_filter1, "%.2s%s", argv[1], "_filter1.pcd");

	pcl::io::savePCDFile(pcdfilename_filter1, *cloud_filter1);

	double z1 = 0;
	double accum = 0;
	vector <double>xcal;
	vector <double>ycal;

	for (int i = 0; i < cloud_filter1->size(); i++)
		z1 += cloud_filter1->points[i].z;
	double dis = z1 / cloud_filter1->size();

	for (int i = 0; i < cloud_filter1->size(); i++)
		accum += (cloud_filter1->points[i].z - dis)*(cloud_filter1->points[i].z - dis);
	double variance = sqrt(accum / cloud_filter1->size());


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter2(new pcl::PointCloud<pcl::PointXYZRGB>);//滤去因因地面反光产生的噪点;
	cloud_filter2->reserve(cloud_filter1->size());
	for (int i = 0; i < cloud_filter1->size(); i++)
	{
		if (cloud_filter1->points[i].z >= dis - variance * 2.5)
		{
			cloud_filter2->push_back(cloud_filter1->points[i]);
			ycal.push_back(cloud_filter1->points[i].y);
		}
	}
	sprintf(pcdfilename_filter2, "%.2s%s", argv[1], "_filter2.pcd");
	pcl::io::savePCDFile(pcdfilename_filter2, *cloud_filter2);

	double xall_max = 0.0;
	double yall_max = 0.0;
	double xall_min = 0.0;
	double yall_min = 0.0;
	int num;
	sscanf_s(argv[7], "%d", &num);//防止滤波不干净的噪声影响，求num个最值再平均为最值;

	find_n_max_number(&ycal, num);
	for (int i = 0; i < num; i++)
		yall_max += ycal[ycal.size() - 1 - i];
	double ymax_avg = yall_max / num;
	find_n_min_number(&ycal, num);
	for (int i = 0; i < num; i++)
		yall_min += ycal[ycal.size() - 1 - i];
	double ymin_avg = yall_min / num;

	double hei = ymax_avg - ymin_avg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter3(new pcl::PointCloud<pcl::PointXYZRGB>);//滤去肩部以下部分;
	cloud_filter3->reserve(cloud_filter2->size());
	for (int i = 0; i < cloud_filter2->size(); i++)
	{
		if ((cloud_filter2->points[i].y - ymin_avg) / hei >= 0.80)//0.8为肩部左右的点;
		{
			cloud_filter3->push_back(cloud_filter2->points[i]);
			xcal.push_back(cloud_filter2->points[i].x);
		}
	}

	sprintf(pcdfilename_filter3, "%.2s%s", argv[1], "_filter3.pcd");
	pcl::io::savePCDFile(pcdfilename_filter3, *cloud_filter3);//可以不需要，就是为了看看最终的滤波效果;

	find_n_max_number(&xcal, num);
	for (int i = 0; i < num; i++)
		xall_max += xcal[xcal.size() - 1 - i];
	double xmax_avg = xall_max / num;
	find_n_min_number(&xcal, num);
	for (int i = 0; i < num; i++)
		xall_min += xcal[xcal.size() - 1 - i];
	double xmin_avg = xall_min / num;

	double wid = xmax_avg - xmin_avg;

	double factor_guess = 4.0 / 2.23;
	double height = hei * factor_guess;
	double width = wid * factor_guess;

	cout << "heignt:" << height << endl;
	cout << "scale" << width / height << endl;
	
	// 清除数据并退出

	cloud->points.clear();
	cloud_filter1->points.clear();
	cloud_filter2->points.clear();
	cloud_filter3->points.clear();
	return 0;

}
