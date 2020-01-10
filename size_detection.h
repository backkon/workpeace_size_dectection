#pragma once
#include <iostream>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h" 
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;

void dataprocess(Point2f &p1, Point2f &p2, Point2f &p3,
	Vec4f &line1, Vec4f &line2, Vec4f &line3, Vec4f &k1_line, Vec4f &k2_line, Vec4f &k3_line,
	double &short_line, double &long_line, double &k1_fit, double &k2_fit, double &k3_fit)
{
	p1.x = (-k2_fit * k2_line[2] + k2_line[3] + k1_fit * k1_line[2] - k1_line[3]) / (k1_fit - k2_fit);
	p1.y = k2_fit * (p1.x - k2_line[2]) + k2_line[3];
	p2.x = (-k2_fit * k2_line[2] + k2_line[3] + k3_fit * k3_line[2] - k3_line[3]) / (k3_fit - k2_fit);
	p2.y = k2_fit * (p2.x - k2_line[2]) + k2_line[3];
	p3.x = (-k3_fit * k3_line[2] + k3_line[3] + k1_fit * k1_line[2] - k1_line[3]) / (k1_fit - k3_fit);
	p3.y = k3_fit * (p3.x - k3_line[2]) + k3_line[3];
	if (abs(k1_fit * k3_fit + 1) < 0.1)
	{
		double l1 = sqrt(pow((p1.x - p3.x), 2) + pow((p1.y - p3.y), 2));
		double l2 = sqrt(pow((p3.x - p2.x), 2) + pow((p3.y - p2.y), 2));
		if (l1 < l2)
		{
			line1 = k1_line;
			line2 = k3_line;
			line3 = k2_line;
			short_line = l1;
			long_line = l2;
		}
		else
		{
			line1 = k3_line;
			line2 = k1_line;
			line3 = k2_line;
			short_line = l2;
			long_line = l1;
		}
	}
	else if (abs(k1_fit * k2_fit + 1) < 0.1)
	{
		double l1 = sqrt(pow((p1.x - p3.x), 2) + pow((p1.y - p3.y), 2));
		double l2 = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
		if (l1 < l2)
		{
			line1 = k1_line;
			line2 = k2_line;
			line3 = k3_line;
			short_line = l1;
			long_line = l2;
		}
		else
		{
			line1 = k1_line;
			line2 = k2_line;
			line3 = k3_line;
			short_line = l2;
			long_line = l1;
		}
	}
	else
	{
		double l1 = sqrt(pow((p3.x - p2.x), 2) + pow((p3.y - p2.y), 2));
		double l2 = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
		if (l1 < l2)
		{
			line1 = k3_line;
			line2 = k2_line;
			line3 = k1_line;
			short_line = l1;
			long_line = l2;
		}
		else
		{
			line1 = k2_line;
			line2 = k3_line;
			line3 = k1_line;
			short_line = l2;
			long_line = l1;
		}
	}
}

int size_detection(Mat src, double &a, double &b, double &d, double &e, double &f, double &alpha, double &scale_factor)
{
	Mat src_gray_line;
	cvtColor(src, src_gray_line, COLOR_RGB2GRAY);
	Mat src_gray_circle = src_gray_line.clone();


	//canny边缘提取
	Mat edges, edges2;
	Canny(src_gray_line, edges, 170, 500, 3);
	Canny(src_gray_circle, edges2, 150, 500, 3);


	//****************************//
	//**********直线检测***********//
	//****************************//
	vector<Vec4f> lines;
	Mat probabilistic_hough;
	cvtColor(edges, probabilistic_hough, CV_GRAY2BGR);
	HoughLinesP(edges, lines, 1, CV_PI / 180, 200, 30, 10);
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4f control_points = lines[i];
		line(probabilistic_hough, Point(control_points[0], control_points[1]), Point(control_points[2], control_points[3]), Scalar(0, 0, 255), 3, CV_AA);
	}
	vector<Vec4f> lines_copy;
	lines_copy = lines;
	//namedWindow("霍夫变换检测直线", WINDOW_AUTOSIZE);
	//namedWindow("霍夫变换检测直线", WINDOW_NORMAL);
	//imshow("霍夫变换检测直线", probabilistic_hough);
	
	double k_equal0 = 0, k1, k2, k3, k_temp;
	//各种类型的控制点集合
	vector<Point> no_k_points, k_equal0_points, k1_points, k2_points, k3_points;
	//控制点1与控制点2
	Point point1_temp, point2_temp;
	int flag = 0, num_no_k = 0, num_k_equal0 = 0, num_k1 = 0, num_k2 = 0, num_k3 = 0;

	for (size_t i = 1; i < lines_copy.size(); i++)
	{
		if (lines_copy[i][2] == lines_copy[i][0])
		{
			point1_temp.x = lines_copy[i][0];
			point1_temp.y = lines_copy[i][1];
			point2_temp.x = lines_copy[i][2];
			point2_temp.y = lines_copy[i][3];
			no_k_points.push_back(point1_temp);
			no_k_points.push_back(point2_temp);
			num_no_k++;
		}
		else if (lines_copy[i][3] == lines_copy[i][1])
		{
			point1_temp.x = lines_copy[i][0];
			point1_temp.y = lines_copy[i][1];
			point2_temp.x = lines_copy[i][2];
			point2_temp.y = lines_copy[i][3];
			k_equal0_points.push_back(point1_temp);
			k_equal0_points.push_back(point2_temp);
			num_k_equal0++;
		}
		else
		{
			k_temp = (lines_copy[i][3] - lines_copy[i][1]) / (lines_copy[i][2] - lines_copy[i][0]);
			if (flag == 0)
			{
				k1 = k_temp;
				flag++;
			}
			if (abs((k_temp - k1) / k1) < 0.6)
			{
				point1_temp.x = lines_copy[i][0];
				point1_temp.y = lines_copy[i][1];
				point2_temp.x = lines_copy[i][2];
				point2_temp.y = lines_copy[i][3];
				k1_points.push_back(point1_temp);
				k1_points.push_back(point2_temp);
				num_k1++;
			}
			else
			{
				if (flag == 1)
				{
					k2 = k_temp;
					flag++;
				}
				if (abs((k_temp - k2) / k2) <0.6)
				{
					point1_temp.x = lines_copy[i][0];
					point1_temp.y = lines_copy[i][1];
					point2_temp.x = lines_copy[i][2];
					point2_temp.y = lines_copy[i][3];
					k2_points.push_back(point1_temp);
					k2_points.push_back(point2_temp);
					num_k2++;
				}
				else
				{
					if (flag == 2)
					{
						k3 = k_temp;
						flag++;
					}
					if (abs((k_temp - k3) / k3) < 0.6)
					{
						point1_temp.x = lines_copy[i][0];
						point1_temp.y = lines_copy[i][1];
						point2_temp.x = lines_copy[i][2];
						point2_temp.y = lines_copy[i][3];
						k3_points.push_back(point1_temp);
						k3_points.push_back(point2_temp);
						num_k3++;
					}
				}
			}
		}
	}
	//将属于同一直线的点进行拟合得到一条直线
	int img_width = src.cols;
	int img_height = src.rows;
	//namedWindow("直线拟合结果", CV_WINDOW_NORMAL);
	//fitLine ------ 输入二维点集，输出四维向量（前两维是方向向量，后两维是拟合直线上一点）
	//定义输出变量
	Vec4f no_k_line, k_equal0_line, k1_line, k2_line, k3_line;
	double k_equal0_fit, k1_fit, k2_fit, k3_fit;
	if (!no_k_points.empty())
	{
		fitLine(no_k_points, no_k_line, CV_DIST_L2, 0, 0.01, 0.01);
		Point p1(no_k_line[2], 0);
		Point p2(no_k_line[2], img_height - 1);
		line(src_gray_line, p1, p2, Scalar(0, 0, 255), 2);
	}
	if (!k_equal0_points.empty())
	{
		fitLine(k_equal0_points, k_equal0_line, CV_DIST_L2, 0, 0.01, 0.01);
		k_equal0_fit = k_equal0_line[1] / k_equal0_line[0];
		Point p1(0, k_equal0_fit* (0 - k_equal0_line[2]) + k_equal0_line[3]);
		Point p2(img_width - 1, k_equal0_fit * (img_width - 1 - k_equal0_line[2]) + k_equal0_line[3]);
		line(src_gray_line, p1, p2, Scalar(0, 0, 255), 2);
	}
	if (!k1_points.empty())
	{
		fitLine(k1_points, k1_line, CV_DIST_L2, 0, 0.01, 0.01);
		k1_fit = k1_line[1] / k1_line[0];
		//printf_s("直线1的斜率为：%.2f.\n", k1_fit);
		Point p1(0, k1_fit * (0 - k1_line[2]) + k1_line[3]);
		Point p2(img_width - 1, k1_fit * (img_width - 1 - k1_line[2]) + k1_line[3]);
		line(src_gray_line, p1, p2, Scalar(0, 0, 255), 2);
	}
	if (!k2_points.empty())
	{
		fitLine(k2_points, k2_line, CV_DIST_L2, 0, 0.01, 0.01);
		k2_fit = k2_line[1] / k2_line[0];
		//printf_s("直线2的斜率为：%.2f.\n", k2_fit);
		Point p1(0, k2_fit * (0 - k2_line[2]) + k2_line[3]);
		Point p2(img_width - 1, k2_fit * (img_width - 1 - k2_line[2]) + k2_line[3]);
		line(src_gray_line, p1, p2, Scalar(0, 255, 0), 2);
	}
	if (!k3_points.empty())
	{
		fitLine(k3_points, k3_line, CV_DIST_L2, 0, 0.01, 0.01);
		k3_fit = k3_line[1] / k3_line[0];
		//printf_s("直线3的斜率为：%.2f.\n", k3_fit);
		Point p1(0, k3_fit * (0 - k3_line[2]) + k3_line[3]);
		Point p2(img_width - 1, k3_fit * (img_width - 1 - k3_line[2]) + k3_line[3]);
		line(src_gray_line, p1, p2, Scalar(0, 0, 255), 2);
	}
	//imshow("直线拟合结果", src_gray_line);

	//****************************//
	//**********圆的检测***********//
	//****************************//
	vector <Vec3f> circles;
	int radius = 0;
	Point center;
	//namedWindow("霍夫变换检测圆", WINDOW_NORMAL);
	HoughCircles(edges2, circles, CV_HOUGH_GRADIENT, 1, img_height / 8, 100, 40, 200, 500);
	if (!circles.size())
	{
		//printf_s("没有检测到圆 请重新设置参数！");
	}
	for (size_t i = 0; i < circles.size(); i++)
	{
		//printf_s("第%d个圆的中心坐标为：%.2f,%.2f 半径为：%.2f\n", i + 1, circles[i][0], circles[i][1], circles[i][2]);
		center.x = cvRound(circles[i][0]);
		center.y = cvRound(circles[i][1]);
		radius = cvRound(circles[i][2]);
		circle(edges2, center, radius, Scalar(0, 0, 255), 10);
	}
	//imshow("霍夫变换检测圆", edges2);
	//****************************//
	//**********尺寸计算***********//
	//****************************//
	//p1,p2,p3三条边的三个交点
	Point2f p1, p2, p3;
	//line1,line2,line3分别为从短到长的三条边
	Vec4f line1, line2, line3;
	//short_line,long_line分别为短垂边与长垂边的长度
	double short_line, long_line;
	if ((!no_k_points.empty()) || (!k_equal0_points.empty()))
	{
		if ((!no_k_points.empty()) && !k_equal0_points.empty())
		{
			p1.x = no_k_line[2];
			p1.y = k1_fit * (no_k_line[2] - k1_line[2]) + k1_line[3];
			p2.x = no_k_line[2];
			p2.y = k_equal0_fit * (no_k_line[2] - k_equal0_line[2]) + k_equal0_line[3];
			p3.x = (-k_equal0_fit * k_equal0_line[2] + k_equal0_line[3] + k1_fit * k1_line[2] - k1_line[3]) / (k1_fit - k_equal0_fit);
			p3.y = k_equal0_fit * (p3.x - k_equal0_line[2]) + k_equal0_line[3];
			double l1 = sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
			double l2 = sqrt(pow((p2.x - p3.x), 2) + pow((p2.y - p3.y), 2));
			if (l1 < l2)
			{
				line1 = no_k_line;
				line2 = k_equal0_line;
				line3 = k1_line;
				short_line = l1;
				long_line = l2;
			}
			else
			{
				line1 = k_equal0_line;
				line2 = no_k_line;
				line3 = k1_line;
				short_line = l2;
				long_line = l1;
			}
		}
		else
		{
			//cout << "检测结果中有错误结果" << endl;
			dataprocess(p1, p2, p3, line1, line2, line3, k1_line, k2_line, k3_line, short_line, long_line, k1_fit, k2_fit, k3_fit);
		}
	}
	else
	{
		dataprocess(p1, p2, p3, line1, line2, line3, k1_line, k2_line, k3_line, short_line, long_line, k1_fit, k2_fit, k3_fit);
	}

    //实际尺寸
	a = short_line * scale_factor;
	b = long_line * scale_factor;
	d = 2 * radius * scale_factor;
	if (abs(line1[0]) < 0.01)
	{
		e = abs(line1[2] - center.x) * scale_factor;
		f = abs(line2[3] - center.y) * scale_factor;
	}
	else if (abs(line2[0]) < 0.01)
	{
		e = abs(line1[3] - center.y) * scale_factor;
		f = abs(line2[2] - center.x) * scale_factor;
	}
	else
	{
		e = scale_factor * abs(line1[1] * center.x - line1[0] * center.y - line1[1] * line1[2] + line1[0] * line1[3]) / sqrtf(pow(line1[0], 2) + pow(line1[1], 2));
		f = scale_factor * abs(line2[1] * center.x - line2[0] * center.y - line2[1] * line2[2] + line2[0] * line2[3]) / sqrtf(pow(line2[0], 2) + pow(line2[1], 2));
	}
	alpha = 180 / 3.1415926 * acos((line1[0] * line2[0] + line1[1] * line2[1]) / (sqrt(pow(line1[0], 2) + pow(line1[1], 2)) * sqrt(pow(line2[0], 2) + pow(line2[1], 2))));
	return 1;
}
