#include <opencv.hpp>
#include <iostream>
#include <numeric>
#include "GrabImage.h"
#include "calibration.h"
#include "size_detection.h"
using namespace cv;
using namespace std;
int main()
{
	string calib_save_path;
	string img_save_path1, img_save_path2, img_save_path3, img_save_path4;
	calib_save_path = "E:\\pg_one\\Technology_of_surveying\\project_design\\calibration_image\\";
	img_save_path1 = "E:\\pg_one\\Technology_of_surveying\\project_design\\image1\\";
	img_save_path2 = "E:\\pg_one\\Technology_of_surveying\\project_design\\image2\\";
	img_save_path3 = "E:\\pg_one\\Technology_of_surveying\\project_design\\image3\\";
	img_save_path4 = "E:\\pg_one\\Technology_of_surveying\\project_design\\image4\\";

	calib_grab_image(calib_save_path);

	cout << "第一步，抓取棋盘格图像已完成！" << endl;

	//calib_results_path 标定结果保存目录
	//chessbord_path 棋盘格图片目录
	string calib_results_path = "E:\\pg_one\\Technology_of_surveying\\project_design\\calibration_image\\calibration_results.txt";
	string chessbord_path = "E:\\pg_one\\Technology_of_surveying\\project_design\\calibration_image";
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));        // 摄像机内参数矩阵
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));          // 摄像机的5个畸变系数：k1,k2,p1,p2,k3
	m_calib(calib_results_path, chessbord_path, cameraMatrix, distCoeffs);
	cout << "相机内参数矩阵：" << endl;
	cout << cameraMatrix << endl << endl;
	cout << "畸变系数：\n";
	cout << distCoeffs << endl << endl;

	cout << "第二步，相机标定已完成！" << endl;

	//system("pause");
	calib_grab_image(img_save_path1);
	//system("pause");
	calib_grab_image(img_save_path2);
	//system("pause");
	calib_grab_image(img_save_path3);
	//system("pause");
	calib_grab_image(img_save_path4);

	cout << "第三步，抓取工件图像已完成！" << endl;

	double at, bt, dt, et, ft, alphat;
	vector<double> a, b, d, e, f, alpha;
	double scale_factor = 1.0 / 79.3;
	Mat src;
	char path[100];
	char resultspath[100];
	char filepath[100];
	char resultsname[] = "measurement_results.txt";
	char filename[100];
	for (int j = 0; j < 4; j++) {
		sprintf_s(path, "E:\\pg_one\\Technology_of_surveying\\project_design\\image%d\\", (j + 1));
		strcpy_s(resultspath, path);
		strcat_s(resultspath, resultsname);
		ofstream fout(resultspath);
		fout << "a\t" << "b\t" << "d\t" << "e\t" << "f\t" << "alpha\t" << "\n";
		for (int i = 0; i < 10; i++)
		{
			sprintf_s(filename, "%d.bmp", (i + 1));
			strcpy_s(filepath, path);
			strcat_s(filepath, filename);
			src = imread(filepath);

			//矫正畸变
			Mat newsrc = src.clone();
			undistort(src, newsrc, cameraMatrix, distCoeffs);

			if (size_detection(newsrc, at, bt, dt, et, ft, alphat, scale_factor)) {
				a.push_back(at); b.push_back(bt); d.push_back(dt); e.push_back(et); f.push_back(ft); alpha.push_back(alphat);
				printf_s("a = %.2f, b = %.2f, d = %.2f, e = %.2f, f = %.2f, alpha = %.2f\n", a[i], b[i], d[i], e[i], f[i], alpha[i]);
				fout << at << "\t" << bt << "\t" << dt << "\t" << et << "\t" << ft << "\t" << alphat << "\t" << "\n";
			}
			else {
				cout << "检测错误，重新检测!" << endl;
			}
		}
		double a_average = accumulate(a.begin(), a.end(), 0.0) / a.size();
		double b_average = accumulate(b.begin(), b.end(), 0.0) / b.size();
		double d_average = accumulate(d.begin(), d.end(), 0.0) / d.size();
		double e_average = accumulate(e.begin(), e.end(), 0.0) / e.size();
		double f_average = accumulate(f.begin(), f.end(), 0.0) / f.size();
		double alpha_average = accumulate(alpha.begin(), alpha.end(), 0.0) / alpha.size();
		fout << "\nAverage results:\n";
		//printf_s("\na = %.2f, b = %.2f, d = %.2f, e = %.2f, f = %.2f, alpha = %.2f\n", a_average, b_average, d_average, e_average, f_average, alpha_average);
		fout << a_average << "\t" << b_average << "\t" << d_average << "\t" << e_average << "\t" << f_average << "\t" << alpha_average << "\t" << "\n";
		fout << "\nStandrd errors:\n";
		//printf_s("\na = %.2f, b = %.2f, d = %.2f, e = %.2f, f = %.2f, alpha = %.2f\n", a_average, b_average, d_average, e_average, f_average, alpha_average);
		fout << (a_average-15.0) << "\t" << (b_average-20.0) << "\t" << (d_average-6.0) << "\t" << (e_average-5.0) << "\t" << (f_average-5.0) << "\t" << (alpha_average-90.0) << "\t" << "\n";
		a.clear();
		b.clear();
		e.clear();
		d.clear();
		f.clear();
		alpha.clear();
	}

	cout << "第四步，工件尺寸测量已完成，测量报告已保存至相应目录！" << endl;
	system("pause");
	return 1;
}