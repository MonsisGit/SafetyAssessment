// SafetyAssessment.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include<opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "Utilities.h"

int main()
{
    /*std::string path = "Resource/test.txt";
    PointCloud pointcloud(100000);
    pointcloud.fill_randomly();
    pointcloud.read(path);*/

    Img img;
    img.read("Resource/test.png");
    img.pad(20, 20, 100);
    std::vector<uchar> vec_img = img.get();

    Img img2(vec_img, img.get_rows(), img.get_cols());
    img2.show();
    img2.write("Resource/test_2.png");
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
