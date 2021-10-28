#ifndef MYVECTOR_H_
#define MYVECTOR_H_

#include <string>
#include <vector>
#include<opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp>

class PointCloud {
public:
    // Constructor methods
    PointCloud(int size);

    // Destructor method
    ~PointCloud(void);
    void write(std::string);
    void read(std::string);
    void fill_randomly(void);
    void plot(void);
    std::vector<double> get_x();
    std::vector<double> get_y();
    std::vector<double> get_z();
    int vsize();


private:
    int size;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;


};

class Img {
public:
    // Constructor methods
    Img(void);
    Img(cv::Mat);

    // Destructor method
    //~Img(void);
    void write(std::string);
    void read(std::string);
    cv::Mat to_Mat(void);
    void from_Mat(cv::Mat);
    void show(void);
    int get_rows(void);
    int get_cols(void);
    std::vector<uchar> get_img();

private:
    int rows;
    int cols;
    std::vector<uchar> img;


};

cv::Mat filter_image(cv::Mat img, int n, int m);
cv::Mat pad_image(cv::Mat img, int pad_width, int pad_val);
double* get_filter(std::string filter);

#endif /* MYVECTOR_H_ */
