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

cv::Mat filter_image(cv::Mat img, int n, int m);
cv::Mat create_histogramm(cv::Mat image, bool greyscale);
cv::Mat threshold_image(cv::Mat img, int thresh);
cv::Mat pad_image(cv::Mat img, int pad_width = 1, int pad_val = 0);
cv::Mat contour_search(cv::Mat img);
cv::Mat find_correspondence(cv::Mat img_1, cv::Mat img_2);
double* get_filter(std::string filter);

#endif /* MYVECTOR_H_ */
