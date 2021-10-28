#include "Utilities.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <random>
#include <fstream>
#include<opencv2/opencv.hpp> 
#include <opencv2/highgui/highgui.hpp>

#define MAX_RAND 100000

// The constructor
PointCloud::PointCloud(int size) {
    this->size = size;
    for (int i = 0; i < size; i++) {
        this->x.push_back(0);
        this->y.push_back(0);
        this->z.push_back(0);
    }
    return;
}


// Destructor method
PointCloud::~PointCloud(void) {
    return;
}

void PointCloud::read(std::string path) {
    std::ifstream inFile(path);
    std::string l,token;

    int cnt_length = 0;
    while (getline(inFile, l)) {
        size_t pos = 0;
        int cnt = 0;
        while ((pos = l.find(";")) != std::string::npos) {
            token = l.substr(0, pos);
            l.erase(0, pos + 1);

            if (cnt==0) x.push_back(std::stod(token));
            if (cnt == 1) y.push_back(std::stod(token));
            if (cnt == 2) z.push_back(std::stod(token));

            cnt++;

        }
        cnt_length++;
    }

    if (cnt_length != this->size) {
        std::cout << "------------------------------------" << std::endl;
        std::cout << "Size of " << path << " is not equal of initialized vector length! (" << cnt_length << " vs "<< this->size << ")"<<std::endl;
    }



}

void PointCloud::write(std::string path) {

    std::ofstream outFile(path);
    for (int i = 0;i<x.size();i++) {
        outFile << x[i] << ";" << y[i] << ";" << z[i] << ";" << "\n";
    }
    std::cout << "File written to: " << path << std::endl;
}

void PointCloud::fill_randomly(void) {
    std::random_device rnd_device;
    // Specify the engine and distribution.
    std::mt19937 mersenne_engine{ rnd_device() };  // Generates random integers
    std::uniform_int_distribution<int> dist{ 1, 100 };

    auto gen = [&dist, &mersenne_engine]() {
        return dist(mersenne_engine);
    };

    generate(begin(x), end(x), gen);
    generate(begin(y), end(y), gen);
    generate(begin(z), end(z), gen);

};

std::vector<double> PointCloud::get_x() {
    return this->x;
}

std::vector<double> PointCloud::get_y() {
    return this->y;
}

std::vector<double> PointCloud::get_z() {
    return this->z;
}

int PointCloud::vsize() {
    return this->size;
}

void PointCloud::plot(void) {
    //TODO
}



cv::Mat contour_search(cv::Mat img) {
    unsigned char* pic = new unsigned char[img.rows * img.cols];
    cv::Mat c_img = cv::Mat(img.rows, img.cols, img.type(), cv::Scalar(0));
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            pic[i * img.cols + j] = img.at<uchar>(i, j);
        }
    }

    int B = img.step;
    int rimx[MAX_RAND], rimy[MAX_RAND];
    int newpos, local_tresh = 0, draw_type, pos;
    draw_type = 0;
    for (int i = 0; i < img.rows * img.cols; i++) {
        if (pic[i] == 255) {
            pos = i;
            break;
        }
    }
    newpos = pos;
    int count = 0;
    while (newpos >= 0L && newpos < img.rows * img.cols)
    {
        rimx[count] = newpos % B; // save current position in list
        rimy[count] = newpos / B;
        count++;
        draw_type = (draw_type + 6) % 8; // Select next search direction
        switch (draw_type)
        {
        case 0: if (pic[newpos + 1] > local_tresh) { newpos += 1; draw_type = 0; break; }
        case 1: if (pic[newpos + B + 1] > local_tresh) { newpos += B + 1; draw_type = 1; break; }
        case 2: if (pic[newpos + B] > local_tresh) { newpos += B; draw_type = 2; break; }
        case 3: if (pic[newpos + B - 1] > local_tresh) { newpos += B - 1; draw_type = 3; break; }
        case 4: if (pic[newpos - 1] > local_tresh) { newpos -= 1; draw_type = 4; break; }
        case 5: if (pic[newpos - B - 1] > local_tresh) { newpos -= B + 1; draw_type = 5; break; }
        case 6: if (pic[newpos - B] > local_tresh) { newpos -= B; draw_type = 6; break; }
        case 7: if (pic[newpos - B + 1] > local_tresh) { newpos -= B - 1; draw_type = 7; break; }
        case 8: if (pic[newpos + 1] > local_tresh) { newpos += 1; draw_type = 0; break; }
        case 9: if (pic[newpos + B + 1] > local_tresh) { newpos += B + 1; draw_type = 1; break; }
        case 10: if (pic[newpos + B] > local_tresh) { newpos += B; draw_type = 2; break; }
        case 11: if (pic[newpos + B - 1] > local_tresh) {
            newpos += B -
                1; draw_type = 3; break;
        }
        case 12: if (pic[newpos - 1] > local_tresh) { newpos -= 1; draw_type = 4; break; }
        case 13: if (pic[newpos - B - 1] > local_tresh) {
            newpos -= B + 1; draw_type = 5; break;
        }
        case 14: if (pic[newpos - B] > local_tresh) { newpos -= B; draw_type = 6; break; }
        }
        if (newpos == pos) {
            break;
        }
        if (count >= MAX_RAND)
            break;
    }
    for (int i = 0; i < count; i++) {
        c_img.at<uchar>(rimy[i], rimx[i]) = 255;
    }

    delete[] pic;
    return c_img;
}



Img::Img(cv::Mat image) {
    std::string ty = this->type2str(image.type());
    if (ty != "8UC1") {
        std::cout << "Converting img to greyscale of type 8UC1, was of type: " << ty.c_str() << " before" << std::endl;
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }

    if (image.isContinuous()) {
        this->img.assign(image.data, image.data + image.total() * image.channels());
    }
    else {
        std::cerr << "Matrix not continuous!";
    }
    this->rows = image.rows;
    this->cols = image.cols;
    return;
}

Img::Img(void) {
    this->rows = 0;
    this->cols = 0;
    return;
}

Img::Img(std::vector<uchar> vec, int rows=0, int cols=0) {
    this->img = vec;
    this->rows = rows;
    this->cols = cols;
    return;
}

void Img::show(void) {
    cv::Mat image = this->to_Mat();
    cv::imshow("Image", image);
    cv::waitKey(0);
}

cv::Mat Img::to_Mat(void) {
    cv::Mat img = cv::Mat(this->rows, this->cols, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < this->cols; i++) {
        for (int j = 0; j < this->rows; j++) {
            img.at<uchar>(j, i) = this->img[j * cols + i];
        }
    }
    return img;
}


void Img::from_Mat(cv::Mat image) {
    std::string ty = this->type2str(image.type());
    if (ty != "8UC1") {
        std::cout << "Converting img to greyscale of type 8UC1, was of type: " << ty.c_str() << " before" << std::endl;
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }
    if (image.isContinuous()) {
        this->img.assign(image.data, image.data + image.total() * image.channels());
    }
    else {
        std::cerr << "Matrix not continuous!";
    }
    this->rows = image.rows;
    this->cols = image.cols;
}

inline bool path_exists(const std::string& name) {
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

void Img::read(std::string path) {
    if (!path_exists(path)) {
        std::cerr << path << " does not exist!" << std::endl;
    }
    cv::Mat img = cv::imread(path);
    this->from_Mat(img);
}

void Img::write(std::string path) {
    cv::Mat img = this->to_Mat();
    cv::imwrite(path,img);
    std::cout << "Wrote Image to: " << path << std::endl;
}

int Img::get_rows(void) {
    return this->rows;
}

int Img::get_cols(void) {
    return this->cols;
}

std::vector<uchar> Img::get() {
    return this->img;
};

void Img::conv2d(double filter[] ,int n, int m) {
    cv::Mat img = this->to_Mat();
    cv::Mat img_filtered = img.clone();
    //float scaling = 1 / pow(n + m + 1, 2);

    for (int i = n; i < img.cols - n; i++) {
        for (int j = m; j < img.rows - m; j++) {
            int values = 0;
            int position = 0;
            for (int p = i - n; p < i + 2 * n; p++) {
                for (int k = j - m; k < j + 2 * m; j++) {
                    values += filter[position] * img.at<uchar>(k, p);
                    position++;
                }
            }
            img_filtered.at<uchar>(j, i) = values;
        }
    }
    this->from_Mat(img_filtered);
};

double* Img::get_filter(std::string filter) {
    //Laplace-Gaussian Filter
    if (filter == "laplace-gaussian") {
        double f[] = { 0,		0,		1,		2,		2,		2,		1,		0,		0,
                            0,		1,		5,		10,		12,		10,		5,		1,		0,
                            1,		5,		15,		19,		16,		19,		15,		5,		1,
                            2,		10,		19,	   -19,	   -64,	   -19,		19,		10,		2,
                            2,		12,		16,	   -64,	   -148,   -64,		16,		12,		2,
                            2,		10,		19,	   -19,	   -64,	   -19,		19,		10,		2,
                            1,		5,		15,		19,		16,		19,		15,		5,		1,
                            0,		1,		5,		10,		12,		10,		5,		1,		0,
                            0,		0,		1,		2,		2,		2,		1,		0,		0 };
        return f;
    }
    //Laplace Filter
    else if (filter == "laplace") {
        double f[] = { 0,		1,		0,
                    1,	   -4,		1,
                    0,		1,		0 };
        return f;
    }
    //horizontal gradient filter, (sobel filter)
    else if (filter == "h_gradient") {
        double f[] = { 1,		0,		-1,
                    2,	   0,	-2,
                    1,		0,		-1 };
        return f;
    }

    //vertical gradient filter, (sobel filter)
    else if (filter == "v_gradient") {
        double f[] = { 1,		2,		1,
                    0,	   0,	0,
                    1,		-2,		-1 };
        return f;

    }
    else {
        std::cerr << "No such filter" << std::endl;
    }
};

void Img::pad(int pad_width, int pad_height, int pad_value) {
    cv::Mat img = to_Mat();
    cv::Mat img_padded = cv::Mat(img.rows + pad_width * 2, img.cols + pad_width * 2,
        img.type(), cv::Scalar(pad_value));
    for (int i = pad_width; i < img.cols + pad_width; i++) {
        for (int j = pad_width; j < img.rows + pad_width; j++) {
            img_padded.at<uchar>(j, i) = img.at<uchar>(j - pad_width, i - pad_width);
        }
    }
    this->from_Mat(img_padded);
};


std::string Img::type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}
