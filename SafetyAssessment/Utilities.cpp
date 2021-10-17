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

cv::Mat filter_image(cv::Mat img, int n, int m, double filter[], double scaling=0) {
    // n: size of filter in horizontal direction
    // m: size of filter in vertical direction

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
            img_filtered.at<uchar>(j, i) = scaling * values;
        }
    }
    return img_filtered;
}

cv::Mat create_histogramm(cv::Mat image, bool greyscale) {
    //https://docs.opencv.org/3.4/d8/dbc/tutorial_histogram_calculation.html

    while (true) {
        if (greyscale == false) {
            std::vector<cv::Mat> bgr_planes;
            split(image, bgr_planes);
            int histSize = 256;
            float range[] = { 0, 256 }; //the upper boundary is exclusive
            const float* histRange[] = { range };
            bool uniform = true, accumulate = false;
            cv::Mat b_hist, g_hist, r_hist;
            calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, histRange, uniform, accumulate);
            calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);
            calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, histRange, uniform, accumulate);
            int hist_w = 512, hist_h = 400;
            int bin_w = cvRound((double)hist_w / histSize);
            cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
            normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
            normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
            normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
            for (int i = 1; i < histSize; i++)
            {
                line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
                    cv::Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
                    cv::Scalar(255, 0, 0), 2, 8, 0);
                line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
                    cv::Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
                    cv::Scalar(0, 255, 0), 2, 8, 0);
                line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
                    cv::Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
                    cv::Scalar(0, 0, 255), 2, 8, 0);
            }
            return histImage;
        }
        else {
            cv::Mat g_plane;
            cvtColor(image, g_plane, cv::COLOR_BGR2GRAY);
            //split(image, bgr_planes);
            int histSize = 256;
            float range[] = { 0, 256 }; //the upper boundary is exclusive
            const float* histRange[] = { range };
            bool uniform = true, accumulate = false;
            cv::Mat g_hist;
            calcHist(&g_plane, 1, 0, cv::Mat(), g_hist, 1, &histSize, histRange, uniform, accumulate);

            int hist_w = 512, hist_h = 400;
            int bin_w = cvRound((double)hist_w / histSize);
            cv::Mat histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
            normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());
            for (int i = 1; i < histSize; i++)
            {
                line(histImage, cv::Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
                    cv::Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
                    cv::Scalar(255, 0, 0), 2, 8, 0);
            }
            return histImage;
        }
    }
}

cv::Mat threshold_image(cv::Mat img, int thresh) {
    for (int i = 0; i < img.cols; i++) {
        for (int j = 0; j < img.rows; j++) {
            if (img.at<uchar>(j, i) > thresh) {
                img.at<uchar>(j, i) = 0;
            }
            else {
                img.at<uchar>(j, i) = 255;
            }
        }
    }
    return img;
}

std::string type2str(int type) {
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

cv::Mat pad_image(cv::Mat img, int pad_width = 1, int pad_val = 0) {

    std::string ty = type2str(img.type());
    if (ty != "8UC1") {
        std::cout << "Converting img to greyscale of type 8UC1, was of type: " << ty.c_str() << " before" << std::endl;
        cvtColor(img, img, cv::COLOR_BGR2GRAY);

    }
    cv::Mat img_padded = cv::Mat(img.rows + pad_width * 2, img.cols + pad_width * 2,
        img.type(), cv::Scalar(pad_val));
    for (int i = pad_width; i < img.cols + pad_width; i++) {
        for (int j = pad_width; j < img.rows + pad_width; j++) {
            img_padded.at<uchar>(j, i) = img.at<uchar>(j - pad_width, i - pad_width);
        }
    }
    imshow("pad", img_padded);
    return img_padded;
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

int add_filter(cv::Mat img, int i, int j, int n, int m) {
    float val = 0;
    for (int p = i - n; p <= i + n; p++) {
        for (int k = j - m; k <= j + m; k++) {
            val += img.at<uchar>(k, p);
        }
    }
    return val;
}

cv::Mat find_correspondence(cv::Mat img_1, cv::Mat img_2) {

    //BROKEN!!

    cv::Mat img_1_cor = cv::Mat(img_1.rows, img_1.cols, img_1.type(), cv::Scalar(0));
    cv::Mat img_2_cor = cv::Mat(img_1.rows, img_1.cols, img_1.type(), cv::Scalar(0));
    cv::Mat img_contours, img_1_binary;
    cvtColor(img_1, img_1, cv::COLOR_BGR2GRAY);

    img_contours = threshold_image(img_1, 200);
    //img_contours = contour_search(img_1_binary);
    //imshow("cont", img_contours);
    //waitKey(0);

    int n = 6, m = 6, cnt = 0;

    for (int i = n; i < img_1.cols - n; i++) {
        for (int j = m; j < img_1.rows - m; j++) {
            if (img_contours.at<uchar>(j, i) > 0) {
                img_1_cor.at<uchar>(j, i) = add_filter(img_1, i, j, n, m);
            }
            img_2_cor.at<uchar>(j, i) = add_filter(img_2, i, j, n, m);
        }
    }
    int l2, tmp, k_tmp, l_tmp;
    std::vector<int> posx, posy, l2_diff;

    for (int i = n; i <= img_1.cols - n; i++) {
        for (int j = m; j <= img_1.rows - m; j++) {

            if (img_1_cor.at<uchar>(j, i) != 0) {
                posx.push_back(i);
                posy.push_back(j);

                l2 = 10e8;
                l_tmp = 0; k_tmp = 0;
                for (int k = n; k <= img_1.cols - n; k++) {
                    for (int l = m; l <= img_1.rows - m; l++) {
                        tmp = abs(img_1_cor.at<uchar>(j, i) - img_2_cor.at<uchar>(l, k));
                        if (tmp < l2) {
                            l2 = tmp;
                            l_tmp = l;
                            k_tmp = k;

                        }
                    }
                }
                l2_diff.push_back(tmp);
                posx.push_back(k_tmp);
                posy.push_back(l_tmp);
                std::cout << "i,j dist: " << i << " " << j << "L2 dist : " << l2_diff[l2_diff.size() - 1] << "  With pos x, y : " << posx[posx.size() - 1] << " " << posy[posy.size() - 1] << std::endl;
            }
        }
    }
    return img_1;
};

double* get_filter(std::string filter) {
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


};