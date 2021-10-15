#include "Utilities.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <random>
#include <fstream>

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


