#ifndef MYVECTOR_H_
#define MYVECTOR_H_

#include <string>
#include <vector>

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

#endif /* MYVECTOR_H_ */
