#ifndef MYVECTOR_H_
#define MYVECTOR_H_

#include <string>

class PointCloud {
public:
    // Constructor methods
    PointCloud(void);

    // Destructor method
    ~PointCloud(void);
    void plot(void);
    void read(std::string);


private:

    unsigned int vsize;
    unsigned int capacity;
};

#endif /* MYVECTOR_H_ */
