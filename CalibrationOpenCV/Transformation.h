#pragma once
#include <vector>

/**
* struct Transformation
* 
* This struct contains a matrix and an array of doubles representing the 
* transformation of a camera to another with the matrix of rotation and 
* the vector of translation (extrinsinc parameters)
*/
struct Transformation
{
    // Rotation matrix 
    double R[3][3];
    // Translation vector
    double t[3];

    // Construct an identity transformation.
    Transformation() {
        setEyeRotMatrix();
        t[0], t[1], t[2] = 0;
    }

    //TODO : create a function to compose the transformations ? 

    void setEyeRotMatrix() {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R[i][j] = 0;
            }
        }
        R[0][0] = 1;
        R[1][1] = 1;
        R[2][2] = 1;
    }

    void setRotationMatrix(std::vector<double> matrix) {
        if (matrix.size() == 9) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    R[i][j] = matrix[i * 3 + j];
                }
            }
        }
        else {
            std::cerr << "Error the size of the matrix isn't correct : need to be of size 9" << std::endl;
        }
    }

    void setRotationMatrix(double matrix[9]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R[i][j] = matrix[i * 3 + j];
            }
        }
    }

    void setRotationMatrix(double matrix[3][3]) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R[i][j] = matrix[i][j];
            }
        }
    }
};