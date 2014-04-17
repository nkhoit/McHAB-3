#ifndef Estimator_h
#define Estimator_h

void normalize(int input[3][1], float normVec[3][1]);
void cross(float vector[3][1], float output[3][3]);
//void estimate(int a[3][1], float g[3][1], int m[3][1], float mag_i[3][1], float e[3][3]);
void estimate(int a[3][1], float g[3][1], int m[3][1], float e[3][3]);
void triad(int a[3][1], int m[3][1], float e[3][3]); 

#endif

