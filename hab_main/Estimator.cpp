#include "MatrixMath.h"
#include "Estimator.h"
#include "Constants.h"

const float identity[3][3] = {{1,0,0},
                                {0,1,0},
                                {0,0,1}}; 
                    
float g_i[3][1] = {{0},
                         {0},
                         {-1}};

float Cbi_hat[3][3] = {{1,0,0},
                        {0,1,0},
                        {0,0,1}};
                                                
float b_i[3][1]; //Magnetic reference frame

float k = -5.0;  //Innovation gain
float kg = 1.0;  //Gravitational vector gain
float kb = 0.25; //Magnetic vector gain

float normA[3][1];
float normM[3][1];
float b_b[3][1];
float g_b[3][1];
float r_g[3][1];
float r_b[3][1];
float r[3][1];
float cross_g[3][3];
float cross_b[3][3];
float omega_hat[3][1];
float omega_hat_mag;
float omega_hat_mag_r;
float Ak_1[3][3];
float Ak_2[3][3];
float Ak_3[3][3];
float Ak[3][3];
float Cbi_hat_new[3][3];
float cross_omega_hat[3][3];

float q[4][1];
float mag_q;
float q_t[1][4];

boolean firstRunEstimate = true;      

float determinant(float c[3][3]){
    return c[0][0]*(c[1][1]*c[2][2] - c[1][2]*c[2][1]) - c[0][1]*(c[1][0]*c[2][2] - c[1][2]*c[2][0]) + c[0][2]*(c[1][0]*c[2][1] - c[1][1]*c[2][0]);  
}

void normalize(int input[3][1], float normVec[3][1]){
    //Find the vector norm
    float norm=0;
    for(int i=0;i<3;i++){
         norm += (float)input[i][0] * (float)input[i][0];
    }
    norm = sqrt(norm);
    
    //Normalize the vector
    for(int i=0;i<3;i++){
        normVec[i][0] = (float) input[i][0] / norm;
    }
}

void cross(float vector[3][1], float output[3][3]){
    output[0][0] = 0;
    output[0][1] = -vector[2][0];
    output[0][2] = vector[1][0];
    
    output[1][0] = vector[2][0];
    output[1][1] = 0;
    output[1][2] = -vector[0][0];
    
    output[2][0] = -vector[1][0];
    output[2][1] = vector[0][0];
    output[2][2] = 0;
}

void toQuaternion(float c[3][3], float q[4][1]){
    float trace=0;
    float nu;
    
    for(int i=0;i<3;i++){
      trace += c[i][i];  
    }
    
    nu = sqrt(trace+1)/2;
    
    
    q[0][0] = (c[1][2] - c[2][1])/(4*nu);
    q[1][0] = (c[2][0] - c[0][2])/(4*nu);
    q[2][0] = (c[0][1] - c[1][0])/(4*nu);
    q[3][0] = nu;
}

void toRotation(float q[4][1], float c[3][3]){
    float e[3][1];
    float e_t[1][3];
    float e_m[3][3];
    float e_cr[3][3];
    float e_add[3][3];
    float n_3x3[3][3];
    
    for(int i=0;i<3;i++){
        e[i][0] = q[i][0];
    }
    
    Matrix.Transpose((float*)e, 3, 1, (float*)e_t);
    Matrix.Multiply((float*)e, (float*)e_t, 3, 1, 3, (float*)e_m);
    Matrix.Scale((float*)e_m, 3, 3, 2);
    
    cross(e, e_cr);
    Matrix.Scale((float*) e_cr, 3, 3, -2*q[3][0]);
    Matrix.Add((float*)e_m, (float*)e_cr, 3, 3, (float*)e_add);
    
    Matrix.Copy((float*)identity, 3, 3, (float*)n_3x3);
    Matrix.Scale((float*)n_3x3, 3, 3, 2*q[3][0]*q[3][0]-1);
    
    Matrix.Add((float*)n_3x3, (float*)e_add, 3, 3, (float*)c);
}

void estimate(int a[3][1], float g[3][1], int m[3][1], float e[3][3]){
    //Normalize each column vector
    normalize(a, normA);
    normalize(m, normM);

    //Initialize the initial global frames
    if(firstRunEstimate){
        for(int i=0;i<3;i++){
            b_i[i][0] = normM[i][0];
        }
        firstRunEstimate = !firstRunEstimate;
    }
    
    Matrix.Multiply((float*)Cbi_hat, (float*)b_i, 3, 3, 1, (float*)b_b);
    Matrix.Multiply((float*)Cbi_hat, (float*)g_i, 3, 3, 1, (float*)g_b);
    
    cross(b_b, cross_b);
    cross(g_b, cross_g);
    
    //Calculate the innovation term
    Matrix.Multiply((float*)cross_g, (float*)normA, 3, 3, 1, (float*)r_g);
    Matrix.Scale((float*)r_g, 3, 3, (float)kg);
    
    Matrix.Multiply((float*)cross_b, (float*)normM, 3, 3, 1, (float*)r_b);
    Matrix.Scale((float*)r_b, 3, 3, (float)kb);
    
    Matrix.Add((float*)r_b, (float*)r_g, 3, 1, (float*)r);
    Matrix.Scale((float*)r, 3, 1, k);
    
    //Prediction step
    Matrix.Add((float*)g, (float*)r, 3, 1, (float*)omega_hat);
    
    for(int i=0;i<3;i++){
        omega_hat_mag += omega_hat[i][0]*omega_hat[i][0];  
    }
    omega_hat_mag = sqrt(omega_hat_mag);
    omega_hat_mag_r = 1.0/omega_hat_mag;
    
    cross(omega_hat, cross_omega_hat);
    
    Matrix.Multiply((float*)cross_omega_hat, (float*)cross_omega_hat, 
                     3, 3, 3, (float*)Ak_1);
    Matrix.Scale((float*)Ak_1, 3, 3, 
                 (1-cos(omega_hat_mag*1.0/EST_FREQ))*pow(omega_hat_mag_r,2));

    Matrix.Copy((float*)cross_omega_hat, 3, 3, (float*)Ak_2);
    Matrix.Scale((float*)Ak_2, 3, 3, 
                 sin(omega_hat_mag*1.0/EST_FREQ)*omega_hat_mag_r);
    
    Matrix.Add((float*)Ak_1, (float*)Ak_2, 3, 3, (float*)Ak_3);
    Matrix.Subtract((float*)identity, (float*)Ak_3, 3, 3, (float*)Ak);
    
    //Normalize Ak matrix
    toQuaternion(Ak, q);
    mag_q = sqrt(q[0][0]*q[0][0] + q[1][0]*q[1][0] 
                + q[2][0]*q[2][0] + q[3][0]*q[3][0]);
    for(int i=0;i<4;i++){
        q[i][0] = q[i][0]/mag_q;
    }  
    toRotation(q, Ak); 
    
    //Correction Step  
    Matrix.Multiply((float*)Ak, (float*)Cbi_hat, 3, 3, 3, (float*)Cbi_hat_new);
    Matrix.Copy((float*)Cbi_hat_new, 3, 3, (float*)Cbi_hat);
    
    //Normalize Cbi_hat matrix
    toQuaternion(Cbi_hat, q);
    mag_q = sqrt(q[0][0]*q[0][0] + q[1][0]*q[1][0] 
                + q[2][0]*q[2][0] + q[3][0]*q[3][0]);
    for(int i=0;i<4;i++){
        q[i][0] = q[i][0]/mag_q;
    }  
    toRotation(q, Cbi_hat); 
    
    float det = determinant(Cbi_hat);
    
    if((int)(det*1000) < 1){
        Matrix.Copy((float*)identity, 3, 3, (float*)Cbi_hat);
        Serial.println("!SYS: Reset Cbi_hat matrix");
    }
    
    //Serial.println(det,10);
    
    /*
    Matrix.Print((float*)Cbi_hat,3,3,"est");
    */
    //Update the output
    Matrix.Copy((float*)Cbi_hat, 3, 3, (float*)e);
}

void triad(int a[3][1], int m[3][1], float e[3][3]){
    float norm;

    float cross_g[3][3];
    float cross_v1_i[3][3];
    float v2_i[3][1];
    float v3_i[3][1];
    
    float cross_a[3][3];
    float cross_v1_b[3][3];
    float v2_b[3][1];
    float v3_b[3][1];
    
    float vb_3x3[3][3];
    float vi_3x3[3][3];
    float vi_3x3_t[3][3];
    
    //Normalize each column vector
    normalize(a, normA);
    normalize(m, normM);
    
    //Initialize the initial global frames
    if(firstRunEstimate){
        for(int i=0;i<3;i++){
            b_i[i][0] = normM[i][0];
        }
        firstRunEstimate = !firstRunEstimate;
    }
    
    //Inertial Frame
    cross(g_i, cross_g);
    Matrix.Multiply((float*)cross_g, (float*)b_i, 3, 3, 1, (float*)v2_i);
    
    norm = 0;
    for(int i=0;i<3;i++){
        norm += v2_i[i][0] * v2_i[i][0];
    }
    norm = sqrt(norm);
    for(int i=0;i<3;i++){
        v2_i[i][0] = v2_i[i][0]/norm;
    }
    
    Matrix.Multiply((float*)cross_g, (float*)v2_i, 3, 3, 1, (float*)v3_i);
    
    //Body Frame
    cross(normA, cross_a);
    Matrix.Multiply((float*)cross_a, (float*)normM, 3, 3, 1, (float*)v2_b);
    
    norm = 0;
    for(int i=0;i<3;i++){
        norm += v2_b[i][0] * v2_b[i][0];
    }
    norm = sqrt(norm);
    for(int i=0;i<3;i++){
        v2_b[i][0] = v2_b[i][0]/norm;
    }
    
    Matrix.Multiply((float*)cross_a, (float*)v2_b, 3, 3, 1, (float*)v3_b);
    
    //C = v_b * v_i^T
    for(int i=0;i<3;i++){
      vb_3x3[i][0] = normA[i][0];
    }
    for(int i=0;i<3;i++){
      vb_3x3[i][1] = v2_b[i][0];
    }
    for(int i=0;i<3;i++){
      vb_3x3[i][2] = v3_b[i][0];
    }
    
    for(int i=0;i<3;i++){
      vi_3x3[i][0] = g_i[i][0];
    }
    for(int i=0;i<3;i++){
      vi_3x3[i][1] = v2_i[i][0];
    }
    for(int i=0;i<3;i++){
      vi_3x3[i][2] = v3_i[i][0];
    }
    
    Matrix.Transpose((float*)vi_3x3, 3, 3, (float*)vi_3x3_t);
    Matrix.Multiply((float*)vb_3x3, (float*)vi_3x3_t, 3, 3, 3, (float*)e);
}
