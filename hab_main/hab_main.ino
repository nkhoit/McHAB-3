#include <FreeRTOS_ARM.h>
#include <Wire.h>
#include <L3G.h>
#include <LSM303.h>
#include <TinyGPS.h>

#include "MatrixMath.h"
#include "Estimator.h"
#include "Constants.h"
#include "Adafruit_BMP085.h"

#undef F
#define F(str) str

//FreeRTOS task declarations
xTaskHandle est_task;
xTaskHandle torque_control_task;
xTaskHandle current_control_task;
xTaskHandle bmp_task;
xTaskHandle gps_task;
xTaskHandle buzzer_task;

//Mutex handles for serial and I2C bus.
xSemaphoreHandle i2cGateKeeper;
xSemaphoreHandle serialGateKeeper;

// Sensor Definitions
LSM303 compass;
L3G gyro;
Adafruit_BMP085 bmp;
TinyGPS gps;

//Filtering variables
const int filtLength = 10;
float filtBuffer[3][20] = {{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
int filtIndex = 0;
float filtered[3][1] = {{0},
                        {0},
                        {0}};

// IMU Rotation to body frame
/*
int rotation[3][3] = {{0, 0, 1},
                      {0, 1, 0},
                      {-1, 0, 0}};
*/

int rotation[3][3] = {{0, 1, 0},
                      {0, 0, 1},
                      {1, 0, 0}};

/*
int rotation[3][3] = {{1, 0, 0},
                      {0, 1, 0},
                      {0, 0, 1}};
*/
// RAW IMU Measurements
int rawAccel[3][1];
int rawGyro[3][1];
int rawMag[3][1];
    
// Rotated IMU Measurements
int rotAccel[3][1];
int rotGyro[3][1];
int rotMag[3][1];

// Inertia mag field
float mag_i[3][1] = {{0},
                     {0},
                     {0}};

// Estimation variables
float omega_dot[3][1] = {{0},
                         {0},
                         {0}};
float est[3][3] = {{0},
                   {0},
                   {0}};

//Euler angles
float pitch=0;
float roll=0;
float yaw=0;  

// Control Law variables
float voltCmd[2] = {0,0};
float current=0;
float rpm=0;
float spdCmd;
float current_cmd;
float tau;

// Aux Sensor Outputs
float batteryVoltage;
float payloadAltitude=0;
float bmpAltitude = 0;
float altitude = 236.5;
float highestAltitude = 0;

// GPS Outputs
float flat = 45.709205;
float flon = -73.1862883;
unsigned long age;
int satellites;
int hdop;
float gpsAltitude = 236.5;
int altitudeCounter=0;
unsigned long utc_time, date;

// Flight System Parameters
boolean missionStart = true;
int startTime = 0;
boolean missionEnd = false;
int missionEndCount = 0;

void magnetic_field(float mag[3][1]){
    //calculate magnetic field vector in inertial frame
    float g0 = (-29496.5 + 11.4*3)*pow(10,-9);
    float g1 = (-1585.9 + 16.7*3)*pow(10,-9);
    float h1 = (4945.1 - 28.8*3)*pow(10,-9);

    //coelevation
    float theta = PI/2 - flat*PI/180.0;
    float phi = flon*PI/180.0;
    float latitude = flat*PI/180;

    float Re = 6371.2*pow(10,3);

    float rb = altitude + Re;

    float br = 2*pow(Re/rb,3)*(g0*cos(theta)+(g1*cos(phi)+h1*sin(phi))*sin(theta));
    float btheta = pow(Re/rb,3)*(g0*sin(theta)-(g1*cos(phi)+h1*sin(phi))*cos(theta));
    float bphi = pow(Re/rb,3)*(g1*sin(phi)-h1*cos(phi));
    
    float rawMag[3][1] = {{-btheta},
                          {-bphi},
                          {br}};
    
    //Find the vector norm
    float norm=0;
    for(int i=0;i<3;i++){
         norm += (float)rawMag[i][0] * (float)rawMag[i][0];
    }
    norm = sqrt(norm);
    
    //Normalize the vector
    for(int i=0;i<3;i++){
        mag[i][0] = (float) rawMag[i][0] / norm;
    }
}

// Convert digital gyroscope measurements to rad/sec
void convert_gyro(int raw[3][1], float omega_dot[3][1]){
    int dps = 250; // deg per second spec sheet for Pololu MiniIMU-9
    int bits = 16; // Precision of gyro measurements
    
    for(int i=0;i<3;i++){
        omega_dot[i][0] = 250*raw[i][0]/(pow(2,bits-1)-1) * PI/180.0;
    }
}

// Multiple a 3x3 by a 3x1 matrix.
// Used to rotate the IMU measurements to the body frame.
void multiply_3x3(int* a, int* b, int* c)
{
    int i, j, k;
    for (i=0;i<3;i++){
        for(j=0;j<1;j++){
            c[1*i+j]=0;
            for (k=0;k<3;k++){
                c[1*i+j]= c[1*i+j]+a[3*i+k]*b[1*k+j];
            }
        }
    }
}

// Estimation Task.
// This task reads the IMU and estimates the rotation matrix.
static void vEstimationTask(void *pvParameters){
    vTaskDelay(configTICK_RATE_HZ);
    if (!gyro.init())
    {
      Serial.println("Failed to autodetect gyro type!");
      while (1);
    }
  
    gyro.enableDefault();
    
    compass.init();
    compass.enableDefault();  
    //compass.setMagGain(compass.magGain_56); 
    
    vTaskDelay(configTICK_RATE_HZ);
    
    // initialise the ticks variable with the current time.
    portTickType ticks = xTaskGetTickCount();
    
    int imu_time = millis();
    int est_time = millis();
    
    boolean wrote = false;

    while(1){   
        // wait until time to estimate
        vTaskDelayUntil(&ticks, 1000/EST_FREQ);
        
        if(xSemaphoreTake(i2cGateKeeper, 100)){
            // Read IMU values
            compass.read();
            gyro.read();
            
            imu_time = millis();
            
            xSemaphoreGive(i2cGateKeeper);
        }
        
        // Store IMU values into corresponding column vectors
        rawAccel[0][0] = -compass.a.x;
        rawAccel[1][0] = -compass.a.y;
        rawAccel[2][0] = -compass.a.z;

        
        rawGyro[0][0] = gyro.g.x;
        rawGyro[1][0] = gyro.g.y;
        rawGyro[2][0] = gyro.g.z;
        
        rawMag[0][0] = compass.m.x;
        rawMag[1][0] = compass.m.y;
        rawMag[2][0] = compass.m.z; 
        
        // Rotate IMU values onto the body frame
        multiply_3x3((int*)rotation, (int*)rawAccel, (int*)rotAccel);
        multiply_3x3((int*)rotation, (int*)rawGyro, (int*)rotGyro);
        multiply_3x3((int*)rotation, (int*)rawMag, (int*)rotMag);
        
        // Convert raw gyro values into rad/sec
        convert_gyro(rotGyro, omega_dot);
        /*
        Serial.print("!EST:");
        Serial.print(est_time);
        Serial.print(":");
        Serial.print(omega_dot[0][0]);
        Serial.print(",");
        Serial.print(omega_dot[1][0]);
        Serial.print(",");
        Serial.println(omega_dot[1][0]);
        */
        
        filtBuffer[0][filtIndex] = omega_dot[0][0];
        filtBuffer[1][filtIndex] = omega_dot[1][0];
        filtBuffer[2][filtIndex] = omega_dot[2][0];
        float sum[3][1] = {{0},{0},{0}};
        for(int i=0;i<3;i++){
            for(int j=0;j<filtLength;j++){
                sum[i][0] += filtBuffer[i][j];
            }      
        }
        for(int i=0;i<3;i++){
            filtered[i][0] = sum[i][0]/filtLength;
        }
        filtIndex += 1;
        if(filtIndex >= filtLength) filtIndex = 0;
        
        //Find the inertial frame for the magnetic field
        //magnetic_field(mag_i);
        
        // Estimate the attitude of the system
        estimate(rotAccel, omega_dot, rotMag, est);  
        est_time = millis();
        
        // Convert rotation matrix to euler angles
        yaw = atan2(est[0][1],est[0][0]);
        pitch = -asin(est[0][2]);
        roll = atan2(est[1][2],est[2][2]);
        
        /*
        Serial.print("!ANG:");
        Serial.print(roll * 180.0/PI);
        Serial.print(",");
        Serial.print(-pitch * 180.0/PI);
        Serial.print(",");
        Serial.println(-yaw * 180.0/PI);
        */
        
        /*
        triad(rotAccel, rotMag, est);  
        
        yaw = atan2(est[0][1],est[0][0]);
        pitch = -asin(est[0][2]);
        roll = atan2(est[1][2],est[2][2]);
        
        Serial.print("!EST:");
        Serial.print(est_time);
        Serial.print(":");
        Serial.print(yaw * 180.0/PI);
        Serial.print(",");
        Serial.print(pitch * 180.0/PI);
        Serial.print(",");
        Serial.println(roll * 180.0/PI);
        */
        if(!wrote){
            if(xSemaphoreTake(serialGateKeeper, 0)){
                Serial1.print("!IMU:");
                Serial1.print(imu_time);
                Serial1.print(":");
                Serial1.print(rotAccel[0][0]);
                Serial1.print(",");
                Serial1.print(rotAccel[1][0]);
                Serial1.print(",");
                Serial1.print(rotAccel[2][0]);
                Serial1.print(",");
                Serial1.print(rotGyro[0][0]);
                Serial1.print(",");
                Serial1.print(rotGyro[1][0]);
                Serial1.print(",");
                Serial1.print(rotGyro[2][0]);
                Serial1.print(",");
                Serial1.print(rotMag[0][0]);
                Serial1.print(",");
                Serial1.print(rotMag[1][0]);
                Serial1.print(",");
                Serial1.println(rotMag[2][0]);
    
                Serial1.print("!EST:");
                Serial1.print(est_time);
                Serial1.print(":");
                Serial1.print(yaw);
                Serial1.print(",");
                Serial1.print(pitch);
                Serial1.print(",");
                Serial1.println(roll);
                
                
                xSemaphoreGive(serialGateKeeper);
            }  
            wrote = !wrote;
        }else{
            wrote = !wrote;
        }
    }
}


static void vTorqueControlTask(void *pvParameters){
    float kp = 0.01;
    float kd = 0.01;
    float ki = 0.0001;
    float kt = .0137;  
    float x_i = 0;
    
    vTaskDelay(configTICK_RATE_HZ * 2);    
    
    // initialise the ticks variable with the current time.
    portTickType ticks = xTaskGetTickCount();

    
    while(1){
        vTaskDelayUntil(&ticks, 1000/TORQUE_CONT_FREQ);
        
        x_i += yaw;
        tau = -kp*yaw - kd*omega_dot[2][0] - ki*x_i;
        //tau = - kd*omega_dot[2][0];
        current_cmd = tau/kt;
        
        /*
        Serial.print("!CONT:");
        Serial.print(yaw);
        Serial.print(",");
        Serial.println(current_cmd);
        */
        
        
        if(tau > 0){
          current_cmd = 70.87 * tau + 0.20;
        }else{
          current_cmd = 70.87 * tau - 0.20;
        }   
    }
}

static void vCurrentControlTask(void *pvParameters){
    float kp = 0.1;
    float ki = 0.0005;
    float spd;
    float error=0;
    float x_i=0;
    
    int time = millis();
    
    boolean saturated = false;
    int saturatedTime = 0;
    
    vTaskDelay(configTICK_RATE_HZ * 2);
    
    // initialise the ticks variable with the current time.
    portTickType ticks = xTaskGetTickCount();
    
    boolean wrote = false;
    
    while(1){
        //batteryVoltage = (float)analogRead(VOLTAGE_PIN)/1023.0 * 3.3 * 4.0;
        if(missionStart && !missionEnd){
            vTaskDelayUntil(&ticks, 1000/CURRENT_CONT_FREQ);
            if(!saturated){
                current = -(((float)analogRead(CURRENT_PIN)/1024 * 3.3) - 1.28) * 5/1.25;
                
                
                
                if(current_cmd > 5){
                    current_cmd = 5;
                }else if(current_cmd < -5){
                    current_cmd = -5;
                }
                
                error = current_cmd - current;
                
                x_i = x_i + error;
                voltCmd[0] += kp*error + ki*x_i;
                
                /*
                Serial.print("!CONT:");
                Serial.print(current);
                Serial.print(":");
                Serial.print(current_cmd);
                Serial.print(":");
                Serial.println(voltCmd[0]);
                */
                
                if(voltCmd[0] > 12.0){
                    voltCmd[0] = 0; 
                    analogWrite(SPEED_PIN, 0); 
                    if(xSemaphoreTake(serialGateKeeper, 0)){
                        Serial1.print("!SYS:");
                        Serial1.print(time);
                        Serial1.print(":");
                        Serial1.print(voltCmd[0]);
                        Serial1.print(",");
                        Serial1.println("WHEEL SATURATED, SLEEPING FOR 2 MINTUES");
                        
                        xSemaphoreGive(serialGateKeeper);
                    }
                    saturated = true;
                    saturatedTime = millis();
                    
                }else if(voltCmd[0] < -12.0){
                    voltCmd[0] = 0;
                    analogWrite(SPEED_PIN, 0); 
                    if(xSemaphoreTake(serialGateKeeper, 0)){
                        Serial1.print("!SYS:");
                        Serial1.print(time);
                        Serial1.print(":");
                        Serial1.print(voltCmd[0]);
                        Serial1.print(",");
                        Serial1.println("WHEEL SATURATED, SLEEPING FOR 2 MINTUES");
                        
                        xSemaphoreGive(serialGateKeeper);
                    }
                    saturated = true;
                    saturatedTime = millis();
                    
                }
                
                spd = voltCmd[0];
                
                if(voltCmd[0] > 0){
                    digitalWrite(DIR_PIN,HIGH);
                }else{
                    digitalWrite(DIR_PIN,LOW);
                    spd = -spd;
                }
                
                analogWrite(SPEED_PIN, spd/12 * 255); 
                
                time = millis();
                if(!wrote){
                    if(xSemaphoreTake(serialGateKeeper, 0)){
                        Serial1.print("!CUR:");
                        Serial1.print(time);
                        Serial1.print(":");
                        Serial1.print(current_cmd);
                        Serial1.print(",");
                        Serial1.print(current);
                        Serial1.print(",");
                        Serial1.println(voltCmd[0]);
                        
                        xSemaphoreGive(serialGateKeeper);
                    }
                    wrote = !wrote;
                }else{
                    wrote = !wrote;
                }
            }else{
                if(millis() - saturatedTime > 120000){
                    saturated = false;  
                }
            }
        }else{
            analogWrite(SPEED_PIN, 0); 
        }
    }
}

static void vReadBMP(void *pvParameters){
    if (!bmp.begin()) {
        //Serial.println("Could not find a valid BMP085 sensor, check wiring!");
        while (1) {}
    }
    
    // BMP085 outputs
    float temp;
    float pressure;
    float alt;
    
    // initialise the ticks variable with the current time.
    portTickType ticks = xTaskGetTickCount();
    
    int time=millis();
    
    while(1){
        vTaskDelayUntil(&ticks, 1000 / READ_BMP_FREQ);   
        
        batteryVoltage = (float)analogRead(VOLTAGE_PIN)/1023.0 * 3.3 * 4.0;
        
        if(xSemaphoreTake(i2cGateKeeper, 100)){
            time = millis();
            temp = bmp.readTemperature();
            pressure = bmp.readPressure();
            bmpAltitude = bmp.readAltitude();
            
            Serial.println(temp);
            
            xSemaphoreGive(i2cGateKeeper);
        }
        
        if(altitude > highestAltitude){
            highestAltitude = altitude;
        }
        
        if(xSemaphoreTake(serialGateKeeper, 0)){
            Serial1.print("!BMP:");
            Serial1.print(time);
            Serial1.print(":");
            Serial1.print(temp);
            Serial1.print(",");
            Serial1.print(pressure);
            Serial1.print(",");
            Serial1.print(batteryVoltage);
            Serial1.print(",");
            Serial1.println(bmpAltitude);
            
            
            xSemaphoreGive(serialGateKeeper);
        }   
    }
}

static void vReadGPS(void *pvParameters){   
    // initialise the ticks variable with the current time.
    portTickType ticks = xTaskGetTickCount();
    
    int time=millis();
    
    // New GPS coordinate available
    boolean newGPS = false;
    
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;
    float newAltitude = 0;
    
    pinMode(GPS_LED_PIN, OUTPUT);
    
    while(1){
        //vTaskDelayUntil(&ticks, 1000 / READ_GPS_FREQ);   
        
        while (Serial2.available())
        {
            char c = Serial2.read();
            //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
            
            if (gps.encode(c)){ // Did a new valid sentence come in?
                newData = true;
            }
            
            if (newData)
            {
                time=millis();
                
                //Parse out relevant GPS data
                satellites = gps.satellites();
                hdop = gps.hdop();
                newAltitude = gps.f_altitude();
                if(newAltitude < 50000){
                    altitude = newAltitude;  
                }
                gps.f_get_position(&flat, &flon, &age);
                gps.get_datetime(&date, &utc_time, &age);
                
                digitalWrite(GPS_LED_PIN, HIGH);
                vTaskDelay(configTICK_RATE_HZ/8);
                digitalWrite(GPS_LED_PIN, LOW);

                if(altitude > ALTITUDE_FLOOR && !missionStart){
                    missionStart = true;
                    if(xSemaphoreTake(serialGateKeeper, 0)){
                      
                        Serial1.print("!SYS:");
                        Serial1.print(time);
                        Serial1.print(":");
                        Serial1.println("Mission Started!");
                        
                        startTime = time;
                        
                        xSemaphoreGive(serialGateKeeper);
                    }
                    //}else if((altitude > ALTITUDE_CEILING) || (time - startTime > 10800000)){
                }else if((highestAltitude - altitude > 500) && !missionEnd){
                    missionEndCount += 1;
                    if(missionEndCount > 5){
                        missionEnd = true;
                        
                        if(xSemaphoreTake(serialGateKeeper, 0)){
                          
                            Serial1.print("!SYS:");
                            Serial1.print(time);
                            Serial1.print(":");
                            Serial1.println("Mission Ended!");
                            
                            xSemaphoreGive(serialGateKeeper);
                        }
                    }
                }

                if(xSemaphoreTake(serialGateKeeper, 0)){
                    Serial1.print("!GPS:");
                    Serial1.print(time);
                    Serial1.print(":");
                    Serial1.print(utc_time);
                    Serial1.print(",");
                    Serial1.print(flat,6);
                    Serial1.print(",");
                    Serial1.print(flon,6);
                    Serial1.print(",");
                    Serial1.print(satellites);
                    Serial1.print(",");
                    Serial1.print(hdop);
                    Serial1.print(",");
                    Serial1.println(altitude,5);
                    
                    xSemaphoreGive(serialGateKeeper);
                }
                newData = false;
            }
        }
    }
}

static void vBuzzerTask(void *pvParameters){
    // initialise the ticks variable with the current time.
    portTickType ticks = xTaskGetTickCount();
    
    pinMode(BUZZER_PIN, OUTPUT);
    boolean buzzerOn = false;
    int time=millis();
    
    while(1){ 
        if(missionStart && missionEnd){
            if(buzzerOn){
                digitalWrite(BUZZER_PIN, HIGH);
                buzzerOn = !buzzerOn;
                vTaskDelay(configTICK_RATE_HZ * 1);
            }else{
                digitalWrite(BUZZER_PIN, LOW);
                buzzerOn = !buzzerOn;
                vTaskDelay(configTICK_RATE_HZ * 10);
            }
        }
    }
}


void setup()
{
    Serial.begin(57600);
    Serial1.begin(57600);
    Serial2.begin(9600);
    Wire.begin();
        
    pinMode(DIR_PIN, OUTPUT);
    
    i2cGateKeeper = xSemaphoreCreateMutex();
    serialGateKeeper = xSemaphoreCreateMutex();
    
    /*
    xTaskCreate(vEstimationTask,
                (signed portCHAR *) "Estimator",
                configMINIMAL_STACK_SIZE + 200,
                NULL,
                tskIDLE_PRIORITY + 4,
                &est_task);
    
    xTaskCreate(vTorqueControlTask,
                (signed portCHAR *) "Torque Controller",
                configMINIMAL_STACK_SIZE + 20,
                NULL,
                tskIDLE_PRIORITY + 2,
                &torque_control_task);
                
    xTaskCreate(vCurrentControlTask,
                (signed portCHAR *) "Current Controller",
                configMINIMAL_STACK_SIZE + 20,
                NULL,
                tskIDLE_PRIORITY + 3,
                &current_control_task);    
    */
    xTaskCreate(vReadBMP, 
                (signed portCHAR *) "BMP",
                configMINIMAL_STACK_SIZE + 20,
                NULL,
                tskIDLE_PRIORITY + 1,
                &bmp_task);   
    /*      
    xTaskCreate(vReadGPS, 
                (signed portCHAR *) "GPS",
                configMINIMAL_STACK_SIZE + 200,
                NULL,
                tskIDLE_PRIORITY + 1,
                &gps_task); 
                
    xTaskCreate(vBuzzerTask, 
                (signed portCHAR *) "BUZZER",
                configMINIMAL_STACK_SIZE + 20,
                NULL,
                tskIDLE_PRIORITY + 1,
                &buzzer_task);
    */
    vTaskStartScheduler();

    // should never return
    while(1);
}

void loop()
{
  
}
