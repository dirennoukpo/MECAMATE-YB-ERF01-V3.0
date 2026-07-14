#include "app_angle.h"
#include "app_math.h"
#include "config.h"


#define squa(Sq) (((float)Sq) * ((float)Sq))

const float Gyro_Gr = 0.00013323f * 2; // Angular rate to radians	this parameter matches a 500 deg/s gyro: 0.00026646f


static quaternion_t NumQ = {1, 0, 0, 0};
float vecxZ, vecyZ, veczZ;
float wz_acc_tmp[2];

attitude_t g_attitude;
imu_data_t g_imu_data;


/* Compute 1/sqrt(x) */
float q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (threehalfs - (x2 * y * y));
    return y;
}


/* Reset the quaternion */
void reset_quaternion(void)
{
    NumQ.q0 = 1.0;
    NumQ.q1 = 0.0;
    NumQ.q2 = 0.0;
    NumQ.q3 = 0.0;
}


/* Quaternion update  dt: 10MS */
void get_attitude_angle(imu_data_t *p_imu, attitude_t *p_angle, float dt)
{
    vector_t Gravity, Acc, Gyro, AccGravity;
    static vector_t GyroIntegError = {0};
    static float KpDef = 0.8f;
    static float KiDef = 0.0003f;
    float q0_t, q1_t, q2_t, q3_t;
    float NormQuat;
    float HalfTime = dt * 0.5f;

    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
    // Accelerometer normalization,
    NormQuat = q_rsqrt(squa(p_imu->accX)+ squa(p_imu->accY) +squa(p_imu->accZ)); 

    //After normalization these become the direction components of a unit vector
    Acc.x = p_imu->accX * NormQuat;
    Acc.y = p_imu->accY * NormQuat;
    Acc.z = p_imu->accZ * NormQuat;

    //Vector cross product; it yields the error of the rotation matrix gravity components relative to the new acceleration components
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;

    //Angular rate fused with the accelerometer proportional compensation; with the three lines above this forms the PI compensation, giving the corrected angular rate
    Gyro.x = p_imu->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x; //In radians; what is compensated here is the gyro drift
    Gyro.y = p_imu->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = p_imu->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
    // Update the quaternion
    // Integrate the corrected angular rate to get the change, between two attitude solutions, of the quaternion real part Q0 and the three imaginary parts Q1~3
    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

    //Add the integrated values to the previous quaternion, i.e. the new quaternion
    NumQ.q0 += q0_t; 
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // Normalize the quaternion again to get a unit vector
    NormQuat = q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3)); //Get the norm of the quaternion
    NumQ.q0 *= NormQuat;                                                               //Update the quaternion values with the norm
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    /* Compute the attitude angles */
    vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3; /*matrix (3,1)*/                                 //Gravity component of the X axis in the geographic frame
    vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1; /*matrix (3,2)*/                                 //Gravity component of the Y axis in the geographic frame
    veczZ = NumQ.q0 * NumQ.q0 - NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 + NumQ.q3 * NumQ.q3; /*matrix (3,3)*/ //Gravity component of the Z axis in the geographic frame

    #if ENABLE_ROLL_PITCH
    p_angle->pitch = asin(vecxZ);             //Pitch angle
    p_angle->roll = atan2f(vecyZ, veczZ);     //Roll angle
    #endif

    p_angle->yaw = atan2(2 * (NumQ.q1 * NumQ.q2 + NumQ.q0 * NumQ.q3), 
        NumQ.q0 * NumQ.q0 + NumQ.q1 * NumQ.q1 - NumQ.q2 * NumQ.q2 - NumQ.q3 * NumQ.q3); //Yaw angle

}
