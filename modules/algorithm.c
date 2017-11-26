/*
 * algorithm.c
 *
 *  Created on: Oct 15, 2017
 *      Author: snow
 */

#include "algorithm.h"

/* IMU_P IMU_I:Complementary filtering coefficient */
float IMU_P = 5.0f;
float IMU_I = 0.f;


//6轴计算得到四元数
void sensfusion6UpdateQ(FLOAT_GYRO *gyro, FLOAT_ACC *accel, float dt, volatile Quat *Q)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    float twoKp = IMU_P;    // 2 * proportional gain (Kp)
    float twoKi = IMU_I;     // 2 * integral gain (Ki)

    static float integralFBx = 0.0f;
    static float integralFBy = 0.0f;
    static float integralFBz = 0.0f;  // integral error terms scaled by Ki

    float qw, qx, qy, qz;
    float gx, gy, gz;
    float ax, ay, az;

    qw = Q->qw;
    qx = Q->qx;
    qy = Q->qy;
    qz = Q->qz;

    gx =  gyro->gyroX;
    gy =  gyro->gyroY;
    gz =  gyro->gyroZ;

    ax =  accel->accX;
    ay =  accel->accY;
    az =  accel->accZ;

    gx = gx * DEG_TO_RAD;
    gy = gy * DEG_TO_RAD;
    gz = gz * DEG_TO_RAD;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = qx * qz - qw * qy;
        halfvy = qw * qx + qy * qz;
        halfvz = qw * qw - 0.5f + qz * qz;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled

        integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;
        gx += integralFBx;  // apply integral feedback
        gy += integralFBy;
        gz += integralFBz;

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    float delta2 = (gx * gx + gy * gy + gz * gz) * dt * dt;

    float qw_last =  qw;
    float qx_last =  qx;
    float qy_last =  qy;
    float qz_last =  qz;

    qw = qw_last * (1.0f - delta2 * 0.125f) + (-qx_last * gx - qy_last * gy - qz_last * gz) * 0.5f * dt;
    qx = qx_last * (1.0f - delta2 * 0.125f) + (qw_last * gx + qy_last * gz - qz_last * gy) * 0.5f * dt;
    qy = qy_last * (1.0f - delta2 * 0.125f) + (qw_last * gy - qx_last * gz + qz_last * gx) * 0.5f * dt;
    qz = qz_last * (1.0f - delta2 * 0.125f) + (qw_last * gz + qx_last * gy - qy_last * gx) * 0.5f * dt;

    // Normalise quaternion
    recipNorm = invSqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= recipNorm;
    qx *= recipNorm;
    qy *= recipNorm;
    qz *= recipNorm;

    Q->qw = qw;
    Q->qx = qx;
    Q->qy = qy;
    Q->qz = qz;

}

//将欧拉角转换成四元数
void EulerAngleToQuaternion1(FLOAT_RPY *eur, volatile Quat *Q)
{
    float cosHRoll, cosHPitch, cosHYaw; //第一套欧拉角转四元数算法
    float sinHRoll, sinHPitch, sinHYaw;
    float recipNorm;

    float Pitch_in = eur->Pitch;
    float Rool_in = eur->Roll;
    float Yaw_in = eur->Yaw;

    float qw, qx, qy, qz;

    Rool_in  = Rool_in * DEG_TO_RAD;
    Pitch_in = Pitch_in * DEG_TO_RAD;
    Yaw_in   = Yaw_in  * DEG_TO_RAD;

    cosHRoll = cosf(0.5f * Rool_in);
    cosHPitch = cosf(0.5f * Pitch_in);
    cosHYaw = cosf(0.5f * Yaw_in);

    sinHRoll = sinf(0.5f * Rool_in);
    sinHPitch = sinf(0.5f * Pitch_in);
    sinHYaw = sinf(0.5f * Yaw_in);
//Yaw
	qw = cosHRoll * cosHPitch * cosHYaw + sinHRoll * sinHPitch * sinHYaw;
	qx = sinHRoll * cosHPitch * cosHYaw - cosHRoll * sinHPitch * sinHYaw;
	qy = cosHRoll * sinHPitch * cosHYaw + sinHRoll * cosHPitch * sinHYaw;
	qz = cosHRoll * cosHPitch * sinHYaw - sinHRoll * sinHPitch * cosHYaw;
//Roll
//	qw = cosHRoll * cosHPitch * cosHYaw - sinHRoll * sinHPitch * sinHYaw;
//	qx = sinHRoll * cosHPitch * cosHYaw + cosHRoll * sinHPitch * sinHYaw;
//	qy = cosHRoll * sinHPitch * cosHYaw - sinHRoll * cosHPitch * sinHYaw;
//	qz = cosHRoll * cosHPitch * sinHYaw + sinHRoll * sinHPitch * cosHYaw;

    recipNorm = 1.0f / sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= recipNorm;
    qx *= recipNorm;
    qy *= recipNorm;
    qz *= recipNorm;

    Q->qw = qw;
    Q->qx = qx;
    Q->qy = qy;
    Q->qz = qz;
}

//四元数到欧拉角，笛卡尔坐标系下 
void Quat2Euler(volatile  Quat *Q, FLOAT_RPY *eur)
{
    float qw, qx, qy, qz;

//    OS_ALLOC_SR();
//    OS_ENTER_CRITICAL();

    qw = Q->qw;
    qx = Q->qx;
    qy = Q->qy;
    qz = Q->qz;

//    OS_EXIT_CRITICAL();

    eur->Roll    = atan2f(2 * (qw * qx + qy * qz) , 1 - 2 * (qx * qx + qy * qy)) * RAD_TO_DEG; //输出+-90
//    eur->Pitch   = asinf(2 * (qw * qy - qz * qx)) * RAD_TO_DEG;              //笛卡尔坐标系                 //输出+-180
	eur->Pitch   = asinf(-2 * (qw * qy - qz * qx)) * RAD_TO_DEG;             //NED坐标系                  //输出+-180
    eur->Yaw     = atan2f(2 * (qw * qz + qx * qy) , 1 - 2 * (qy * qy + qz * qz)) * RAD_TO_DEG; //输出+-180
}

//四元数叉乘
/*
Q_Start :起始状态四元数
Q_Rotate：绕该四元数旋转
Q_Terminal：起始状态四元数Q_Start绕 Q_Rotate旋转后得到的新的状态四元数
Q_Terminal = Q_Start +  Q_Rotate
*/
void QuaternionMultiplicationCross(Quat *Q_Start, Quat *Q_Rotate, volatile Quat *Q_Terminal)
{
//    OS_ALLOC_SR();
//    OS_ENTER_CRITICAL();

    Quat Q_Start_c =  *Q_Start;
    Quat Q_Rotate_c = *Q_Rotate;

//    OS_EXIT_CRITICAL();

    Quat Q_Result;
    //叉乘
    Q_Result.qw = Q_Start_c.qw * Q_Rotate_c.qw - Q_Start_c.qx * Q_Rotate_c.qx - Q_Start_c.qy * Q_Rotate_c.qy - Q_Start_c.qz * Q_Rotate_c.qz;
    Q_Result.qx = Q_Start_c.qw * Q_Rotate_c.qx + Q_Start_c.qx * Q_Rotate_c.qw + Q_Start_c.qz * Q_Rotate_c.qy - Q_Start_c.qy * Q_Rotate_c.qz;
    Q_Result.qy = Q_Start_c.qw * Q_Rotate_c.qy + Q_Start_c.qy * Q_Rotate_c.qw + Q_Start_c.qx * Q_Rotate_c.qz - Q_Start_c.qz * Q_Rotate_c.qx;
    Q_Result.qz = Q_Start_c.qw * Q_Rotate_c.qz + Q_Start_c.qz * Q_Rotate_c.qw + Q_Start_c.qy * Q_Rotate_c.qx - Q_Start_c.qx * Q_Rotate_c.qy;

    float recipNorm;
    recipNorm = 1 / sqrtf(Q_Result.qw * Q_Result.qw + Q_Result.qx * Q_Result.qx + Q_Result.qy * Q_Result.qy + Q_Result.qz * Q_Result.qz);
    Q_Result.qw *= recipNorm;
    Q_Result.qx *= recipNorm;
    Q_Result.qy *= recipNorm;
    Q_Result.qz *= recipNorm;

//    OS_ENTER_CRITICAL();

    *Q_Terminal =  Q_Result;

//    OS_EXIT_CRITICAL();
}

//求误差四元数
/*
Q_Start:当前起始状态四元数
Q_Terminal：目标状态四元数
Q_Rotate：起始状态四元数到目标状态四元数需要旋转的量（Q_Terminal/Q_Start）
Q_Rotate = Q_Terminal - Q_Start
*/
void QuaternionDiviCross(Quat *Q_Start, Quat *Q_Terminal, volatile Quat *Q_Rotate)
{
//    OS_ALLOC_SR();
//    OS_ENTER_CRITICAL();

    Quat Q_Start_c =  *Q_Start;
    Quat Q_Terminal_c = *Q_Terminal;

//    OS_EXIT_CRITICAL();

    Quat Q_Result;
    //求共轭
    Q_Start_c.qw =  Q_Start_c.qw;
    Q_Start_c.qx = -Q_Start_c.qx;
    Q_Start_c.qy = -Q_Start_c.qy;
    Q_Start_c.qz = -Q_Start_c.qz;

    //叉乘
    Q_Result.qw = Q_Start_c.qw * Q_Terminal_c.qw - Q_Start_c.qx * Q_Terminal_c.qx - Q_Start_c.qy * Q_Terminal_c.qy - Q_Start_c.qz * Q_Terminal_c.qz;
    Q_Result.qx = Q_Start_c.qw * Q_Terminal_c.qx + Q_Start_c.qx * Q_Terminal_c.qw + Q_Start_c.qz * Q_Terminal_c.qy - Q_Start_c.qy * Q_Terminal_c.qz;
    Q_Result.qy = Q_Start_c.qw * Q_Terminal_c.qy + Q_Start_c.qy * Q_Terminal_c.qw + Q_Start_c.qx * Q_Terminal_c.qz - Q_Start_c.qz * Q_Terminal_c.qx;
    Q_Result.qz = Q_Start_c.qw * Q_Terminal_c.qz + Q_Start_c.qz * Q_Terminal_c.qw + Q_Start_c.qy * Q_Terminal_c.qx - Q_Start_c.qx * Q_Terminal_c.qy;

    float recipNorm;
    recipNorm = 1 / sqrtf(Q_Result.qw * Q_Result.qw + Q_Result.qx * Q_Result.qx + Q_Result.qy * Q_Result.qy + Q_Result.qz * Q_Result.qz);
    Q_Result.qw *= recipNorm;
    Q_Result.qx *= recipNorm;
    Q_Result.qy *= recipNorm;
    Q_Result.qz *= recipNorm;


//    OS_ENTER_CRITICAL();

    *Q_Rotate =  Q_Result;

//    OS_EXIT_CRITICAL();
}

//得到四元数旋转量
/*
Q: 误差四元数
Vector：得到的误差向量
Alpha： 得到的误差总角度
*/
void QuatToRotate(volatile Quat *rotateQ, volatile FLOAT_XYZ *rotateAngle)
{
    FLOAT_XYZ angle = {0};
    Quat rotateQ_c = *rotateQ;
    float Alpha = 0;
    Alpha = acosf(rotateQ_c.qw) * RAD_TO_DEG * 2;

    float scale = sqrt(rotateQ_c.qx * rotateQ_c.qx + rotateQ_c.qy * rotateQ_c.qy + rotateQ_c.qz * rotateQ_c.qz);
    if(scale == 0)
    {
        rotateAngle->X = 0;
        rotateAngle->Y = 0;
        rotateAngle->Z = 0;

        return ;
    }

    angle.X = rotateQ_c.qx * Alpha / scale ;
    angle.Y = rotateQ_c.qy * Alpha / scale;
    angle.Z = rotateQ_c.qz * Alpha / scale;

    *rotateAngle  = angle;
}

//误差四元数转到机体系
/*
Q:当前状态四元数
Dir：0:世界系转到机体系 1：机体系转到世界系
from： 需要旋转的向量
to: 旋转后的向量
*/
void quaternion_rotateVector(Quat *Q, FLOAT_XYZ *from, volatile FLOAT_XYZ *to, unsigned char Dir)
{

//    OS_ALLOC_SR();
//    OS_ENTER_CRITICAL();

    Quat Q_in = *Q;
    FLOAT_XYZ from_c = *from;

//    OS_EXIT_CRITICAL();

    FLOAT_XYZ to_c;

    if(Dir == 1) { Q_in.qw *= 1; Q_in.qx *= -1; Q_in.qy *= -1; Q_in.qz *= -1;  }


    float x2 = Q_in.qx * 2;
    float y2 = Q_in.qy * 2;
    float z2 = Q_in.qz * 2;

    float wx2 = Q_in.qw * x2;
    float wy2 = Q_in.qw * y2;
    float wz2 = Q_in.qw * z2;

    float xx2 = Q_in.qx * x2;
    float yy2 = Q_in.qy * y2;
    float zz2 = Q_in.qz * z2;

    float xy2 = Q_in.qx * y2;
    float yz2 = Q_in.qy * z2;
    float xz2 = Q_in.qz * x2;

    to_c.X = from_c.X * (1   - yy2 - zz2) + from_c.Y * (xy2 - wz2)       + from_c.Z * (xz2 + wy2);
    to_c.Y = from_c.X * (xy2 + wz2)       + from_c.Y * (1   - xx2 - zz2) + from_c.Z * (yz2 - wx2);
    to_c.Z = from_c.X * (xz2 - wy2)       + from_c.Y * (yz2 + wx2)       + from_c.Z * (1   - xx2 - yy2);

//    OS_ENTER_CRITICAL();

    * to =  to_c;

//    OS_EXIT_CRITICAL();
}

/**
    * Constructor from quaternion
    *
    * Instance is initialized from quaternion representing
    * coordinate transformation from frame 2 to frame 1.
    *
    * @param q quaternion to set dcm to
    */
void Dcm_from_quat(Quat q, float dcm[3][3]){
       float a = q.qw;
       float b = q.qx;
       float c = q.qy;
       float d = q.qz;
       float aSq = a * a;
       float bSq = b * b;
       float cSq = c * c;
       float dSq = d * d;
       dcm[0][0] = aSq + bSq - cSq - dSq;
       dcm[0][1] = 2 * (b * c - a * d);
       dcm[0][2] = 2 * (a * c + b * d);
       dcm[1][0] = 2 * (b * c + a * d);
       dcm[1][1] = aSq - bSq + cSq - dSq;
       dcm[1][2] = 2 * (c * d - a * b);
       dcm[2][0] = 2 * (b * d - a * c);
       dcm[2][1] = 2 * (a * b + c * d);
       dcm[2][2] = aSq - bSq - cSq + dSq;
}

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float constrain_float(float amt, float low, float high)
{
    if (isnan(amt))
    {
        return (low + high) * 0.5f;
    }
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

float apply_deadband(float value, float deadband)
{
    if (fabs(value) <= deadband) { value = 0; }

    if( value > deadband ) { value -= deadband; }
    if( value < -deadband ) { value += deadband; }

    return value;
}

float apply_limit(float value, float limit)
{
	if(fabs(value) >= limit) value = (value / fabs(value)) * limit;
	return value;
}

void step_change(float *in, float target, float step, float deadBand )
{
    if(fabs(*in - target) <= deadBand) {*in = target; return;}

    if(*in > target) {*in -= fabs(step); return;}
    if(*in < target) {*in += fabs(step); return;}
}

void pid_loop(PID *pid, float eurDesir, float measure)
{
    float err, diff;
    err = eurDesir - measure; //得到误差

    if(isnan(err)) { return; }

    diff = (err - pid->PreErr) / pid->Dt; //得到误差变化
    pid->PreErr = err;         //更新误差
    pid->EC = diff;


    pid->I_sum += err * pid->Dt;  //积分
    if(pid->I_sum > pid->I_max ) { pid->I_sum = pid->I_max; }
    if(pid->I_sum < -pid->I_max ) { pid->I_sum = -pid->I_max; } //抗饱和

    pid->Pout = pid->kp * err;
    pid->Iout = pid->ki * pid->I_sum;
    pid->Dout = pid->kd * diff;

    pid->Output = pid->Pout + pid->Dout + pid->Iout;
}

const unsigned char crcRfTbl[] =
{
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};


unsigned char  crcRfCal(unsigned char *buffer, unsigned int len)
{
    unsigned char crc = 0;
    for(unsigned int i = 0; i < len; i++)
    {
        crc =  crcRfTbl[crc ^ buffer[i]];
    }
    return  crc;
}

//冒泡排序，dir = 0 小到大  dir = 1 大到小
void sort(int *buffer, unsigned int len, unsigned char dir)
{
    if(len <= 1)
    {
        return ;
    }
    else
    {
        for(unsigned int i = 0; i < len - 1; i++)
        {
            for(unsigned int j = 0; j < len - 1 - i; j++)
            {
                if(dir == 0)
                {
                    if(buffer[j] > buffer[j + 1])
                    {
                        int temp;
                        temp = buffer[j];
                        buffer[j] = buffer[j + 1];
                        buffer[j + 1] = temp;
                    }
                }
                else
                {
                    if(buffer[j] < buffer[j + 1])
                    {
                        int temp;
                        temp = buffer[j];
                        buffer[j] = buffer[j + 1];
                        buffer[j + 1] = temp;
                    }
                }
            }
        }
    }
}

//线性变换
float linermap(const float L_max,   const float L_min,  \
              const float tar_max, const float tar_min, \
              float input_t){
    float input = constrain_float(input_t, L_min, L_max);
    float in_t = (input - L_min) / (L_max - L_min);

    float output_t = tar_min + ((tar_max - tar_min) * in_t);
    return output_t;
}
