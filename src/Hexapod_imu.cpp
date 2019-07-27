/**
 * Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include <Arduino.h>
#include <Hexapod_imu.h>

/**
 *
 */
euler_angles_t IMU::quatToEulerWikipedia(quaternion_t q)
{
    euler_angles_t angles;

    // angle A, roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.qw * q.qx + q.qy * q.qz);
    double cosr_cosp = +1.0 - 2.0 * (q.qx * q.qx + q.qy * q.qy);
    angles.eA = atan2(sinr_cosp, cosr_cosp);

    // angle B, pitch (y-axis rotation)
    double sinp = +2.0 * (q.qw * q.qy - q.qz * q.qx);
    if (fabs(sinp) >= 1)
        angles.eB = copysign(HALF_PI, sinp); // use 90 degrees if out of range
    else
        angles.eB = asin(sinp);

    // angle C, yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.qw * q.qz + q.qx * q.qy);
    double cosy_cosp = +1.0 - 2.0 * (q.qy * q.qy + q.qz * q.qz);
    angles.eC = atan2(siny_cosp, cosy_cosp);

    angles.eA *= RAD_TO_DEG;
    angles.eB *= RAD_TO_DEG;
    angles.eC *= RAD_TO_DEG;

    return angles;
}

/**
 * Source:
 * http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
 *
 * airplane        telescope  symbol     angular    velocity
 * applied first   heading    azimuth    θ (theta)  yaw
 * applied second  attitude   elevation  φ (phi)    pitch
 * applied last    bank       tilt       ψ (psi)    roll
 */
euler_angles_t IMU::quatToEulerEuclideanspace(quaternion_t q1)
{
    euler_angles_t angles;
    double test = q1.qx * q1.qy + q1.qz * q1.qw;
    if (test > 0.499)
    { // singularity at north pole
        angles.eC = 2 * atan2(q1.qx, q1.qw);
        angles.eB = HALF_PI;
        angles.eA = 0;
        return angles;
    }
    if (test < -0.499)
    { // singularity at south pole
        angles.eC = -2 * atan2(q1.qx, q1.qw);
        angles.eB = -HALF_PI;
        angles.eA = 0;
        return angles;
    }
    double sqx = q1.qx * q1.qx;
    double sqy = q1.qy * q1.qy;
    double sqz = q1.qz * q1.qz;
    angles.eC = atan2(2 * q1.qy * q1.qw - 2 * q1.qx * q1.qz, 1 - 2 * sqy - 2 * sqz);
    angles.eB = asin(2 * test);
    angles.eA = atan2(2 * q1.qx * q1.qw - 2 * q1.qy * q1.qz, 1 - 2 * sqx - 2 * sqz);
    return angles;
}

/**
 *
 */
void IMU::toJSON(char *jsonMsg)
{
    // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
    // are all updated.
    // Quaternion values are, by default, stored in Q30 long
    // format. calcQuat turns them into a float between -1 and 1
    quaternion_t quat = {
        imu.calcQuat(imu.qw),
        imu.calcQuat(imu.qx),
        imu.calcQuat(imu.qy),
        imu.calcQuat(imu.qz)};

#define ALGO EUCLIDEANSPACE
#if ALGO == SPARKFUN
    eulerAngles = {imu.roll, imu.pitch, imu.yaw};
#elif ALGO == WIKIPEDIA
    eulerAngles = quatToEulerWikipedia(quat);
#elif ALGO == EUCLIDEANSPACE
    eulerAngles = quatToEulerEuclideanspace(quat);
#endif

    const char *formatString =
        R"rawText({"quat":{"qw":%f,"qx":%f,"qy":%f,"qz":%f},
 "euler":{"eA":%f,"eB":%f,"eC":%f}})rawText";
    sprintf(jsonMsg,
            formatString,
            quat.qw, quat.qx, quat.qy, quat.qz,
            eulerAngles.eA, eulerAngles.eB, eulerAngles.eC);
}

/**
 *
 */
void IMU::setupIMU(unsigned short fifoRate)
{
    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS)
    {
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
        while (1)
        {
            yield();
        }
    }

    imu.dmpBegin(true * DMP_FEATURE_6X_LP_QUAT |  // Enable 6-axis quat
                     true * DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                 fifoRate);                       // Set DMP FIFO rate to 10 Hz
                                                  // DMP_FEATURE_LP_QUAT can also be used. It uses the
                                                  // accelerometer in low-power mode to estimate quat's.
                                                  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
}

/**
 *
 */
euler_angles_t IMU::getEulerAngles()
{
    return eulerAngles;
}

/**
 *
 */
int8_t IMU::readIMU(char *jsonMsg)
{
    unsigned short fifoAvailable = imu.fifoAvailable();

    // Check for new data in the FIFO
    if (!fifoAvailable)
    {
        sprintf(jsonMsg, R"rawText({"msg":"NO IMU DATA AVAILABLE","Tms":%lu})rawText",
                millis());
        return 1;
    }

    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() != INV_SUCCESS)
    {
        sprintf(jsonMsg, R"rawText({"msg":"FIFO NOT UPDATED","Tms":%lu})rawText",
                millis());
        return 2;
    }

    // computeEulerAngles can be used -- after updating the
    // quaternion values -- to estimate roll, pitch, and yaw
    imu.computeEulerAngles();
    this->toJSON(jsonMsg);

    return 0;
}
