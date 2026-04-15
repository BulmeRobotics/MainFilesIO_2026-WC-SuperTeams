#pragma once
/**
* @name:    Gyro.h
* @date:	15.01.2026
* @authors: Florian Wiesner
* @details: Header file for Gyro Sensor Class
*/

#ifdef _MSC_VER
    #pragma region Libraries //--------------------------------------------------------------------------------------------------
#endif
#include <Arduino.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include "CustomDatatypes.h"

#ifdef _MSC_VER
    #pragma endregion
    #pragma region Gyro Class //-------------------------------------------------------------------------------------------------
#endif
class Gyro {
    private:
        // Configuration
        static constexpr uint8_t I2C_ADDRESS_GYRO = 0x28;

        // Object
        Adafruit_BNO055 bno = Adafruit_BNO055(-1, I2C_ADDRESS_GYRO, &Wire);

        // Variables
        float diff_x, diff_y, diff_z;

        // Methods
        float GetRawAngle(GyroAxles axis);
    public:
        // Constructor
        Gyro() = default;

        // Methods
        /**
        * @brief  Initializes and configures the IMU (Gyro).
        * @return OK if the sensor was intitalized succesfully.
        *         ERROR if something went wrong.
        */
        ErrorCodes Init(void);

        /**
        * @brief  Method to get an (absolute) angle measurement from the IMU (0-360°).
        * @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
        * @return current angle in degrees (0-360°).
        */
        float GetAngle(GyroAxles axis);

        /**
        * @brief  Overloaded method to calculate values suiting for control loops in the struct GyroData.
        *         Includes absolute angle, cartesian angle, shortest direction and angleError. A measurement
        *         will be done if an axis is given, if a second angle is given it will be only calculated.
        * @param  targetAngle target angular value of the control loop (in degrees).
        * @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z). A measurement will be done.
        * @param actualAngle if instead of an axis an existing angle is given, the data will only be calculated
        * @return current angle in degrees (0-360°), the caluclated data is stored in 'data'.
        */
        float GetAngleAdvanced(float targetAngle, float actualAngle);
        float GetAngleAdvanced(float targetAngle, GyroAxles axis);

        /**
        * @brief  Method to get the angular value from a given orientation (NESW).
        * @param  orientation enum for the orientation (North, East, South, West).
        * @return corresponding angle in degrees (0, 90, 180, 270).
        */
        float GetAngleFromOrientation(Orientations orientation);

        /**
        * @brief  Overloaded method to get the orientation (NESW) from an angular value.
        * @param  angle angular value in degrees (0-360°).
        *         If no parameter is given, the angle is measured by the IMU.
        * @return corresponding orientation (NESW) (enum).
        */
        Orientations GetOrientationFromAngle(float angle);
        Orientations GetOrientationFromAngle(void);

        /**
        * @brief  Method to get an acceleration measurement from the IMU. DISCLAIMER - not well tested.
        * @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
        * @return current acceleration.
        */
        float GetAcceleration(GyroAxles axis);

        /**
        * @brief  Method to get an gravity measurement from the IMU. DISCLAIMER - not well tested.
        * @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
        * @return current gravity.
        */
        float GetGravity(GyroAxles axis);

        /**
        * @brief  Method to reset the angle of a specific axis. Zeros an axis.
        * @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
        */
        void ResetAngle(GyroAxles axis);

        /**
        * @brief  Method to reset all angles of the IMU. Zeros the IMU.
        */
        void ResetAllAngles(void);

        /**
        * @brief  Method to set an axis to a specific angle.
        * @param  axis enum to specify the axis (Axis_X, Axis_Y, Axis_Z).
        * @param  value angular value in degrees (0-360°).
        */
        void SetAngle(GyroAxles axis, float value);

        /**
        * @brief  Method to get a temperature measurement from the IMU.
        * @return temperature in °C
        */
        int8_t GetTemp(void);
        
        /**
        * @brief  Struct to store values used for control loops.
        * @param  angle_abs absolute angle in degrees (0-360°).
        * @param  angle_car cartesian angle in degrees (-180° to +180°)
        * @param  angle_error angular error between the target angle and the measurement (target-actual).
        * @param  direction_left true if a left-turn is the shortest option
        * @param  direction_right true if a right-turn is the shortest option
        */
        GyroData data;
};
#ifdef _MSC_VER
    #pragma endregion
#endif