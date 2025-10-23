// Developed by Jose Francisco Lopez-Ruiz, University of Seville

#ifndef msr_airlib_GimbalBase_hpp
#define msr_airlib_GimbalBase_hpp

#include "sensors/SensorBase.hpp"

namespace msr
{
    namespace airlib
    {

        class GimbalBase : public SensorBase
        {
        public:
            GimbalBase(const std::string& sensor_name = "")
                : SensorBase(sensor_name)
            {
            }

        public: // Types
            struct Output
            {
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                    TTimePoint time_stamp;

                // IMU data (from gimbal's internal IMU)
                Quaternionr orientation;        // Global NED orientation of gimbal platform
                Vector3r angular_velocity;      // Gimbal angular velocity (rad/s)
                Vector3r linear_acceleration;   // Gimbal linear acceleration (m/s²)

                // Encoder data
                float yaw;                      // Yaw encoder angle (radians)
                float pitch;                    // Pitch encoder angle (radians)
                float roll;                     // Roll encoder angle (radians)

                // Encoder velocities
                float yaw_rate;                 // Yaw angular rate from encoder (rad/s)
                float pitch_rate;               // Pitch angular rate from encoder (rad/s)
                float roll_rate;                // Roll angular rate from encoder (rad/s)
            };

        public:
            const Output& getOutput() const
            {
                return output_;
            }

        protected:
            void setOutput(const Output& output)
            {
                output_ = output;
            }

        private:
            Output output_;
        };
    }
} //namespace
#endif
