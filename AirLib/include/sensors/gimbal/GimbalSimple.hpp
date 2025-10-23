// Developed by Jose Francisco Lopez-Ruiz, University of Seville

#ifndef msr_airlib_GimbalSimple_hpp
#define msr_airlib_GimbalSimple_hpp

#include "common/Common.hpp"
#include "GimbalSimpleParams.hpp"
#include "GimbalBase.hpp"

namespace msr
{
    namespace airlib
    {

        class GimbalSimple : public GimbalBase
        {
        public:
            // Constructors
            GimbalSimple(const AirSimSettings::GimbalSetting& setting = AirSimSettings::GimbalSetting())
                : GimbalBase(setting.sensor_name)
            {
                // Initialize params
                add_noise_ = params_.initializeFromSettings(setting);

                gyro_bias_stability_norm_ = params_.gyro.bias_stability / sqrt(params_.gyro.tau);
                accel_bias_stability_norm_ = params_.accel.bias_stability / sqrt(params_.accel.tau);
            }

            //*** Start: UpdatableState implementation ***//
            virtual void resetImplementation() override
            {
                last_time_ = clock()->nowNanos();

                state_.gyroscope_bias = params_.gyro.turn_on_bias;
                state_.accelerometer_bias = params_.accel.turn_on_bias;
                gauss_dist.reset();
                updateOutput();
            }

            virtual void update(float delta = 0) override
            {
                GimbalBase::update(delta);

                updateOutput();
            }
            //*** End: UpdatableState implementation ***//

            virtual ~GimbalSimple() = default;

        public:
            // Method to set the ground truth gimbal end-effector orientation
            void setEndEffectorOrientation(const Quaternionr& end_effector_orientation)
            {
                gimbal_end_effector_orientation_ = end_effector_orientation;
            }

        private: //methods
            void updateOutput()
            {
                Output output;
                const GroundTruth& ground_truth = getGroundTruth();

                // IMU data from gimbal platform
                output.angular_velocity = ground_truth.kinematics->twist.angular;
                output.linear_acceleration = ground_truth.kinematics->accelerations.linear - ground_truth.environment->getState().gravity;
                output.orientation = ground_truth.kinematics->pose.orientation;

                // Acceleration is in world frame so transform to body frame
                output.linear_acceleration = VectorMath::transformToBodyFrame(output.linear_acceleration,
                    ground_truth.kinematics->pose.orientation,
                    true);

                // Calculate encoder angles from gimbal orientation
                calculateEncoderAngles(output);

                // Add noise to both IMU and encoder data
                if (add_noise_) {
                    addNoise(output.linear_acceleration, output.angular_velocity);
                    addEncoderNoise(output);
                }

                output.time_stamp = clock()->nowNanos();

                setOutput(output);
            }

            void calculateEncoderAngles(Output& output)
            {
                // Extract Euler angles from the end-effector orientation
                float pitch, roll, yaw;
                VectorMath::toEulerianAngle(end_effector_orientation_, pitch, roll, yaw);

                // Store encoder angles for each axis
                output.yaw = yaw;
                output.pitch = pitch;
                output.roll = roll;

                // Calculate encoder-based angular rates (simple finite difference)
                TTimeDelta dt = clock()->updateSince(last_time_);
                if (dt > 0)
                {
                    output.yaw_rate = (output.yaw - prev_yaw_) / dt;
                    output.pitch_rate = (output.pitch - prev_pitch_) / dt;
                    output.roll_rate = (output.roll - prev_roll_) / dt;
                }
                else
                {
                    output.yaw_rate = 0.0f;
                    output.pitch_rate = 0.0f;
                    output.roll_rate = 0.0f;
                }

                // Store current angles for next iteration
                prev_yaw_ = output.yaw;
                prev_pitch_ = output.pitch;
                prev_roll_ = output.roll;
            }

            void addEncoderNoise(Output& output)
            {
                // Add encoder noise and quantization
                output.yaw = quantizeAngle(output.yaw + params_.encoder.bias + gauss_dist.next() * params_.encoder.noise_stddev);
                output.pitch = quantizeAngle(output.pitch + params_.encoder.bias + gauss_dist.next() * params_.encoder.noise_stddev);
                output.roll = quantizeAngle(output.roll + params_.encoder.bias + gauss_dist.next() * params_.encoder.noise_stddev);

                // Add noise to encoder rates
                float rate_noise_scale = 10.0f; // Scale for rate noise
                output.yaw_rate += gauss_dist.next() * params_.encoder.noise_stddev * rate_noise_scale;
                output.pitch_rate += gauss_dist.next() * params_.encoder.noise_stddev * rate_noise_scale;
                output.roll_rate += gauss_dist.next() * params_.encoder.noise_stddev * rate_noise_scale;
            }

            float quantizeAngle(float angle)
            {
                // Quantize to encoder resolution
                return std::round(angle / params_.encoder.resolution) * params_.encoder.resolution;
            }

            void addNoise(Vector3r& linear_acceleration, Vector3r& angular_velocity)
            {
                TTimeDelta dt = clock()->updateSince(last_time_);

                real_T sqrt_dt = static_cast<real_T>(sqrt(std::max<TTimeDelta>(dt, params_.min_sample_time)));

                // Gyrosocpe
                // Convert arw to stddev
                real_T gyro_sigma_arw = params_.gyro.arw / sqrt_dt;
                angular_velocity += gauss_dist.next() * gyro_sigma_arw + state_.gyroscope_bias;
                // Update bias random walk
                real_T gyro_sigma_bias = gyro_bias_stability_norm_ * sqrt_dt;
                state_.gyroscope_bias += gauss_dist.next() * gyro_sigma_bias;

                // Accelerometer
                // Convert vrw to stddev
                real_T accel_sigma_vrw = params_.accel.vrw / sqrt_dt;
                linear_acceleration += gauss_dist.next() * accel_sigma_vrw + state_.accelerometer_bias;
                // Update bias random walk
                real_T accel_sigma_bias = accel_bias_stability_norm_ * sqrt_dt;
                state_.accelerometer_bias += gauss_dist.next() * accel_sigma_bias;
            }

        private: //fields
            GimbalSimpleParams params_;
            RandomVectorGaussianR gauss_dist = RandomVectorGaussianR(0, 1);

            // Cached calculated values
            real_T gyro_bias_stability_norm_, accel_bias_stability_norm_;

            bool add_noise_;

            struct State
            {
                Vector3r gyroscope_bias;
                Vector3r accelerometer_bias;
            } state_;

            TTimePoint last_time_;

            // Previous encoder angles for rate calculation
            float prev_yaw_ = 0.0f;
            float prev_pitch_ = 0.0f;
            float prev_roll_ = 0.0f;
            
            // Ground truth gimbal end-effector orientation
            Quaternionr end_effector_orientation_ = Quaternionr::Identity();
        };
    }
} //namespace
#endif
