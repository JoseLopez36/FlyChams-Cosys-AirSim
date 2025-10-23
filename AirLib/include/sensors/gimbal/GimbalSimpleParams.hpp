// Developed by Jose Francisco Lopez-Ruiz, University of Seville

#ifndef msr_airlib_GimbalSimpleParams_hpp
#define msr_airlib_GimbalSimpleParams_hpp

#include "common/Common.hpp"
#include "common/EarthUtils.hpp"
#include "common/AirSimSettings.hpp"
#include <cmath>

namespace msr
{
    namespace airlib
    {

        struct GimbalSimpleParams
        {
            // Gimbal IMU parameters
            struct Gyroscope
            {
                // Angular random walk (ARW)
                real_T arw = 0.30f / sqrt(3600.0f) * M_PIf / 180; // deg/sqrt(hour) converted to rad/sqrt(sec)
                // Bias Stability (tau = 500s)
                real_T tau = 500;
                real_T bias_stability = 4.6f / 3600 * M_PIf / 180; // deg/hr converted to rad/sec
                Vector3r turn_on_bias = Vector3r::Zero(); // Assume calibration is done
            } gyro;

            struct Accelerometer
            {
                // Velocity random walk (ARW)
                real_T vrw = 0.24f * EarthUtils::Gravity / 1.0E3f; // mg converted to m/s^2
                // Bias Stability (tau = 800s)
                real_T tau = 800;
                real_T bias_stability = 36.0f * 1E-6f * EarthUtils::Gravity; // ug converted to m/s^2
                Vector3r turn_on_bias = Vector3r::Zero(); // Assume calibration is done
            } accel;

            real_T min_sample_time = 1 / 1000.0f; // Internal IMU frequency

            // Encoder parameters
            struct Encoder
            {
                real_T resolution = 0.02f * M_PIf / 180;        // 0.02° resolution
                real_T bias = 0.0f;                             // Systematic offset (radians)
                real_T noise_stddev = 0.01f * M_PIf / 180;      // Random noise (radians)
            } encoder;

            bool initializeFromSettings(const AirSimSettings::GimbalSetting& settings)
            {
                const auto& json = settings.settings;
                float arw = json.getFloat("AngularRandomWalk", Utils::nan<float>());
                if (!std::isnan(arw)) {
                    gyro.arw = arw / sqrt(3600.0f) * M_PIf / 180; // //deg/sqrt(hour) converted to rad/sqrt(sec)
                }
                gyro.tau = json.getFloat("GyroBiasStabilityTau", gyro.tau);
                float bias_stability = json.getFloat("GyroBiasStability", Utils::nan<float>());
                if (!std::isnan(bias_stability)) {
                    gyro.bias_stability = bias_stability / 3600 * M_PIf / 180; //deg/hr converted to rad/sec
                }
                auto vrw = json.getFloat("VelocityRandomWalk", Utils::nan<float>());
                if (!std::isnan(vrw)) {
                    accel.vrw = vrw * EarthUtils::Gravity / 1.0E3f; //mg converted to m/s^2
                }
                accel.tau = json.getFloat("AccelBiasStabilityTau", accel.tau);
                bias_stability = json.getFloat("AccelBiasStability", Utils::nan<float>());
                if (!std::isnan(bias_stability)) {
                    accel.bias_stability = bias_stability * 1E-6f * EarthUtils::Gravity; //ug converted to m/s^2
                }

                // Encoder parameters
                float encoder_resolution = json.getFloat("EncoderResolution", Utils::nan<float>());
                if (!std::isnan(encoder_resolution)) {
                    encoder.resolution = encoder_resolution * M_PIf / 180; // degrees to radians
                }
                encoder.bias = json.getFloat("EncoderBias", encoder.bias);
                float encoder_noise = json.getFloat("EncoderNoise", Utils::nan<float>());
                if (!std::isnan(encoder_noise)) {
                    encoder.noise_stddev = encoder_noise * M_PIf / 180; // degrees to radians
                }

                return json.getBool("GenerateNoise", false);
            }
        };
    }
} //namespace
#endif
