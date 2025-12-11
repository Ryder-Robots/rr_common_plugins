// Copyright (c) 2025 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "rr_common_plugins/imu_service_serial_plugin.hpp"
#include "rr_common_plugins/generated/rr_serial.pb.h"
#include "rr_common_base/rr_constants.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using namespace rr_common_plugins::rr_serial_plugins;
using RROpCodeE = rr_constants::rr_op_code_t;

class TestImuServiceSerialPlugin : public testing::Test
{
  protected:
    TestImuServiceSerialPlugin() {}

    ~TestImuServiceSerialPlugin() override
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    void SetUp() override
    {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }

    // Helper function to create a serialized IMU response
    std::vector<uint8_t> createImuResponse(
        double qx, double qy, double qz, double qw,
        double avx, double avy, double avz,
        double lax, double lay, double laz)
    {
        org::ryderrobots::ros2::serial::Response response;
        response.set_op(RROpCodeE::MSP_RAW_IMU);

        auto* imu_data = response.mutable_msp_raw_imu();

        // Set orientation
        auto* orientation = imu_data->mutable_orientation();
        orientation->set_x(qx);
        orientation->set_y(qy);
        orientation->set_z(qz);
        orientation->set_w(qw);

        // Set orientation covariance (9 elements)
        for (int i = 0; i < 9; i++) {
            imu_data->add_orientation_covariance(0.01 * i);
        }

        // Set angular velocity
        auto* angular_velocity = imu_data->mutable_angular_velocity();
        angular_velocity->set_x(avx);
        angular_velocity->set_y(avy);
        angular_velocity->set_z(avz);

        // Set angular velocity covariance (9 elements)
        for (int i = 0; i < 9; i++) {
            imu_data->add_angular_velocity_covariance(0.02 * i);
        }

        // Set linear acceleration
        auto* linear_acceleration = imu_data->mutable_linear_acceleration();
        linear_acceleration->set_x(lax);
        linear_acceleration->set_y(lay);
        linear_acceleration->set_z(laz);

        // Set linear acceleration covariance (9 elements)
        for (int i = 0; i < 9; i++) {
            imu_data->add_linear_acceleration_covariance(0.03 * i);
        }

        std::string serialized_data;
        response.SerializeToString(&serialized_data);

        return std::vector<uint8_t>(serialized_data.begin(), serialized_data.end());
    }

    // Helper function to create a response with wrong opcode
    std::vector<uint8_t> createWrongOpcodeResponse()
    {
        org::ryderrobots::ros2::serial::Response response;
        response.set_op(999); // Invalid opcode

        std::string serialized_data;
        response.SerializeToString(&serialized_data);

        return std::vector<uint8_t>(serialized_data.begin(), serialized_data.end());
    }
};

TEST_F(TestImuServiceSerialPlugin, plugin_instantiation)
{
    std::shared_ptr<ImuServiceSerialPlugin> plugin =
        std::make_shared<ImuServiceSerialPlugin>();

    EXPECT_TRUE(plugin != nullptr);
}

TEST_F(TestImuServiceSerialPlugin, protobuf_serialization_orientation)
{
    // Test that we can create and serialize an IMU response with orientation
    auto serialized = createImuResponse(
        0.0, 0.0, 0.0, 1.0,  // Quaternion (identity)
        0.1, 0.2, 0.3,       // Angular velocity
        9.8, 0.0, 0.0        // Linear acceleration
    );

    EXPECT_GT(serialized.size(), 0);

    // Verify deserialization works
    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));
    EXPECT_EQ(response.op(), RROpCodeE::MSP_RAW_IMU);

    const auto& imu_data = response.msp_raw_imu();
    EXPECT_TRUE(imu_data.has_orientation());
    EXPECT_DOUBLE_EQ(imu_data.orientation().x(), 0.0);
    EXPECT_DOUBLE_EQ(imu_data.orientation().y(), 0.0);
    EXPECT_DOUBLE_EQ(imu_data.orientation().z(), 0.0);
    EXPECT_DOUBLE_EQ(imu_data.orientation().w(), 1.0);
}

TEST_F(TestImuServiceSerialPlugin, protobuf_serialization_angular_velocity)
{
    // Test angular velocity values
    auto serialized = createImuResponse(
        0.0, 0.0, 0.0, 1.0,
        0.5, -0.3, 0.8,      // Angular velocity
        9.8, 0.0, 0.0
    );

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));

    const auto& imu_data = response.msp_raw_imu();
    EXPECT_TRUE(imu_data.has_angular_velocity());
    EXPECT_DOUBLE_EQ(imu_data.angular_velocity().x(), 0.5);
    EXPECT_DOUBLE_EQ(imu_data.angular_velocity().y(), -0.3);
    EXPECT_DOUBLE_EQ(imu_data.angular_velocity().z(), 0.8);
}

TEST_F(TestImuServiceSerialPlugin, protobuf_serialization_linear_acceleration)
{
    // Test linear acceleration values
    auto serialized = createImuResponse(
        0.0, 0.0, 0.0, 1.0,
        0.1, 0.2, 0.3,
        -1.5, 2.3, 9.81      // Linear acceleration
    );

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));

    const auto& imu_data = response.msp_raw_imu();
    EXPECT_TRUE(imu_data.has_linear_acceleration());
    EXPECT_DOUBLE_EQ(imu_data.linear_acceleration().x(), -1.5);
    EXPECT_DOUBLE_EQ(imu_data.linear_acceleration().y(), 2.3);
    EXPECT_DOUBLE_EQ(imu_data.linear_acceleration().z(), 9.81);
}

TEST_F(TestImuServiceSerialPlugin, protobuf_covariance_matrices)
{
    // Test that covariance matrices are properly serialized
    auto serialized = createImuResponse(
        0.0, 0.0, 0.0, 1.0,
        0.1, 0.2, 0.3,
        9.8, 0.0, 0.0
    );

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));

    const auto& imu_data = response.msp_raw_imu();

    // Check orientation covariance
    EXPECT_EQ(imu_data.orientation_covariance_size(), 9);
    for (int i = 0; i < 9; i++) {
        EXPECT_DOUBLE_EQ(imu_data.orientation_covariance(i), 0.01 * i);
    }

    // Check angular velocity covariance
    EXPECT_EQ(imu_data.angular_velocity_covariance_size(), 9);
    for (int i = 0; i < 9; i++) {
        EXPECT_DOUBLE_EQ(imu_data.angular_velocity_covariance(i), 0.02 * i);
    }

    // Check linear acceleration covariance
    EXPECT_EQ(imu_data.linear_acceleration_covariance_size(), 9);
    for (int i = 0; i < 9; i++) {
        EXPECT_DOUBLE_EQ(imu_data.linear_acceleration_covariance(i), 0.03 * i);
    }
}

TEST_F(TestImuServiceSerialPlugin, protobuf_wrong_opcode_deserialization)
{
    // Test that we can deserialize a message with wrong opcode
    auto serialized = createWrongOpcodeResponse();

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));
    EXPECT_NE(response.op(), RROpCodeE::MSP_RAW_IMU);
}

TEST_F(TestImuServiceSerialPlugin, protobuf_empty_data_deserialization)
{
    // Test deserialization of empty data (protobuf accepts empty as valid empty message)
    std::vector<uint8_t> empty_data;

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(empty_data.data(), empty_data.size()));
    // Empty data should result in default values
    EXPECT_EQ(response.op(), 0);
    EXPECT_FALSE(response.has_msp_raw_imu());
}

TEST_F(TestImuServiceSerialPlugin, protobuf_request_serialization)
{
    // Test that we can create and serialize an IMU request
    org::ryderrobots::ros2::serial::Request request;
    request.set_op(RROpCodeE::MSP_RAW_IMU);

    auto* monitor = request.mutable_monitor();
    monitor->set_is_request(true);

    std::string serialized_data;
    EXPECT_TRUE(request.SerializeToString(&serialized_data));
    EXPECT_GT(serialized_data.size(), 0);

    // Verify deserialization
    org::ryderrobots::ros2::serial::Request request_check;
    EXPECT_TRUE(request_check.ParseFromString(serialized_data));
    EXPECT_EQ(request_check.op(), RROpCodeE::MSP_RAW_IMU);
    EXPECT_TRUE(request_check.has_monitor());
    EXPECT_TRUE(request_check.monitor().is_request());
}

TEST_F(TestImuServiceSerialPlugin, extreme_values_quaternion)
{
    // Test with extreme quaternion values
    auto serialized = createImuResponse(
        1.0, 0.0, 0.0, 0.0,  // 180 degree rotation around X
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    );

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));

    const auto& imu_data = response.msp_raw_imu();
    EXPECT_DOUBLE_EQ(imu_data.orientation().x(), 1.0);
    EXPECT_DOUBLE_EQ(imu_data.orientation().w(), 0.0);
}

TEST_F(TestImuServiceSerialPlugin, negative_acceleration_values)
{
    // Test with negative acceleration (e.g., free fall or inverted)
    auto serialized = createImuResponse(
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, -9.81      // Inverted gravity
    );

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));

    const auto& imu_data = response.msp_raw_imu();
    EXPECT_DOUBLE_EQ(imu_data.linear_acceleration().z(), -9.81);
}

TEST_F(TestImuServiceSerialPlugin, high_angular_velocity)
{
    // Test with high angular velocity values (rapid rotation)
    auto serialized = createImuResponse(
        0.0, 0.0, 0.0, 1.0,
        10.5, -8.3, 15.7,    // High angular velocity (rad/s)
        0.0, 0.0, 9.81
    );

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));

    const auto& imu_data = response.msp_raw_imu();
    EXPECT_DOUBLE_EQ(imu_data.angular_velocity().x(), 10.5);
    EXPECT_DOUBLE_EQ(imu_data.angular_velocity().y(), -8.3);
    EXPECT_DOUBLE_EQ(imu_data.angular_velocity().z(), 15.7);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
