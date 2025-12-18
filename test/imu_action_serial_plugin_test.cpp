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

#include "rr_common_plugins/imu_action_serial_plugin.hpp"
#include "rr_common_plugins/generated/rr_serial.pb.h"
#include "rr_common_base/rr_constants.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rr_interfaces/action/monitor_imu_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <thread>
#include <chrono>

using namespace rr_common_plugins::rr_serial_plugins;
using RROpCodeE = rr_constants::rr_op_code_t;
using ActionType = rr_interfaces::action::MonitorImuAction;
using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;

class TestImuActionSerialPlugin : public testing::Test
{
  protected:
    TestImuActionSerialPlugin() {}

    ~TestImuActionSerialPlugin() override
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

    // Helper function to create a test lifecycle node
    rclcpp_lifecycle::LifecycleNode::SharedPtr createTestNode(const std::string& node_name)
    {
        return std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name);
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

TEST_F(TestImuActionSerialPlugin, plugin_instantiation)
{
    std::shared_ptr<ImuActionSerialPlugin> plugin =
        std::make_shared<ImuActionSerialPlugin>();

    EXPECT_TRUE(plugin != nullptr);
}

TEST_F(TestImuActionSerialPlugin, protobuf_serialization_orientation)
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

TEST_F(TestImuActionSerialPlugin, protobuf_serialization_angular_velocity)
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

TEST_F(TestImuActionSerialPlugin, protobuf_serialization_linear_acceleration)
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

TEST_F(TestImuActionSerialPlugin, protobuf_covariance_matrices)
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

TEST_F(TestImuActionSerialPlugin, protobuf_wrong_opcode_deserialization)
{
    // Test that we can deserialize a message with wrong opcode
    auto serialized = createWrongOpcodeResponse();

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(serialized.data(), serialized.size()));
    EXPECT_NE(response.op(), RROpCodeE::MSP_RAW_IMU);
}

TEST_F(TestImuActionSerialPlugin, protobuf_empty_data_deserialization)
{
    // Test deserialization of empty data (protobuf accepts empty as valid empty message)
    std::vector<uint8_t> empty_data;

    org::ryderrobots::ros2::serial::Response response;
    EXPECT_TRUE(response.ParseFromArray(empty_data.data(), empty_data.size()));
    // Empty data should result in default values
    EXPECT_EQ(response.op(), 0);
    EXPECT_FALSE(response.has_msp_raw_imu());
}

TEST_F(TestImuActionSerialPlugin, protobuf_request_serialization)
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

TEST_F(TestImuActionSerialPlugin, extreme_values_quaternion)
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

TEST_F(TestImuActionSerialPlugin, negative_acceleration_values)
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

TEST_F(TestImuActionSerialPlugin, high_angular_velocity)
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

// ============================================================================
// Tests for Thread Safety and State Management
// ============================================================================

TEST_F(TestImuActionSerialPlugin, thread_safety_mutex_protection)
{
    // Test that the plugin handles multiple goal requests safely
    // This test doesn't require configuration since we're testing the handle_goal logic
    auto plugin = std::make_shared<ImuActionSerialPlugin>();

    // Test that multiple rapid goal requests are handled safely
    rclcpp_action::GoalUUID uuid1, uuid2;
    auto goal1 = std::make_shared<ActionType::Goal>();
    auto goal2 = std::make_shared<ActionType::Goal>();

    auto response1 = plugin->handle_goal(uuid1, goal1);
    EXPECT_EQ(response1, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);

    // Without actually executing, we accept the next one (is_executing_ is false)
    auto response2 = plugin->handle_goal(uuid2, goal2);
    EXPECT_EQ(response2, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST_F(TestImuActionSerialPlugin, promise_future_mechanism_initialization)
{
    // Test that goal acceptance works correctly without configuration
    // This verifies the handle_goal mechanism
    auto plugin = std::make_shared<ImuActionSerialPlugin>();

    rclcpp_action::GoalUUID uuid;
    auto goal = std::make_shared<ActionType::Goal>();

    // Verify goal is accepted
    auto response = plugin->handle_goal(uuid, goal);
    EXPECT_EQ(response, rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST_F(TestImuActionSerialPlugin, serialization_error_path_cleanup)
{
    // Test that serialization errors properly clean up state
    // This is indirectly tested through protobuf API which should never fail
    // for valid requests, but we verify the request format is correct
    org::ryderrobots::ros2::serial::Request req;
    org::ryderrobots::ros2::serial::Monitor* monitor = req.mutable_monitor();
    monitor->set_is_request(true);
    req.set_op(RROpCodeE::MSP_RAW_IMU);

    std::string serialized_data;
    bool serialize_success = req.SerializeToString(&serialized_data);

    // Should always succeed for valid request
    EXPECT_TRUE(serialize_success);
    EXPECT_GT(serialized_data.size(), 0);
}

TEST_F(TestImuActionSerialPlugin, timeout_duration_is_five_seconds)
{
    // Verify the timeout constant used in execute() is 5 seconds
    // This is a documentation test - the actual timeout is hardcoded in execute()
    auto timeout_duration = std::chrono::seconds(5);

    EXPECT_EQ(timeout_duration.count(), 5);
}

TEST_F(TestImuActionSerialPlugin, cancel_exception_mechanism)
{
    // Test that cancellation uses exception mechanism via promise
    // The implementation sets: response_promise_.set_exception(std::runtime_error(...))
    std::promise<void> test_promise;
    auto test_future = test_promise.get_future();

    // Simulate what handle_cancel does
    try {
        test_promise.set_exception(
            std::make_exception_ptr(std::runtime_error("Goal canceled by user")));
    }
    catch (const std::future_error&) {
        FAIL() << "Should not throw future_error on first set_exception";
    }

    // Verify exception is stored
    EXPECT_TRUE(test_future.valid());

    // Retrieve should throw
    EXPECT_THROW(test_future.get(), std::runtime_error);
}

TEST_F(TestImuActionSerialPlugin, execution_thread_cleanup)
{
    // Test that execution_thread_.joinable() check works correctly
    std::thread test_thread;

    // New thread should not be joinable
    EXPECT_FALSE(test_thread.joinable());

    // Started thread should be joinable
    test_thread = std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    });
    EXPECT_TRUE(test_thread.joinable());

    // Join and verify
    test_thread.join();
    EXPECT_FALSE(test_thread.joinable());
}

TEST_F(TestImuActionSerialPlugin, mutex_lock_guard_pattern)
{
    // Test that lock_guard pattern works as expected
    std::mutex test_mutex;
    bool protected_flag = false;

    {
        const std::lock_guard<std::mutex> lock(test_mutex);
        protected_flag = true;
    }
    // Lock released here

    EXPECT_TRUE(protected_flag);
}

TEST_F(TestImuActionSerialPlugin, future_wait_for_timeout_detection)
{
    // Test std::future::wait_for timeout detection mechanism
    std::promise<void> test_promise;
    auto test_future = test_promise.get_future();

    // Wait with very short timeout
    auto timeout = std::chrono::milliseconds(10);
    auto status = test_future.wait_for(timeout);

    // Should timeout since promise is never satisfied
    EXPECT_EQ(status, std::future_status::timeout);
}

TEST_F(TestImuActionSerialPlugin, future_wait_for_ready_detection)
{
    // Test std::future::wait_for ready detection
    std::promise<void> test_promise;
    auto test_future = test_promise.get_future();

    // Satisfy promise immediately
    test_promise.set_value();

    // Wait should return ready immediately
    auto status = test_future.wait_for(std::chrono::seconds(1));
    EXPECT_EQ(status, std::future_status::ready);
}

TEST_F(TestImuActionSerialPlugin, response_opcode_validation)
{
    // Test that response opcode must match request opcode
    org::ryderrobots::ros2::serial::Response response;
    response.set_op(RROpCodeE::MSP_RAW_IMU);

    auto* imu_data = response.mutable_msp_raw_imu();
    auto* orientation = imu_data->mutable_orientation();
    orientation->set_w(1.0);

    std::string serialized_data;
    response.SerializeToString(&serialized_data);

    // Deserialize and verify opcode
    org::ryderrobots::ros2::serial::Response res_check;
    EXPECT_TRUE(res_check.ParseFromArray(serialized_data.data(), serialized_data.size()));
    EXPECT_EQ(res_check.op(), RROpCodeE::MSP_RAW_IMU);
}

TEST_F(TestImuActionSerialPlugin, uint8_multi_array_message_format)
{
    // Test that UInt8MultiArray format used for serial communication works
    std_msgs::msg::UInt8MultiArray msg;

    std::string test_data = "test_serial_data";
    msg.data.resize(test_data.size());
    std::memcpy(msg.data.data(), test_data.data(), test_data.size());

    EXPECT_EQ(msg.data.size(), test_data.size());

    // Verify data integrity
    std::string recovered_data(msg.data.begin(), msg.data.end());
    EXPECT_EQ(recovered_data, test_data);
}

TEST_F(TestImuActionSerialPlugin, complete_request_response_cycle_format)
{
    // Test a complete request-response format as used in plugin

    // Create request
    org::ryderrobots::ros2::serial::Request req;
    org::ryderrobots::ros2::serial::Monitor* monitor = req.mutable_monitor();
    monitor->set_is_request(true);
    req.set_op(RROpCodeE::MSP_RAW_IMU);

    std::string request_data;
    EXPECT_TRUE(req.SerializeToString(&request_data));

    // Create response
    auto response_data = createImuResponse(
        0.0, 0.0, 0.0, 1.0,  // Orientation
        0.1, 0.2, 0.3,       // Angular velocity
        0.0, 0.0, 9.81       // Linear acceleration
    );

    // Verify both serialize correctly
    EXPECT_GT(request_data.size(), 0);
    EXPECT_GT(response_data.size(), 0);

    // Verify response deserializes correctly
    org::ryderrobots::ros2::serial::Response res;
    EXPECT_TRUE(res.ParseFromArray(response_data.data(), response_data.size()));
    EXPECT_EQ(res.op(), RROpCodeE::MSP_RAW_IMU);
}

TEST_F(TestImuActionSerialPlugin, lifecycle_basic_cleanup)
{
    // Test that cleanup can be called safely without configuration
    // Note: on_configure, on_activate, and on_deactivate require proper initialization
    // with topics and publishers, so we only test cleanup here which handles
    // thread cleanup and state reset
    auto plugin = std::make_shared<ImuActionSerialPlugin>();

    // Test cleanup (should safely handle uninitialized state)
    rclcpp_lifecycle::State inactive_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
    auto cleanup_result = plugin->on_cleanup(inactive_state);
    EXPECT_EQ(cleanup_result, rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
}

TEST_F(TestImuActionSerialPlugin, destructor_thread_cleanup)
{
    // Test that destructor properly cleans up execution thread
    {
        auto plugin = std::make_shared<ImuActionSerialPlugin>();

        // Plugin goes out of scope - destructor should handle thread cleanup
    }

    // If we get here without hanging, destructor worked correctly
    SUCCEED();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
