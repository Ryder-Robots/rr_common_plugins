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


#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rr_common_base/rr_action_plugin_iface.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_plugins/generated/rr_serial.pb.h"
#include <memory>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace rr_common_plugins
{
    /**
     * @class RRActionPluginBase
     *
     * @brief Provides lifecycle methods and serial communication infrastructure for plugin actions.
     *
     * This base class manages serial communication via protobuf-serialized messages over ROS2 topics.
     * It handles message serialization, deserialization, buffer management, and provides thread-safe
     * access to communication state. Derived action plugins use this class to communicate with
     * hardware devices via the /serial_write and /serial_read topics.
     *
     * State transitions are as follows:
     *
     * ACTION_STATE_PREPARING → ACTION_STATE_SENT → ACTION_STATE_PROCESSING → ACTION_STATE_SUCCESS/FAIL
     *
     * Thread Safety:
     * All public methods are thread-safe. Internal state is protected by the provided mutex.
     * The class uses a _no_lock naming convention for internal methods that assume the mutex
     * is already held by the caller.
     *
     */
    class RRActionPluginBase
    {
        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        using State = rclcpp_lifecycle::State;
        using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
        using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;
        using LifecyclePublisher = rclcpp_lifecycle::LifecyclePublisher<UInt8MultiArray>;
        using Subscription = rclcpp::Subscription<UInt8MultiArray>;
        using RRActionStatusE = rr_constants::rr_action_status_t;
        using RROpCodeE = rr_constants::rr_op_code_t;
        using SerialResponse = org::ryderrobots::ros2::serial::Response;
        using SerialRequest = org::ryderrobots::ros2::serial::Request;
        using Time = builtin_interfaces::msg::Time;

      public:
        /**
         * @brief Constructs the action plugin base with specified operation code and mutex.
         *
         * @param op_code The operation code that this plugin will handle. Used to filter
         *                incoming serial responses to match only relevant messages.
         * @param mutex Shared mutex for thread-safe access to internal state. This mutex
         *              should be shared with the owning action plugin to coordinate state.
         */
        explicit RRActionPluginBase(RROpCodeE op_code, std::shared_ptr<std::mutex> mutex) : op_code_(op_code), mutex_(mutex) {}
        ~RRActionPluginBase() = default;

        /**
         * @fn on_configure
         * @brief Called by action concrete implementation during configure phase of its lifecycle.
         *
         * Verifies that /serial_read and /serial_write topics are available in the ROS2 graph.
         * Creates publisher and subscriber for serial communication with SensorDataQoS profile.
         * If topics are not available, configuration will fail.
         *
         * @param state Previous or current state of concrete node, or the state of previous lifecycle method.
         * @param node Concrete node shared pointer, used to create topic subscriptions and publishers.
         * @return CallbackReturn::SUCCESS if both topics are available and subscriptions created successfully,
         *         CallbackReturn::FAILURE if topics are not available.
         */
        [[nodiscard]] CallbackReturn on_configure(const State &state, LifecycleNode::SharedPtr node);

        /**
         * @fn on_activate
         * @brief Performs lifecycle activation procedure.
         *
         * Clears the internal buffer and resets state to ACTION_STATE_PREPARING.
         * Activates the lifecycle publisher for /serial_write topic.
         *
         * @param state Current lifecycle state.
         * @return CallbackReturn::SUCCESS on successful activation.
         */
        [[nodiscard]] CallbackReturn on_activate(const State &state);

        /**
         * @fn on_deactivate
         * @brief Performs lifecycle deactivation routines.
         *
         * Sets deactivation flag to prevent new callbacks from processing.
         * Deactivates the publisher and cleans up the subscription after a brief delay
         * to allow in-flight callbacks to complete.
         *
         * @param state Current lifecycle state.
         * @return CallbackReturn::SUCCESS on successful deactivation.
         */
        [[nodiscard]] CallbackReturn on_deactivate(const State &state);

        /**
         * @fn on_cleanup
         * @brief Performs lifecycle cleanup routines.
         *
         * Currently a no-op placeholder for future cleanup operations.
         *
         * @param state Current lifecycle state.
         * @return CallbackReturn::SUCCESS always.
         */
        [[nodiscard]] CallbackReturn on_cleanup(const State &state);

        /**
         * @fn set_status
         * @brief Thread-safe setter for action status.
         *
         * Updates the internal action state machine status. This is used to track
         * the progress of serial communication from preparing through to success/failure.
         *
         * @param status New status to set.
         */
        void set_status(RRActionStatusE status);

        /**
         * @fn get_status
         * @brief Thread-safe getter for action status.
         *
         * @return Current action status from the state machine.
         */
        RRActionStatusE get_status();

        /**
         * @fn is_res_avail
         * @brief Thread-safe check for response availability.
         *
         * @return true if a valid response has been received and is ready for retrieval,
         *         false otherwise.
         */
        bool is_res_avail();

        /**
         * @fn get_res
         * @brief Thread-safe retrieval of serial response.
         *
         * Retrieves the deserialized protobuf response and marks it as consumed
         * by setting res_avail_ to false. Should only be called after is_res_avail()
         * returns true.
         *
         * @return SerialResponse containing the protobuf response data.
         */
        SerialResponse get_res();

        /**
         * @fn publish
         * @brief Serializes and publishes a serial request to /serial_write topic.
         *
         * Serializes the protobuf request, appends termination character, and publishes
         * to the serial write topic. Resets the buffer state to ACTION_STATE_PREPARING
         * to await the response.
         *
         * @param req SerialRequest protobuf message to serialize and send.
         * @return true if serialization and publish succeeded, false if serialization failed.
         */
        bool publish(SerialRequest req);

        /**
         * @fn get_time_stamp
         * @brief Thread-safe retrieval of the timestamp from the last received response.
         *
         * The timestamp is updated when a response is successfully received and set via
         * set_res(). This can be used to timestamp result messages.
         *
         * @return Time message containing the timestamp of the last response.
         */
        Time get_time_stamp();

      private:
        // Topic names for serial communication
        static constexpr const char* WRITE_TOPIC_ = "/serial_write";  ///< Topic for publishing serialized requests
        static constexpr const char*  READ_TOPIC_ = "/serial_read";   ///< Topic for receiving serialized responses

        LifecyclePublisher::SharedPtr publisher_ = nullptr;   ///< Publisher for /serial_write topic
        Subscription::SharedPtr subscription_ = nullptr;      ///< Subscriber for /serial_read topic
        rclcpp::Logger logger_ = rclcpp::get_logger("ImuActionSerialPluginBase");  ///< Logger for debug/error messages

        /**
         * @brief Internal method to set response without acquiring mutex.
         *
         * Stores the received response, updates timestamp, and marks response as available.
         * MUST be called with mutex already held.
         *
         * @param res Deserialized serial response to store.
         */
        void set_res(SerialResponse res);

        /**
         * @brief Marks buffer as complete and resets state without acquiring mutex.
         *
         * Sets buf_complete_ flag, updates status, and clears the buffer.
         * MUST be called with mutex already held.
         *
         * @param status Status to set (typically PREPARING, SUCCESS, or FAIL).
         */
        void buf_complete_no_lock(RRActionStatusE status);

        /**
         * @brief Thread-safe version of buf_complete_no_lock.
         *
         * Acquires mutex before calling buf_complete_no_lock.
         *
         * @param status Status to set (typically PREPARING, SUCCESS, or FAIL).
         */
        void buf_complete(RRActionStatusE status);

        /**
         * @brief Thread-safe check if buffer processing is complete.
         *
         * @return true if previous buffer has been processed and cleared.
         */
        bool is_buf_complete();

        /**
         * @brief Callback for receiving serial data packets from /serial_read.
         *
         * Accumulates incoming data into buffer_ until termination character is received.
         * When complete packet is received, deserializes protobuf response and validates
         * op_code matches this plugin's expected operation. Handles partial packets
         * across multiple callbacks.
         *
         * @param packet Incoming data packet containing serialized protobuf data.
         */
        void subscriber_cb(const UInt8MultiArray::UniquePtr &packet);

        bool buf_complete_ = false;  ///< Flag indicating if current buffer processing is complete
        bool res_avail_ = false;     ///< Flag indicating if a response is available for retrieval
        bool deactivate_ = false;    ///< Flag to stop processing callbacks during deactivation

        /**
         * @brief Thread-safe getter for buffer size.
         *
         * @return Current size of the accumulation buffer.
         */
        size_t buffer_size();

        std::string buffer_;  ///< Accumulation buffer for assembling multi-packet messages

        const uint8_t TERM_CHAR = 0x1E;  ///< Record separator character marking end of protobuf message

        RRActionStatusE status_ = RRActionStatusE::ACTION_STATE_PREPARING;  ///< Current action state
        RROpCodeE op_code_;        ///< Operation code this plugin handles (for response filtering)
        SerialResponse res_;       ///< Most recent deserialized response
        Time timestamp_;           ///< Timestamp of most recent response
        rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);  ///< Clock for generating timestamps

        std::shared_ptr<std::mutex> mutex_;  ///< Shared mutex for thread-safe access to internal state
    };
} // namespace rr_common_plugins