#include "rclcpp/rclcpp.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_plugins/rr_state_maintainer_impl.hpp"
#include "rr_common_plugins/rr_subscriber_gps_impl.hpp"
#include "rr_common_plugins/rr_subscriber_joy_impl.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace rr_common_plugins;

class TestCommonSubscriber : public testing::Test
{
  protected:
    TestCommonSubscriber()
    {
    }

    ~TestCommonSubscriber() override
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    void SetUp() override
    {
        state_maintainer_->init(logger_);
    }

    void TearDown() override
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    rclcpp::Logger logger_ = rclcpp::get_logger("test_logger");
    std::shared_ptr<rrobot::RrStateMaintainer> state_maintainer_ = std::make_shared<RrStateMaintainerImpl>();
};

TEST_F(TestCommonSubscriber, gps)
{
    rclcpp::Clock clock;
    auto current_time = clock.now();
    auto msg_gps = std::make_shared<sensor_msgs::msg::NavSatFix>();

    msg_gps->header.stamp = current_time;
    msg_gps->header.frame_id = rr_constants::LINK_GPS;
    msg_gps->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg_gps->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    msg_gps->latitude = -33.8688;  // Degrees, e.g., Sydney
    msg_gps->longitude = 151.2093; // Degrees, e.g., Sydney
    msg_gps->altitude = 58.0;      // In meters above WGS84 ellipsoid
    std::fill(std::begin(msg_gps->position_covariance), std::end(msg_gps->position_covariance), 0.0);
    msg_gps->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

    // initlization of callback.
    auto subscriber = RrSubscriberGpsImpl();
    subscriber.set_state_handler(state_maintainer_);

    // running the callback
    subscriber.callback(msg_gps);

    EXPECT_TRUE(state_maintainer_->has_gps());
    EXPECT_EQ(state_maintainer_->get_gps().header.stamp, current_time);
    EXPECT_EQ(state_maintainer_->get_gps().header.frame_id, rr_constants::LINK_GPS);
    EXPECT_EQ(state_maintainer_->get_gps().status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    EXPECT_EQ(state_maintainer_->get_gps().status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
    EXPECT_NEAR(state_maintainer_->get_gps().latitude, -33.8688, 0.000009);
    EXPECT_NEAR(state_maintainer_->get_gps().longitude, 151.2093, 0.000009);
    EXPECT_NEAR(state_maintainer_->get_gps().altitude, 58, 1);

    EXPECT_EQ(subscriber.get_topic_default(), rr_constants::TOPIC_GPS_FIXED);
}

TEST_F(TestCommonSubscriber, joy)
{
    rclcpp::Clock clock;
    auto current_time = clock.now();
    auto msg = std::make_shared<sensor_msgs::msg::Joy>();

    msg->header.stamp = current_time;
    msg->header.frame_id = rr_constants::LINK_JOY_PS4;

    msg->axes.resize(4);
    auto it = msg->axes.begin();

    std::advance(it, rr_constants::JOY_AXES_LEFTX);
    msg->axes.insert(it, 0.1f);

    it = msg->axes.begin();
    std::advance(it, rr_constants::JOY_AXES_LEFTY);
    msg->axes.insert(it, -0.8f);

    it = msg->axes.begin();
    std::advance(it, rr_constants::JOY_AXES_RIGHTX);
    msg->axes.insert(it, 0.0);

    it = msg->axes.begin();
    std::advance(it, rr_constants::JOY_AXES_RIGHTY);
    msg->axes.insert(it, 0.3f);

    // fill up buffer to avoid nulls
    msg->buttons.resize(14);
    std::fill(msg->buttons.begin(), msg->buttons.end(), false);

    auto it2 = msg->buttons.begin();
    std::advance(it2, rr_constants::JOY_A_CROSS);
    msg->buttons.insert(it2, true);

    it2 = msg->buttons.begin();
    std::advance(it2, rr_constants::JOY_DPAD_UP);
    msg->buttons.insert(it2, true);

    auto subscriber = RrSubscriberJoyImpl();
    subscriber.set_state_handler(state_maintainer_);
    subscriber.callback(msg);

    // From experience found controllers will start to lose precision over time, especially
    // if you like first player shooters hahahahahahah, going with 0.0009 which is pretty arbitory
    sensor_msgs::msg::Joy actual = state_maintainer_->get_joystick();
    EXPECT_NEAR(actual.axes.at(rr_constants::JOY_AXES_LEFTX), 0.1, 0.11);
    EXPECT_NEAR(actual.axes.at(rr_constants::JOY_AXES_LEFTY), -0.8, 0.81);
    EXPECT_NEAR(actual.axes.at(rr_constants::JOY_AXES_RIGHTX), -0.8, 0.81);
    EXPECT_NEAR(actual.axes.at(rr_constants::JOY_AXES_RIGHTY), 0.3, 0.31);
    EXPECT_TRUE(actual.buttons.at(rr_constants::JOY_A_CROSS));
    EXPECT_TRUE(actual.buttons.at(rr_constants::JOY_DPAD_UP));

    EXPECT_TRUE(state_maintainer_->has_joystick());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
