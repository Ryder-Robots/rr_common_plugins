#include "rclcpp/rclcpp.hpp"
#include "rr_common_plugins/rr_state_maintainer_impl.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_plugins/rr_common_subscribers.hpp"
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
    sensor_msgs::msg::NavSatFix::SharedPtr msg_gps;

    msg_gps->header.stamp = current_time;
    msg_gps->header.frame_id = rr_constants::LINK_GPS;
    msg_gps->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg_gps->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    msg_gps->latitude = -33.8688;  // Degrees, e.g., Sydney
    msg_gps->longitude = 151.2093; // Degrees, e.g., Sydney
    msg_gps->altitude = 58.0;      // In meters above WGS84 ellipsoid
    std::fill(std::begin(msg_gps->position_covariance), std::end(msg_gps->position_covariance), 0.0);

    RrSubscriberGpsImpl gps;

    // msg_gps->std::shared_ptr<RrSubscriberGpsImpl>> gps_callback = std::make_shared<RrSubscriberGpsImpl>();
    // gps_callback.callback();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
