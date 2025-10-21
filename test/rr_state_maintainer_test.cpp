#include "rclcpp/rclcpp.hpp"
#include "rr_common_plugins/rr_state_maintainer_impl.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>


using namespace rr_common_plugins;

class TestController : public testing::Test
{
  protected:
    TestController()
    {
    }

    ~TestController() override
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
    //RrStateMaintainerImpl state_maintainer_;
    std::shared_ptr<rrobot::RrStateMaintainer> state_maintainer_ = std::make_shared<RrStateMaintainerImpl>();
};

// Test setters and getters
TEST_F(TestController, gps)
{
    rclcpp::Clock clock;
    auto current_time = clock.now();

    sensor_msgs::msg::NavSatFix expected;
    expected.header.stamp = current_time;
    expected.header.frame_id = "gps_link";
    expected.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    expected.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    // Set geographic coordinates (latitude, longitude, altitude)
    expected.latitude = -33.8688;  // Degrees, e.g., Sydney
    expected.longitude = 151.2093; // Degrees, e.g., Sydney
    expected.altitude = 58.0;      // In meters above WGS84 ellipsoid

    // Set position covariance (if known, otherwise leave as default zeros)
    std::fill(std::begin(expected.position_covariance), std::end(expected.position_covariance), 0.0);
    expected.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    state_maintainer_->set_gps(expected);

    // Result
    sensor_msgs::msg::NavSatFix actual = state_maintainer_->get_gps();
    EXPECT_EQ(actual.header.stamp, current_time);
    EXPECT_EQ(actual.header.frame_id, "gps_link");
    EXPECT_EQ(actual.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
    EXPECT_EQ(actual.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);

    // allow a tolerance of around 1 meter.
    EXPECT_NEAR(actual.latitude, -33.8688, 0.000009);
    EXPECT_NEAR(actual.longitude, 151.2093, 0.000009);
    EXPECT_NEAR(actual.altitude, 58.0, 1);

    GTEST_EXPECT_TRUE(state_maintainer_->has_gps());
}

TEST_F(TestController, range)
{
    rclcpp::Clock clock;
    auto current_time = clock.now();
    sensor_msgs::msg::Range expected1;
    expected1.header.frame_id = rr_constants::LINK_ULTRA_SONIC_CENTER;
    expected1.header.stamp = current_time;

    // reference: https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304
    expected1.min_range = 2;
    expected1.max_range = 500;
    expected1.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    expected1.range = 80;

    // this value can be calculated as FoV = 2 x arctan(beam radius / distance from sensor), URM09 the beam radius is 60 degrees
    expected1.field_of_view = 2 * atan(2 * (60 / expected1.range));

    RCLCPP_INFO(logger_, "attempting to set range");
    state_maintainer_->set_range(expected1);
    RCLCPP_INFO(logger_, "range should be set");

    EXPECT_TRUE(state_maintainer_->get_feature_set().has_ranges);
    EXPECT_EQ(rr_constants::LINK_ULTRA_SONIC_CENTER, state_maintainer_->get_ranges()[0].header.frame_id);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}
