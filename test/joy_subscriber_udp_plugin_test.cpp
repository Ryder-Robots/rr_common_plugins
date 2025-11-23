#include "rr_common_plugins/generated/rr_client.pb.h"
#include "rr_common_plugins/joy_subscriber_udp_plugin.hpp"
#include <gtest/gtest.h>

using namespace rr_common_plugins::rr_udp_plugins;

class TestController : public testing::Test
{
  protected:
    TestController() {}

    ~TestController() override
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    void SetUp() override
    {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override { rclcpp::shutdown(); }
};


TEST_F(TestController, deserialize1)
{
    std::shared_ptr<RrJoySubscriberUdpPlugin> deserializer =
        std::make_shared<RrJoySubscriberUdpPlugin>();

    if (deserializer == nullptr) {
      EXPECT_TRUE(false);
    }    
      
    InboundMessage packet;
    packet.clear_data();
    Joystick *joystick_data = packet.mutable_joystick();

    joystick_data->add_axes(0.5f);
    joystick_data->add_axes(-0.7f);

    // Initialize joystick buttons (example values)
    joystick_data->add_buttons(1);
    joystick_data->add_buttons(0);

    sensor_msgs::msg::Joy joy = deserializer->deserialize(packet);

    EXPECT_EQ(joy.axes[0], 0.5f);
    EXPECT_EQ(joy.axes[1], -0.7f);
    EXPECT_EQ(joy.buttons[0], 1);
    EXPECT_EQ(joy.buttons[1], 0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto result = RUN_ALL_TESTS();
    return result;
}