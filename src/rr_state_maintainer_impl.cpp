#include "rr_common_plugins/rr_state_maintainer_impl.hpp"

using namespace rr_common_plugins;

// gps setter/getter
void RrStateMaintainerImpl::set_gps(const sensor_msgs::msg::NavSatFix gps)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    feature_set_.has_gps = true;
    gps_ = gps;
}

bool RrStateMaintainerImpl::has_gps()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return feature_set_.has_gps;
}

const sensor_msgs::msg::NavSatFix RrStateMaintainerImpl::get_gps()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return gps_;
}

void RrStateMaintainerImpl::set_joystick(const sensor_msgs::msg::Joy joystick)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    feature_set_.has_joy = true;
    joystick_ = joystick;
}

bool RrStateMaintainerImpl::has_joystick()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return feature_set_.has_joy;
}

const sensor_msgs::msg::Joy RrStateMaintainerImpl::get_joystick()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return joystick_;
}

void RrStateMaintainerImpl::set_batt_state(const sensor_msgs::msg::BatteryState batt_state)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    feature_set_.has_batt_state = true;
    batt_state_ = batt_state;
}

bool RrStateMaintainerImpl::has_batt_state()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return feature_set_.has_batt_state;
}

const sensor_msgs::msg::BatteryState RrStateMaintainerImpl::get_batt_state()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return batt_state_;
}

void RrStateMaintainerImpl::set_image(const sensor_msgs::msg::Image img)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    feature_set_.has_img = true;
    img_ = img;
}

bool RrStateMaintainerImpl::has_image()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return feature_set_.has_img;
}

const sensor_msgs::msg::Image RrStateMaintainerImpl::get_image()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return img_;
}

void RrStateMaintainerImpl::set_imu(const sensor_msgs::msg::Imu imu)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);
    feature_set_.has_imu = true;
    imu_ = imu;
}

bool RrStateMaintainerImpl::has_imu()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return feature_set_.has_imu;
}

const sensor_msgs::msg::Imu RrStateMaintainerImpl::get_imu()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return imu_;
}

void RrStateMaintainerImpl::set_range(const sensor_msgs::msg::Range range)
{
    std::unique_lock<std::shared_mutex> lock(mutex_);

    if (std::find(RANGES_LINKS_.begin(), RANGES_LINKS_.end(), range.header.frame_id) == RANGES_LINKS_.end()) {
        RCLCPP_ERROR(logger_, "unsupported range sensor, %s not found in supported", range.header.frame_id.c_str());
    }

    feature_set_.has_ranges = true;
    for (size_t i = 0; i < ranges_.size(); ++i) {
        if (ranges_.at(i).header.frame_id == range.header.frame_id) {
            ranges_[i] = range;
            return;
        }
    }
    
    ranges_.resize(ranges_.size() + 1);
    ranges_.push_back(range);
}

bool RrStateMaintainerImpl::has_ranges()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return feature_set_.has_ranges;
}

const std::vector<sensor_msgs::msg::Range> RrStateMaintainerImpl::get_ranges()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return ranges_;
}

const rr_interfaces::msg::FeatureSet RrStateMaintainerImpl::get_feature_set()
{
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return feature_set_;
}

// default all feature sets to false to start with.
void RrStateMaintainerImpl::init(rclcpp::Logger logger)
{
    logger_ = logger;
    feature_set_.has_batt_state = false;
    feature_set_.has_gps = false;
    feature_set_.has_img = false;
    feature_set_.has_imu = false;
    feature_set_.has_joy = false;
    feature_set_.has_ranges = false;

    // set to zero for initilization.
    ranges_.resize(0);
}

PLUGINLIB_EXPORT_CLASS(rr_common_plugins::RrStateMaintainerImpl, rrobot::RrStateMaintainer)