#ifndef RR_COMMON_PLUGINS__RR_COMMON_SUBSCRIBERS_HPP_
#define RR_COMMON_PLUGINS__RR_COMMON_SUBSCRIBERS_HPP_

#include "rr_common_plugins/visibility_control.h"
#include "rr_common_base/rr_subscriber.hpp"

namespace rr_common_plugins
{
    /**
     * @class RrSubscriberGps
     * @brief implementation of GPS subscription service.
     */
    class RrSubscriberGpsImpl : public rrobot::RrSubscriberGps 
    {
        std::string get_topic_param() override
        {
            
        }
    };
}
#endif