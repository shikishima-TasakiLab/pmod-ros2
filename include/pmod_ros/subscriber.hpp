#pragma once
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include <std_msgs/msg/string.hpp>

template<class msg_T>
class Subscriber
{
public:
    using SharedPtr = std::shared_ptr<Subscriber>;

    Subscriber(
        const rclcpp::Node::SharedPtr &node,
        const std::string &topic,
        const uint32_t queue_size
    ) {
        this->_sub = node->create_subscription<msg_T>(
            topic, queue_size, std::bind(&Subscriber::_callback, this, _1)
        );
    }

    virtual msg_T::ConstPtr get_msg() {
        std::lock_guard<std::mutex> lock(this->_mtx);
        return this->_msg;
    }

protected:
    rclcpp::Subscription<msg_T>::SharedPtr _sub;
    std::mutex _mtx;

    msg_T::ConstPtr _msg;

    virtual void _callback(
        const msg_T::ConstPtr &msg
    ) {
        std::lock_guard<std::mutex> lock(this->_mtx);
        this->_msg = msg;
    }
};

template<class M0, class M1>
class TimeSyncSubscriber
{
public:
    using SharedPtr = std::shared_ptr<TimeSyncSubscriber>;

    TimeSyncSubscriber(
        const rclcpp::Node::SharedPtr &node,
        const std::string &topic0,
        const std::string &topic1,
        const uint32_t queue_size
    ) {
        this->_sub0 = message_filters::Subscriber<std_msgs::msg::String>(*node, topic0, 1);
        this->_sub1 = message_filters::Subscriber<std_msgs::msg::String>(*node, topic1, 1);
        this->_sync = message_filters::TimeSynchronizer<M0, M1>(*this->_sub0, *this->_sub1, queue_size);
        this->_sync->registerCallback(std::bind(&TimeSyncSubscriber::_callback, this, _1, _2));
    }

    void get_msgs(
        M0::ConstPtr &msg0,
        M1::ConstPtr &msg1
    ) {
        std::lock_guard<std::mutex> lock(this->_mtx);
        msg0 = this->_msg0;
        msg1 = this->_msg1;
    }

private:
    std::mutex _mtx;
    message_filters::Subscriber<M0> _sub0;
    message_filters::Subscriber<M1> _sub1;
    message_filters::TimeSynchronizer<M0, M1> _sync;

    M0::ConstPtr _msg0;
    M1::ConstPtr _msg1;

    void _callback(
        const std::shared_ptr<const M0> &msg0,
        const std::shared_ptr<const M1> &msg1
    ) {
        std::lock_guard<std::mutex> lock(this->_mtx);
        this->_msg0 = msg0;
        this->_msg1 = msg1;
    }
};
