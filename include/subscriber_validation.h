#include <string>
#include <fstream>
#include <vector>

#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/approximate_time_model.h>

#include <sensor_msgs/msg/joint_state.hpp>

#include "signal.h"
#include "helper.h"

using namespace message_filters;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;
using std::placeholders::_6;
using std::placeholders::_7;
using std::placeholders::_8;
using std::placeholders::_9;

namespace synchronizer
{

#define buffer_size 10000000

class SubscriberTopic2
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic2(int period1, int period2, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic2"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), period1_(period1), period2_(period2),
        count_alg_(0), count_mdl_(0), delay_previous1_(0), delay_previous2_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), previous_timestamp1_(0), previous_timestamp2_(0), observed_wcp1_(0), observed_wcp2_(0)
    {
        this->declare_parameter("msg_delay_opt", 0);
        this->declare_parameter("delay_upper", 0);
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("msg_delay_opt", msg_delay_opt_);
        this->get_parameter("delay_upper", delay_upper_);
        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d", msg_delay_opt_, delay_upper_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic2::callback1, this, _1), sub_opt);
        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic2::callback2, this, _1), sub_opt);

        alg_sync_.registerCallback(std::bind(&SubscriberTopic2::alg_callback, this, _1, _2));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic2::mdl_callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d", period1, period2);

        alg_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        alg_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);

        alg_sync_.setAgePenalty(0);
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic2()
    {
    }

    private:
    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2)
    {
        if(count_alg_ > num_published_set_)
        {
            return;    
        }
        if(count_alg_ == num_published_set_)
        {
            cout << "Approximate Time Algorithm have got enough published sets !" << endl;
            if(count_alg_ == num_published_set_ && count_mdl_ == num_published_set_)
            {
                count_alg_++;
                kill(getpid(),SIGINT);
            }
            return;
        }

        count_alg_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        outfile_alg_ << topic1_timestamp << " " << topic2_timestamp << endl;
        double time_disparity = cal_time_disparity(2, topic1_timestamp, topic2_timestamp);

        if(time_disparity > alg_observerd_wctd_)
        {
            alg_observerd_wctd_ = time_disparity;
        }
    }

    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2)
    {
        if(count_mdl_ == num_published_set_)
        {
            cout << "Approximate Time Model have got enough published sets !" << endl;
            return;    
        }
        count_mdl_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        outfile_mdl_ << topic1_timestamp << " " << topic2_timestamp << endl;
        double time_disparity = cal_time_disparity(2, topic1_timestamp, topic2_timestamp);

        if(time_disparity > mdl_observed_wctd_)
        {
            mdl_observed_wctd_ = time_disparity;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous1_ <= delay_upper_ + period1_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous1_, period1_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;
        delay_previous1_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<0>(msg);
        mdl_sync_.add<0>(msg);

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous2_ <= delay_upper_ + period2_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous2_, period2_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous2_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<1>(msg);
        mdl_sync_.add<1>(msg);

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets

    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_;
};

class SubscriberTopic3
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic3(int period1, int period2, int period3, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic3"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), period1_(period1), period2_(period2), period3_(period3),
        count_alg_(0), count_mdl_(0), delay_previous1_(0.), delay_previous2_(0), delay_previous3_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0)
    {
        this->declare_parameter("msg_delay_opt", 0);
        this->declare_parameter("delay_upper", 0);
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("msg_delay_opt", msg_delay_opt_);
        this->get_parameter("delay_upper", delay_upper_);
        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d", msg_delay_opt_, delay_upper_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic3::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic3::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic3::callback3, this, _1), sub_opt);

        alg_sync_.registerCallback(std::bind(&SubscriberTopic3::alg_callback, this, _1, _2, _3));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic3::mdl_callback, this, _1, _2, _3));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d", period1, period2, period3);

        alg_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        alg_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        alg_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        
        alg_sync_.setAgePenalty(0);
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic3()
    {
    }

    private:
    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3)
    {
        if(count_alg_ > num_published_set_)
        {
            return;
        }

        if(count_alg_ == num_published_set_)
        {
            cout << "Approximate Time Algorithm have got enough published sets !" << endl;
            if(count_alg_ == num_published_set_ && count_mdl_ == num_published_set_)
            {
                count_alg_++;

                kill(getpid(),SIGINT);
            }
            return;
        }

        count_alg_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        outfile_alg_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << endl;
        
        double time_disparity = cal_time_disparity(3, topic1_timestamp, topic2_timestamp, topic3_timestamp);

        if(time_disparity > alg_observerd_wctd_)
        {
            alg_observerd_wctd_ = time_disparity;
        }
    }

    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3)
    {
        if(count_mdl_ == num_published_set_)
        {
            cout << "Approximate Time Model have got enough published sets !" << endl;
            return;    
        }

        count_mdl_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
                    
        outfile_mdl_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << endl;
        double time_disparity = cal_time_disparity(3, topic1_timestamp, topic2_timestamp, topic3_timestamp);

        if(time_disparity > mdl_observed_wctd_)
        {
            mdl_observed_wctd_ = time_disparity;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {   

            if(delay_previous1_ <= delay_upper_ + period1_)
            {
                if(msg_delay_opt_)
                {
                    delay_random_time(delay_previous1_, period1_, delay_upper_);
                }
            }

            rclcpp::Time now = this->now();
            rclcpp::Duration absolute_delay = now - msg->header.stamp;

            delay_previous1_ = absolute_delay.seconds() * Mstos;

            alg_sync_.add<0>(msg);
            mdl_sync_.add<0>(msg);

            double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

            if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
            {
                if(previous_timestamp1_ > 0)
                {
                    double observed_now = topic1_timestamp - previous_timestamp1_;
                    if(observed_now > observed_wcp1_)
                    {
                        observed_wcp1_ = observed_now;
                    }
                }
                previous_timestamp1_ = topic1_timestamp;
            }
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
            if(delay_previous2_ <= delay_upper_ + period2_)
            {
                if(msg_delay_opt_)
                {
                    delay_random_time(delay_previous2_, period2_, delay_upper_);
                }
            }

            rclcpp::Time now = this->now();
            rclcpp::Duration absolute_delay = now - msg->header.stamp;

            delay_previous2_ = absolute_delay.seconds() * Mstos;


            alg_sync_.add<1>(msg);
            mdl_sync_.add<1>(msg);

            double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

            if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
            {
                if(previous_timestamp2_ > 0)
                {
                    double observed_now = topic2_timestamp - previous_timestamp2_;
                    if(observed_now > observed_wcp2_)
                    {
                        observed_wcp2_ = observed_now;
                    }
                }
                previous_timestamp2_ = topic2_timestamp;
            }
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
            if(delay_previous3_ <= delay_upper_ + period3_)
            {
                if(msg_delay_opt_)
                {
                    delay_random_time(delay_previous3_, period3_, delay_upper_);
                }
            }

            rclcpp::Time now = this->now();
            rclcpp::Duration absolute_delay = now - msg->header.stamp;

            delay_previous3_ = absolute_delay.seconds() * Mstos;

            alg_sync_.add<2>(msg);
            mdl_sync_.add<2>(msg);

            double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

            if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
            {
                if(previous_timestamp3_ > 0)
                {
                    double observed_now = topic3_timestamp - previous_timestamp3_;
                    if(observed_now > observed_wcp3_)
                    {
                        observed_wcp3_ = observed_now;
                    }
                }
                previous_timestamp3_ = topic3_timestamp;
            }
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets    
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_;
};

class SubscriberTopic4
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic4(int period1, int period2, int period3, int period4, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic4"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl), 
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), 
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0)
    {
        this->declare_parameter("msg_delay_opt", 0);
        this->declare_parameter("delay_upper", 0);
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("msg_delay_opt", msg_delay_opt_);
        this->get_parameter("delay_upper", delay_upper_);
        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d", msg_delay_opt_, delay_upper_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic4::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic4::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic4::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic4::callback4, this, _1), sub_opt);

        alg_sync_.registerCallback(std::bind(&SubscriberTopic4::alg_callback, this, _1, _2, _3, _4));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic4::mdl_callback, this, _1, _2, _3, _4));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d",
                    period1, period2, period3, period4);

        alg_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        alg_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        alg_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        alg_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);

        alg_sync_.setAgePenalty(0);
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic4()
    {
    }

    private:
    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4)
    {
        if(count_alg_ > num_published_set_)
        {
            return;
        }

        if(count_alg_ == num_published_set_)
        {
            cout << "Approximate Time Algorithm have got enough published sets !" << endl;
            if(count_alg_ == num_published_set_ && count_mdl_ == num_published_set_)
            {
                count_alg_++;
                kill(getpid(),SIGINT);
            }
            return;
        }        
        count_alg_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        outfile_alg_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << endl;
        
        double time_disparity = cal_time_disparity(4, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp);
        if(time_disparity > alg_observerd_wctd_)
        {
            alg_observerd_wctd_ = time_disparity;
        }

    }

    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4)
    {
        if(count_mdl_ == num_published_set_)
        {
            cout << "Approximate Time Model have got enough published sets !" << endl;
            return;    
        }
        count_mdl_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        
        outfile_mdl_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << endl;
        double time_disparity = cal_time_disparity(4, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp);
        if(time_disparity > mdl_observed_wctd_)
        {
            mdl_observed_wctd_ = time_disparity;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {   
        if(delay_previous1_ <= delay_upper_ + period1_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous1_, period1_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous1_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<0>(msg);
        mdl_sync_.add<0>(msg);

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous2_ <= delay_upper_ + period2_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous2_, period2_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous2_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<1>(msg);
        mdl_sync_.add<1>(msg);

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous3_ <= delay_upper_ + period3_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous3_, period3_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous3_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<2>(msg);
        mdl_sync_.add<2>(msg);

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous4_ <= delay_upper_ + period4_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous4_, period4_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous4_ = absolute_delay.seconds() * Mstos;
        
        alg_sync_.add<3>(msg);
        mdl_sync_.add<3>(msg);

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic2_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic3_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic4_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_;

    int msg_delay_opt_; // Message delay options. 0: no delay,2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_;
};

class SubscriberTopic5
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic5(int period1, int period2, int period3, int period4, int period5, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic5"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0),
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0)
    {
        this->declare_parameter("msg_delay_opt", 0);
        this->declare_parameter("delay_upper", 0);
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("msg_delay_opt", msg_delay_opt_);
        this->get_parameter("delay_upper", delay_upper_);
        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d", msg_delay_opt_, delay_upper_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;
        
        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic5::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic5::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic5::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic5::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic5::callback5, this, _1), sub_opt);

        alg_sync_.registerCallback(std::bind(&SubscriberTopic5::alg_callback, this, _1, _2, _3, _4, _5));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic5::mdl_callback, this, _1, _2, _3, _4, _5));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d",
                    period1, period2, period3, period4, period5);

        alg_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        alg_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        alg_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        alg_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        alg_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        
        alg_sync_.setAgePenalty(0);
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic5()
    {
    }

    private:
    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5)
    {
        if(count_alg_ > num_published_set_)
        {
            return;
        }

        if(count_alg_ == num_published_set_)
        {
            cout << "Approximate Time Algorithm have got enough published sets !" << endl;
            if(count_alg_ == num_published_set_ && count_mdl_ == num_published_set_)
            {
                count_alg_++;
                kill(getpid(),SIGINT);
            }
            return;
        }        
        count_alg_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;

        outfile_alg_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << endl;        
        double time_disparity = cal_time_disparity(5, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp);
        if(time_disparity > alg_observerd_wctd_)
        {
            alg_observerd_wctd_ = time_disparity;
        }
    }

    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5)
    {
        if(count_mdl_ == num_published_set_)
        {
            cout << "Approximate Time Model have got enough published sets !" << endl;
            return;    
        }
        count_mdl_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;

        outfile_mdl_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << endl;
        double time_disparity = cal_time_disparity(5, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp);
        if(time_disparity > mdl_observed_wctd_)
        {
            mdl_observed_wctd_ = time_disparity;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous1_ <= delay_upper_ + period1_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous1_, period1_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous1_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<0>(msg);
        mdl_sync_.add<0>(msg);

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous2_ <= delay_upper_ + period2_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous2_, period2_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous2_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<1>(msg);
        mdl_sync_.add<1>(msg);

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous3_ <= delay_upper_ + period3_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous3_, period3_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous3_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<2>(msg);
        mdl_sync_.add<2>(msg);

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous4_ <= delay_upper_ + period4_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous4_, period4_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous4_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<3>(msg);
        mdl_sync_.add<3>(msg);

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        if(delay_previous5_ <= delay_upper_ + period5_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous5_, period5_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous5_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<4>(msg);
        mdl_sync_.add<4>(msg);

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets

    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_;
};

class SubscriberTopic6
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic6(int period1, int period2, int period3, int period4, int period5, int period6, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic6"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0), delay_previous6_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0), previous_timestamp6_(0), 
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0), observed_wcp6_(0)
    {
        this->declare_parameter("msg_delay_opt", 0);
        this->declare_parameter("delay_upper", 0);
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("msg_delay_opt", msg_delay_opt_);
        this->get_parameter("delay_upper", delay_upper_);
        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d", msg_delay_opt_, delay_upper_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic6::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic6::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic6::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic6::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic6::callback5, this, _1), sub_opt);

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic6::callback6, this, _1), sub_opt);

        alg_sync_.registerCallback(std::bind(&SubscriberTopic6::alg_callback, this, _1, _2, _3, _4, _5, _6));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic6::mdl_callback, this, _1, _2, _3, _4, _5, _6));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d",
                    period1, period2, period3, period4, period5, period6);

        alg_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        alg_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        alg_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        alg_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        alg_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        alg_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        
        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        mdl_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        
        alg_sync_.setAgePenalty(0);
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic6()
    {
    }

    private:
    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        if(count_alg_ > num_published_set_)
        {
            return;
        }

        if(count_alg_ == num_published_set_)
        {
            cout << "Approximate Time Algorithm have got enough published sets !" << endl;
            if(count_alg_ == num_published_set_ && count_mdl_ == num_published_set_)
            {
                count_alg_++;
                kill(getpid(),SIGINT);
            }
            return;
        }        
        count_alg_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;

        outfile_alg_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << endl;
        double time_disparity = cal_time_disparity(6, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp);

        if(time_disparity > alg_observerd_wctd_)
        {
            alg_observerd_wctd_ = time_disparity;
        }
    }

    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6)
    {
        if(count_mdl_ == num_published_set_)
        {
            cout << "Approximate Time Model have got enough published sets !" << endl;
            return;    
        }
        count_mdl_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;

        outfile_mdl_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << endl;
        double time_disparity = cal_time_disparity(6, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp);
        if(time_disparity > mdl_observed_wctd_)
        {
            mdl_observed_wctd_ = time_disparity;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        if(delay_previous1_ <= delay_upper_ + period1_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous1_, period1_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous1_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<0>(msg);
        mdl_sync_.add<0>(msg);

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous2_ <= delay_upper_ + period2_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous2_, period2_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous2_ = absolute_delay.seconds() * Mstos;


        alg_sync_.add<1>(msg);
        mdl_sync_.add<1>(msg);

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous3_ <= delay_upper_ + period3_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous3_, period3_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous3_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<2>(msg);
        mdl_sync_.add<2>(msg);

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous4_ <= delay_upper_ + period4_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous4_, period4_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous4_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<3>(msg);
        mdl_sync_.add<3>(msg);

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        if(delay_previous5_ <= delay_upper_ + period5_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous5_, period5_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous5_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<4>(msg);
        mdl_sync_.add<4>(msg);

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;
        
        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    void callback6(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous6_ <= delay_upper_ + period6_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous6_, period6_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous6_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<5>(msg);
        mdl_sync_.add<5>(msg);

        double topic6_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp6_ > 0)
            {
                double observed_now = topic6_timestamp - previous_timestamp6_;
                if(observed_now > observed_wcp6_)
                {
                    observed_wcp6_ = observed_now;
                }
            }
            previous_timestamp6_ = topic6_timestamp;
        }
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_, delay_previous6_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_, previous_timestamp6_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_, observed_wcp6_;
};

class SubscriberTopic7
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic7(int period1, int period2, int period3, int period4, int period5, int period6, int period7, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic7"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0), delay_previous6_(0), delay_previous7_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0), previous_timestamp6_(0), previous_timestamp7_(0),
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0), observed_wcp6_(0), observed_wcp7_(0)
    {
        this->declare_parameter("msg_delay_opt", 0);
        this->declare_parameter("delay_upper", 0);
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("msg_delay_opt", msg_delay_opt_);
        this->get_parameter("delay_upper", delay_upper_);
        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d", msg_delay_opt_, delay_upper_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic7::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic7::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic7::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic7::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic7::callback5, this, _1), sub_opt);

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic7::callback6, this, _1), sub_opt);

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic7::callback7, this, _1), sub_opt);

        alg_sync_.registerCallback(std::bind(&SubscriberTopic7::alg_callback, this, _1, _2, _3, _4, _5, _6, _7));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic7::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d",
                    period1, period2, period3, period4, period5, period6, period7);

        alg_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        alg_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        alg_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        alg_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        alg_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        alg_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        alg_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        mdl_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        mdl_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        
        alg_sync_.setAgePenalty(0);
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic7()
    {
    }

    private:
    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7)
    {
        if(count_alg_ > num_published_set_)
        {
            return;
        }

        if(count_alg_ == num_published_set_)
        {
            cout << "Approximate Time Algorithm have got enough published sets !" << endl;
            if(count_alg_ == num_published_set_ && count_mdl_ == num_published_set_)
            {
                count_alg_++;
                kill(getpid(),SIGINT);
            }
            return;
        }        
        count_alg_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;

        outfile_alg_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << endl;
        double time_disparity = cal_time_disparity(7, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp);
        if(time_disparity > alg_observerd_wctd_)
        {
            alg_observerd_wctd_ = time_disparity;
        }
    }

    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7)
    {
        if(count_mdl_ == num_published_set_)
        {
            cout << "Approximate Time Model have got enough published sets !" << endl;
            return;    
        }
        count_mdl_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;

        outfile_mdl_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << endl;
        double time_disparity = cal_time_disparity(7, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp);
        if(time_disparity > mdl_observed_wctd_)
        {
            mdl_observed_wctd_ = time_disparity;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        if(delay_previous1_ <= delay_upper_ + period1_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous1_, period1_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous1_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<0>(msg);
        mdl_sync_.add<0>(msg);

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous2_ <= delay_upper_ + period2_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous2_, period2_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        alg_sync_.add<1>(msg);     
        mdl_sync_.add<1>(msg);

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous3_ <= delay_upper_ + period3_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous3_, period3_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous3_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<2>(msg);
        mdl_sync_.add<2>(msg);

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous4_ <= delay_upper_ + period4_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous4_, period4_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        alg_sync_.add<3>(msg);
        mdl_sync_.add<3>(msg);

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        if(delay_previous5_ <= delay_upper_ + period5_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous5_, period5_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous5_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<4>(msg);
        mdl_sync_.add<4>(msg);

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    void callback6(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous6_ <= delay_upper_ + period6_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous6_, period6_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous6_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<5>(msg);
        mdl_sync_.add<5>(msg);

        double topic6_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp6_ > 0)
            {
                double observed_now = topic6_timestamp - previous_timestamp6_;
                if(observed_now > observed_wcp6_)
                {
                    observed_wcp6_ = observed_now;
                }
            }
            previous_timestamp6_ = topic6_timestamp;
        }
    }

    void callback7(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous7_ <= delay_upper_ + period7_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous7_, period7_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous7_ = absolute_delay.seconds() * Mstos;  

        alg_sync_.add<6>(msg);
        mdl_sync_.add<6>(msg);

        double topic7_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp7_ > 0)
            {
                double observed_now = topic7_timestamp - previous_timestamp7_;
                if(observed_now > observed_wcp7_)
                {
                    observed_wcp7_ = observed_now;
                }
            }
            previous_timestamp7_ = topic7_timestamp;
        }
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_, delay_previous6_, delay_previous7_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_, previous_timestamp6_, previous_timestamp7_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_, observed_wcp6_, observed_wcp7_;
};

class SubscriberTopic8
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic8(int period1, int period2, int period3, int period4, int period5, int period6, int period7, int period8, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic8"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl),
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7), period8_(period8),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0), delay_previous6_(0), delay_previous7_(0), delay_previous8_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0), previous_timestamp6_(0), previous_timestamp7_(0), previous_timestamp8_(0),
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0), observed_wcp6_(0), observed_wcp7_(0), observed_wcp8_(0)
    {
        this->declare_parameter("msg_delay_opt", 0);
        this->declare_parameter("delay_upper", 0);
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("msg_delay_opt", msg_delay_opt_);
        this->get_parameter("delay_upper", delay_upper_);
        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d", msg_delay_opt_, delay_upper_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic8::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic8::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic8::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic8::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic8::callback5, this, _1), sub_opt);

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic8::callback6, this, _1), sub_opt);

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic8::callback7, this, _1), sub_opt);

        topic8_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic8::callback8, this, _1), sub_opt);

        alg_sync_.registerCallback(std::bind(&SubscriberTopic8::alg_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic8::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7, _8));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d, Period 8: %d",
                    period1, period2, period3, period4, period5, period6, period7, period8);

        alg_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        alg_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        alg_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        alg_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        alg_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        alg_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        alg_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        alg_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        mdl_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        mdl_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        mdl_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);
        
        alg_sync_.setAgePenalty(0);
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic8()
    {
    }

    private:
    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8)
    {
        if(count_alg_ > num_published_set_)
        {
            return;
        }

        if(count_alg_ == num_published_set_)
        {
            cout << "Approximate Time Algorithm have got enough published sets !" << endl;
            if(count_alg_ == num_published_set_ && count_mdl_ == num_published_set_)
            {
                count_alg_++;
                kill(getpid(),SIGINT);
            }
            return;
        }        
        count_alg_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;
        double topic8_timestamp = (double)msg8->header.stamp.sec + 1e-9*(double)msg8->header.stamp.nanosec;

        outfile_alg_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << " " << topic8_timestamp << endl;
        double time_disparity = cal_time_disparity(8, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp, topic8_timestamp);
        if(time_disparity > alg_observerd_wctd_)
        {
            alg_observerd_wctd_ = time_disparity;
        }
    }

    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8)
    {
        if(count_mdl_ == num_published_set_)
        {
            cout << "Approximate Time Model have got enough published sets !" << endl;
            return;    
        }
        count_mdl_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;
        double topic8_timestamp = (double)msg8->header.stamp.sec + 1e-9*(double)msg8->header.stamp.nanosec;

        outfile_mdl_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << " " << topic8_timestamp << endl;
        double time_disparity = cal_time_disparity(8, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp, topic8_timestamp);
        if(time_disparity > mdl_observed_wctd_)
        {
            mdl_observed_wctd_ = time_disparity;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        if(delay_previous1_ <= delay_upper_ + period1_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous1_, period1_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous1_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<0>(msg);
        mdl_sync_.add<0>(msg);

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous2_ <= delay_upper_ + period2_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous2_, period2_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        alg_sync_.add<1>(msg);
        mdl_sync_.add<1>(msg);

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }        
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous3_ <= delay_upper_ + period3_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous3_, period3_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous3_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<2>(msg);
        mdl_sync_.add<2>(msg);

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }        
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous4_ <= delay_upper_ + period4_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous4_, period4_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        alg_sync_.add<3>(msg);
        mdl_sync_.add<3>(msg);

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {    
        if(delay_previous5_ <= delay_upper_ + period5_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous5_, period5_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous5_ = absolute_delay.seconds() * Mstos;  


        alg_sync_.add<4>(msg);
        mdl_sync_.add<4>(msg);

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }        
    }

    void callback6(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous6_ <= delay_upper_ + period6_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous6_, period6_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous6_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<5>(msg);
        mdl_sync_.add<5>(msg);

        double topic6_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp6_ > 0)
            {
                double observed_now = topic6_timestamp - previous_timestamp6_;
                if(observed_now > observed_wcp6_)
                {
                    observed_wcp6_ = observed_now;
                }
            }
            previous_timestamp6_ = topic6_timestamp;
        }        
    }

    void callback7(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous7_ <= delay_upper_ + period7_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous7_, period7_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous7_ = absolute_delay.seconds() * Mstos;  

        alg_sync_.add<6>(msg);
        mdl_sync_.add<6>(msg);

        double topic7_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp7_ > 0)
            {
                double observed_now = topic7_timestamp - previous_timestamp7_;
                if(observed_now > observed_wcp7_)
                {
                    observed_wcp7_ = observed_now;
                }
            }
            previous_timestamp7_ = topic7_timestamp;
        }
    }

    void callback8(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous8_ <= delay_upper_ + period8_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous8_, period8_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous8_ = absolute_delay.seconds() * Mstos;       

        alg_sync_.add<7>(msg);
        mdl_sync_.add<7>(msg);

        double topic8_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp8_ > 0)
            {
                double observed_now = topic8_timestamp - previous_timestamp8_;
                if(observed_now > observed_wcp8_)
                {
                    observed_wcp8_ = observed_now;
                }
            }
            previous_timestamp8_ = topic8_timestamp;
        }        
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_, topic8_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_, period8_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
    
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_, delay_previous6_, delay_previous7_, delay_previous8_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_, previous_timestamp6_, previous_timestamp7_, previous_timestamp8_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_, observed_wcp6_, observed_wcp7_, observed_wcp8_;
};

class SubscriberTopic9
    : public rclcpp::Node
{
    ofstream& outfile_alg_;
    ofstream& outfile_mdl_;

    public:
    SubscriberTopic9(int period1, int period2, int period3, int period4, int period5, int period6, int period7, int period8, int period9, std::ofstream& outfile_alg, std::ofstream& outfile_mdl) :
        Node("subscriber_topic9"), outfile_alg_(outfile_alg), outfile_mdl_(outfile_mdl), 
        alg_sync_(buffer_size), mdl_sync_(), 
        period1_(period1), period2_(period2), period3_(period3), period4_(period4), period5_(period5), period6_(period6), period7_(period7), period8_(period8), period9_(period9),
        count_alg_(0), count_mdl_(0), 
        delay_previous1_(0), delay_previous2_(0), delay_previous3_(0), delay_previous4_(0), delay_previous5_(0), delay_previous6_(0), delay_previous7_(0), delay_previous8_(0), delay_previous9_(0),
        alg_observerd_wctd_(0), mdl_observed_wctd_(0), 
        previous_timestamp1_(0), previous_timestamp2_(0), previous_timestamp3_(0), previous_timestamp4_(0), previous_timestamp5_(0), previous_timestamp6_(0), previous_timestamp7_(0), previous_timestamp8_(0), previous_timestamp9_(0), 
        observed_wcp1_(0), observed_wcp2_(0), observed_wcp3_(0), observed_wcp4_(0), observed_wcp5_(0), observed_wcp6_(0), observed_wcp7_(0), observed_wcp8_(0), observed_wcp9_(0)
    {
        this->declare_parameter("msg_delay_opt", 0);
        this->declare_parameter("delay_upper", 0);
        this->declare_parameter("num_published_set", 1000);

        this->get_parameter("msg_delay_opt", msg_delay_opt_);
        this->get_parameter("delay_upper", delay_upper_);
        this->get_parameter("num_published_set", num_published_set_);

        RCLCPP_INFO(this->get_logger(), "Delay option has been set as: %d, with delay upper limit: %d", msg_delay_opt_, delay_upper_);

        callback_group_subscriber_ = this->create_callback_group(
        rclcpp::callback_group::CallbackGroupType::Reentrant);

        // Each of these callback groups is basically a thread
        // Everything assigned to one of them gets bundled into the same thread
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_subscriber_;

        topic1_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic1", 300, 
                                                                        std::bind(&SubscriberTopic9::callback1, this, _1), sub_opt);

        topic2_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic2", 300, 
                                                                        std::bind(&SubscriberTopic9::callback2, this, _1), sub_opt);

        topic3_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic3", 300, 
                                                                        std::bind(&SubscriberTopic9::callback3, this, _1), sub_opt);

        topic4_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic4", 300, 
                                                                        std::bind(&SubscriberTopic9::callback4, this, _1), sub_opt);

        topic5_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic5", 300, 
                                                                        std::bind(&SubscriberTopic9::callback5, this, _1), sub_opt);

        topic6_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic6", 300, 
                                                                        std::bind(&SubscriberTopic9::callback6, this, _1), sub_opt);

        topic7_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic7", 300, 
                                                                        std::bind(&SubscriberTopic9::callback7, this, _1), sub_opt);

        topic8_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic9::callback8, this, _1), sub_opt);

        topic9_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("topic8", 300, 
                                                                        std::bind(&SubscriberTopic9::callback9, this, _1), sub_opt);

        alg_sync_.registerCallback(std::bind(&SubscriberTopic9::alg_callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));
        mdl_sync_.registerCallback(std::bind(&SubscriberTopic9::mdl_callback, this, _1, _2, _3, _4, _5, _6, _7, _8, _9));

        RCLCPP_INFO(this->get_logger(), "Period 1: %d, Period 2: %d, Period 3: %d, Period 4: %d, Period 5: %d, Period 6: %d, Period 7: %d, Period 8: %d, Period 9: %d",
                    period1, period2, period3, period4, period5, period6, period7, period8, period9);

        alg_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        alg_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        alg_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        alg_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        alg_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        alg_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        alg_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        alg_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);
        alg_sync_.setInterMessageLowerBound(8, PeriodBase * 0.001 * period9);

        mdl_sync_.setInterMessageLowerBound(0, PeriodBase * 0.001 * period1);
        mdl_sync_.setInterMessageLowerBound(1, PeriodBase * 0.001 * period2);
        mdl_sync_.setInterMessageLowerBound(2, PeriodBase * 0.001 * period3);
        mdl_sync_.setInterMessageLowerBound(3, PeriodBase * 0.001 * period4);
        mdl_sync_.setInterMessageLowerBound(4, PeriodBase * 0.001 * period5);
        mdl_sync_.setInterMessageLowerBound(5, PeriodBase * 0.001 * period6);
        mdl_sync_.setInterMessageLowerBound(6, PeriodBase * 0.001 * period7);
        mdl_sync_.setInterMessageLowerBound(7, PeriodBase * 0.001 * period8);
        mdl_sync_.setInterMessageLowerBound(8, PeriodBase * 0.001 * period9);
        
        alg_sync_.setAgePenalty(0);
        mdl_sync_.setAgePenalty(0);
    }

    ~SubscriberTopic9()
    {
    }

    private:
    void alg_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg9)
    {
        if(count_alg_ > num_published_set_)
        {
            return;
        }

        if(count_alg_ == num_published_set_)
        {
            cout << "Approximate Time Algorithm have got enough published sets !" << endl;
            if(count_alg_ == num_published_set_ && count_mdl_ == num_published_set_)
            {
                count_alg_++;
                kill(getpid(),SIGINT);
            }
            return;
        }        
        count_alg_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;
        double topic8_timestamp = (double)msg8->header.stamp.sec + 1e-9*(double)msg8->header.stamp.nanosec;
        double topic9_timestamp = (double)msg9->header.stamp.sec + 1e-9*(double)msg9->header.stamp.nanosec;

        outfile_alg_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << " " << topic8_timestamp << " " << topic9_timestamp << endl;
        double time_disparity = cal_time_disparity(9, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp, topic8_timestamp, topic9_timestamp);
        if(time_disparity > alg_observerd_wctd_)
        {
            alg_observerd_wctd_ = time_disparity;
        }
    }

    void mdl_callback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg1, const sensor_msgs::msg::JointState::ConstSharedPtr& msg2, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg3, const sensor_msgs::msg::JointState::ConstSharedPtr& msg4,
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg5, const sensor_msgs::msg::JointState::ConstSharedPtr& msg6, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg7, const sensor_msgs::msg::JointState::ConstSharedPtr& msg8, 
                            const sensor_msgs::msg::JointState::ConstSharedPtr& msg9)
    {
        if(count_mdl_ == num_published_set_)
        {
            cout << "Approximate Time Model have got enough published sets !" << endl;
            return;    
        }
        count_mdl_++;

        double topic1_timestamp = (double)msg1->header.stamp.sec + 1e-9*(double)msg1->header.stamp.nanosec;
        double topic2_timestamp = (double)msg2->header.stamp.sec + 1e-9*(double)msg2->header.stamp.nanosec;
        double topic3_timestamp = (double)msg3->header.stamp.sec + 1e-9*(double)msg3->header.stamp.nanosec;
        double topic4_timestamp = (double)msg4->header.stamp.sec + 1e-9*(double)msg4->header.stamp.nanosec;
        double topic5_timestamp = (double)msg5->header.stamp.sec + 1e-9*(double)msg5->header.stamp.nanosec;
        double topic6_timestamp = (double)msg6->header.stamp.sec + 1e-9*(double)msg6->header.stamp.nanosec;
        double topic7_timestamp = (double)msg7->header.stamp.sec + 1e-9*(double)msg7->header.stamp.nanosec;
        double topic8_timestamp = (double)msg8->header.stamp.sec + 1e-9*(double)msg8->header.stamp.nanosec;
        double topic9_timestamp = (double)msg9->header.stamp.sec + 1e-9*(double)msg9->header.stamp.nanosec;
        
        outfile_mdl_ << topic1_timestamp << " " << topic2_timestamp << " " << topic3_timestamp << " " << topic4_timestamp << " " << topic5_timestamp << " " << topic6_timestamp << " " << topic7_timestamp << " " << topic8_timestamp << " " << topic9_timestamp << endl;
        double time_disparity = cal_time_disparity(9, topic1_timestamp, topic2_timestamp, topic3_timestamp, topic4_timestamp, topic5_timestamp, topic6_timestamp, topic7_timestamp, topic8_timestamp, topic9_timestamp);
        if(time_disparity > mdl_observed_wctd_)
        {
            mdl_observed_wctd_ = time_disparity;
        }
    }

    void callback1(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {       
        if(delay_previous1_ <= delay_upper_ + period1_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous1_, period1_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous1_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<0>(msg);
        mdl_sync_.add<0>(msg);

        double topic1_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp1_ > 0)
            {
                double observed_now = topic1_timestamp - previous_timestamp1_;
                if(observed_now > observed_wcp1_)
                {
                    observed_wcp1_ = observed_now;
                }
            }
            previous_timestamp1_ = topic1_timestamp;
        }
    }

    void callback2(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous2_ <= delay_upper_ + period2_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous2_, period2_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous2_ = absolute_delay.seconds() * Mstos;   

        alg_sync_.add<1>(msg);
        mdl_sync_.add<1>(msg);

        double topic2_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp2_ > 0)
            {
                double observed_now = topic2_timestamp - previous_timestamp2_;
                if(observed_now > observed_wcp2_)
                {
                    observed_wcp2_ = observed_now;
                }
            }
            previous_timestamp2_ = topic2_timestamp;
        }
    }

    void callback3(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous3_ <= delay_upper_ + period3_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous3_, period3_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous3_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<2>(msg);
        mdl_sync_.add<2>(msg);

        double topic3_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp3_ > 0)
            {
                double observed_now = topic3_timestamp - previous_timestamp3_;
                if(observed_now > observed_wcp3_)
                {
                    observed_wcp3_ = observed_now;
                }
            }
            previous_timestamp3_ = topic3_timestamp;
        }
    }

    void callback4(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous4_ <= delay_upper_ + period4_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous4_, period4_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous4_ = absolute_delay.seconds() * Mstos; 

        alg_sync_.add<3>(msg);
        mdl_sync_.add<3>(msg);

        double topic4_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp4_ > 0)
            {
                double observed_now = topic4_timestamp - previous_timestamp4_;
                if(observed_now > observed_wcp4_)
                {
                    observed_wcp4_ = observed_now;
                }
            }
            previous_timestamp4_ = topic4_timestamp;
        }
    }

    void callback5(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {      
        if(delay_previous5_ <= delay_upper_ + period5_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous5_, period5_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous5_ = absolute_delay.seconds() * Mstos;
            
        alg_sync_.add<4>(msg);
        mdl_sync_.add<4>(msg);

        double topic5_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp5_ > 0)
            {
                double observed_now = topic5_timestamp - previous_timestamp5_;
                if(observed_now > observed_wcp5_)
                {
                    observed_wcp5_ = observed_now;
                }
            }
            previous_timestamp5_ = topic5_timestamp;
        }
    }

    void callback6(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous6_ <= delay_upper_ + period6_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous6_, period6_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous6_ = absolute_delay.seconds() * Mstos;

        alg_sync_.add<5>(msg);
        mdl_sync_.add<5>(msg);

        double topic6_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp6_ > 0)
            {
                double observed_now = topic6_timestamp - previous_timestamp6_;
                if(observed_now > observed_wcp6_)
                {
                    observed_wcp6_ = observed_now;
                }
            }
            previous_timestamp6_ = topic6_timestamp;
        }
    }

    void callback7(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous7_ <= delay_upper_ + period7_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous7_, period7_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous7_ = absolute_delay.seconds() * Mstos;  

        alg_sync_.add<6>(msg);
        mdl_sync_.add<6>(msg);

        double topic7_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp7_ > 0)
            {
                double observed_now = topic7_timestamp - previous_timestamp7_;
                if(observed_now > observed_wcp7_)
                {
                    observed_wcp7_ = observed_now;
                }
            }
            previous_timestamp7_ = topic7_timestamp;
        }
    }

    void callback8(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous8_ <= delay_upper_ + period8_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous8_, period8_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous8_ = absolute_delay.seconds() * Mstos;      

        alg_sync_.add<7>(msg);      
        mdl_sync_.add<7>(msg);

        double topic8_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp8_ > 0)
            {
                double observed_now = topic8_timestamp - previous_timestamp8_;
                if(observed_now > observed_wcp8_)
                {
                    observed_wcp8_ = observed_now;
                }
            }
            previous_timestamp8_ = topic8_timestamp;
        }
    }

    void callback9(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if(delay_previous9_ <= delay_upper_ + period9_)
        {
            if(msg_delay_opt_)
            {
                delay_random_time(delay_previous9_, period9_, delay_upper_);
            }
        }

        rclcpp::Time now = this->now();
        rclcpp::Duration absolute_delay = now - msg->header.stamp;

        delay_previous9_ = absolute_delay.seconds() * Mstos;      

        alg_sync_.add<8>(msg);       
        mdl_sync_.add<8>(msg);

        double topic9_timestamp = (double)msg->header.stamp.sec + 1e-9*(double)msg->header.stamp.nanosec;

        if(count_alg_ < num_published_set_ || count_mdl_ < num_published_set_)
        {
            if(previous_timestamp9_ > 0)
            {
                double observed_now = topic9_timestamp - previous_timestamp9_;
                if(observed_now > observed_wcp9_)
                {
                    observed_wcp9_ = observed_now;
                }
            }
            previous_timestamp9_ = topic9_timestamp;
        }
    }

    rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic1_sub_, topic2_sub_, topic3_sub_, topic4_sub_, topic5_sub_, topic6_sub_, topic7_sub_, topic8_sub_, topic9_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState, sensor_msgs::msg::JointState> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    int period1_, period2_, period3_, period4_, period5_, period6_, period7_, period8_, period9_;

    int msg_delay_opt_; // Message delay options. 0: no delay, 2: delay 0-upper;
    int delay_upper_;   // Message delay upper limit (0, 10, 20, 30, 40)
    int num_published_set_; // Required number of published sets
        
    int count_alg_, count_mdl_;
    double delay_previous1_, delay_previous2_, delay_previous3_, delay_previous4_, delay_previous5_, delay_previous6_, delay_previous7_, delay_previous8_, delay_previous9_;

    double alg_observerd_wctd_, mdl_observed_wctd_;
    double previous_timestamp1_, previous_timestamp2_, previous_timestamp3_, previous_timestamp4_, previous_timestamp5_, previous_timestamp6_, previous_timestamp7_, previous_timestamp8_, previous_timestamp9_;
    double observed_wcp1_, observed_wcp2_, observed_wcp3_, observed_wcp4_, observed_wcp5_, observed_wcp6_, observed_wcp7_, observed_wcp8_, observed_wcp9_;
};
}
