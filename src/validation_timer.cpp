#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <string> 

#include "publisher.h"
#include "subscriber_validation.h"

using namespace synchronizer;
using namespace std;

int main(int argc, char * argv[])
{
    string topic_name;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    default_random_engine gen(seed);

    ////////////////////////////////////////////////////////////////////////////////////////
    int channel_num = atoi(argv[1]);
    int lower_limit = atoi(argv[2]);
    int test_num = atoi(argv[3]);
    string result_path = argv[4];
    
    uniform_int_distribution<unsigned> perd(lower_limit, 100);
    ////////////////////////////////////////////////////////////////////////////////////////
    ofstream outfile_alg_;
    ofstream outfile_mdl_;
    
    outfile_alg_.open(result_path + "/timestamp_alg.txt", ios::out);
    if (!outfile_alg_.is_open()) 
    {
        cout<<"Error opening file timestamp_alg.txt! "<<endl;
    }

    // set output accuracy
    outfile_alg_.setf(ios::fixed, ios::floatfield);
    outfile_alg_.precision(9);

    outfile_mdl_.open(result_path + "/timestamp_mdl.txt", ios::out);
    if (!outfile_mdl_.is_open()) 
    {
        cout<<"Error opening file timestamp_mdl.txt! "<<endl;
    }

    // set output accuracy
    outfile_mdl_.setf(ios::fixed, ios::floatfield);
    outfile_mdl_.precision(9);

    switch(channel_num)
    {
    case 2:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);
            
            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            auto sub_node = std::make_shared<SubscriberTopic2>(real_period1, real_period2, outfile_alg_, outfile_mdl_);
            executor.add_node(sub_node);

            executor.spin();

            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 3:
    {   
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            auto sub_node = std::make_shared<SubscriberTopic3>(real_period1, real_period2, real_period3, outfile_alg_, outfile_mdl_);
            executor.add_node(sub_node);

            executor.spin();
            
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }

        break;
    }
    case 4:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            auto sub_node = std::make_shared<SubscriberTopic4>(real_period1, real_period2, real_period3, real_period4, outfile_alg_, outfile_mdl_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 5:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            auto sub_node = std::make_shared<SubscriberTopic5>(real_period1, real_period2, real_period3, real_period4, real_period5, outfile_alg_, outfile_mdl_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 6:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            int real_period6 = perd(gen);
            topic_name = "topic" + to_string(6);
            auto pub_node6 = std::make_shared<Publisher>(topic_name, real_period6);
            executor.add_node(pub_node6);

            auto sub_node = std::make_shared<SubscriberTopic6>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, outfile_alg_, outfile_mdl_);
            executor.add_node(sub_node);

            executor.spin();

            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(pub_node6);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 7:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            int real_period6 = perd(gen);
            topic_name = "topic" + to_string(6);
            auto pub_node6 = std::make_shared<Publisher>(topic_name, real_period6);
            executor.add_node(pub_node6);

            int real_period7 = perd(gen);
            topic_name = "topic" + to_string(7);
            auto pub_node7 = std::make_shared<Publisher>(topic_name, real_period7);
            executor.add_node(pub_node7);

            auto sub_node = std::make_shared<SubscriberTopic7>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, outfile_alg_, outfile_mdl_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(pub_node6);
            executor.remove_node(pub_node7);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }
        break;
    }
    case 8:
    {
        int count = 0;
        while(count < test_num)
        {
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            int real_period6 = perd(gen);
            topic_name = "topic" + to_string(6);
            auto pub_node6 = std::make_shared<Publisher>(topic_name, real_period6);
            executor.add_node(pub_node6);

            int real_period7 = perd(gen);
            topic_name = "topic" + to_string(7);
            auto pub_node7 = std::make_shared<Publisher>(topic_name, real_period7);
            executor.add_node(pub_node7);

            int real_period8 = perd(gen);
            topic_name = "topic" + to_string(8);
            auto pub_node8 = std::make_shared<Publisher>(topic_name, real_period8);
            executor.add_node(pub_node8);

            auto sub_node = std::make_shared<SubscriberTopic8>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, real_period8, outfile_alg_, outfile_mdl_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(pub_node6);
            executor.remove_node(pub_node7);
            executor.remove_node(pub_node8);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }            
        break;
    }
    case 9:
    {
        int count = 0;
        while(count < test_num)
        {        
            rclcpp::init(argc, argv);
            // rclcpp::executors::SingleThreadedExecutor executor;
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(),50);

            int real_period1 = perd(gen);
            topic_name = "topic" + to_string(1);
            auto pub_node1 = std::make_shared<Publisher>(topic_name, real_period1);
            executor.add_node(pub_node1);

            int real_period2 = perd(gen);
            topic_name = "topic" + to_string(2);
            auto pub_node2 = std::make_shared<Publisher>(topic_name, real_period2);
            executor.add_node(pub_node2);

            int real_period3 = perd(gen);
            topic_name = "topic" + to_string(3);
            auto pub_node3 = std::make_shared<Publisher>(topic_name, real_period3);
            executor.add_node(pub_node3);

            int real_period4 = perd(gen);
            topic_name = "topic" + to_string(4);
            auto pub_node4 = std::make_shared<Publisher>(topic_name, real_period4);
            executor.add_node(pub_node4);

            int real_period5 = perd(gen);
            topic_name = "topic" + to_string(5);
            auto pub_node5 = std::make_shared<Publisher>(topic_name, real_period5);
            executor.add_node(pub_node5);

            int real_period6 = perd(gen);
            topic_name = "topic" + to_string(6);
            auto pub_node6 = std::make_shared<Publisher>(topic_name, real_period6);
            executor.add_node(pub_node6);

            int real_period7 = perd(gen);
            topic_name = "topic" + to_string(7);
            auto pub_node7 = std::make_shared<Publisher>(topic_name, real_period7);
            executor.add_node(pub_node7);

            int real_period8 = perd(gen);
            topic_name = "topic" + to_string(8);
            auto pub_node8 = std::make_shared<Publisher>(topic_name, real_period8);
            executor.add_node(pub_node8);

            int real_period9 = perd(gen);
            topic_name = "topic" + to_string(9);
            auto pub_node9 = std::make_shared<Publisher>(topic_name, real_period9);
            executor.add_node(pub_node9);

            auto sub_node = std::make_shared<SubscriberTopic9>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, real_period8, real_period9, outfile_alg_, outfile_mdl_);
            executor.add_node(sub_node);

            executor.spin();
            executor.remove_node(pub_node1);
            executor.remove_node(pub_node2);
            executor.remove_node(pub_node3);
            executor.remove_node(pub_node4);
            executor.remove_node(pub_node5);
            executor.remove_node(pub_node6);
            executor.remove_node(pub_node7);
            executor.remove_node(pub_node8);
            executor.remove_node(pub_node9);
            executor.remove_node(sub_node);

            cout << " We have finished test " << count + 1 << endl;
            count++;
        }            
        break;
    }
    default:
        break;        
    }

    rclcpp::shutdown();
    return 0;
}
