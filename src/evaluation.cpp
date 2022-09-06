#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <string> 

#include "publisher.h"
#include "subscriber_evaluation.h"

using namespace synchronizer;
using namespace std;

ofstream outfile_lat_;
ofstream outfile_mdl_;
ofstream outfile_topic1_;
ofstream outfile_topic2_;
ofstream outfile_topic3_;
ofstream outfile_topic4_;
ofstream outfile_topic5_;
ofstream outfile_topic6_;
ofstream outfile_topic7_;
ofstream outfile_topic8_;
ofstream outfile_topic9_;

bool open_file(int topic_num, string result_path)
{
    outfile_lat_.open(result_path + "/timestamp_lat.txt", ios::out);
    if (!outfile_lat_.is_open()) 
    {
        cout<<"Error opening file timestamp_lat.txt! "<<endl;
    }

    // set output accuracy
    outfile_lat_.setf(ios::fixed, ios::floatfield);
    outfile_lat_.precision(9);

    outfile_mdl_.open(result_path + "/timestamp_mdl.txt", ios::out);
    if (!outfile_mdl_.is_open()) 
    {
        cout<<"Error opening file timestamp_mdl.txt! "<<endl;
    }

    // set output accuracy
    outfile_mdl_.setf(ios::fixed, ios::floatfield);
    outfile_mdl_.precision(9);

    outfile_topic1_.open(result_path + "/topic1_timestamp.txt", ios::out);
    if (!outfile_topic1_.is_open()) 
    {
        cout<<"Error opening file topic1_timestamp.txt! "<<endl;
        return false;
    }

    // set output accuracy
    outfile_topic1_.setf(ios::fixed, ios::floatfield);
    outfile_topic1_.precision(9);

    outfile_topic2_.open(result_path + "/topic2_timestamp.txt", ios::out);
    if (!outfile_topic2_.is_open()) 
    {
        cout<<"Error opening file topic2_timestamp.txt! "<<endl;
        return false;
    }

    // set output accuracy
    outfile_topic2_.setf(ios::fixed, ios::floatfield);
    outfile_topic2_.precision(9);

    if(topic_num > 2)
    {
        outfile_topic3_.open(result_path + "/topic3_timestamp.txt", ios::out);
        if (!outfile_topic3_.is_open()) 
        {
            cout<<"Error opening file topic3_timestamp.txt! "<<endl;
            return false;
        }

        // set output accuracy
        outfile_topic3_.setf(ios::fixed, ios::floatfield);
        outfile_topic3_.precision(9);
        if(topic_num > 3)
        {
            outfile_topic4_.open(result_path + "/topic4_timestamp.txt", ios::out);
            if (!outfile_topic4_.is_open()) 
            {
                cout<<"Error opening file topic4_timestamp.txt! "<<endl;
                return false;
            }

            // set output accuracy
            outfile_topic4_.setf(ios::fixed, ios::floatfield);
            outfile_topic4_.precision(9);
            if(topic_num > 4)
            {
                outfile_topic5_.open(result_path + "/topic5_timestamp.txt", ios::out);
                if (!outfile_topic5_.is_open()) 
                {
                    cout<<"Error opening file topic5_timestamp.txt! "<<endl;
                    return false;
                }

                // set output accuracy
                outfile_topic5_.setf(ios::fixed, ios::floatfield);
                outfile_topic5_.precision(9);
                if(topic_num > 5)
                {
                    outfile_topic6_.open(result_path + "/topic6_timestamp.txt", ios::out);
                    if (!outfile_topic6_.is_open()) 
                    {
                        cout<<"Error opening file topic6_timestamp.txt! "<<endl;
                        return false;
                    }

                    // set output accuracy
                    outfile_topic6_.setf(ios::fixed, ios::floatfield);
                    outfile_topic6_.precision(9);
                    if(topic_num > 6)
                    {
                        outfile_topic7_.open(result_path + "/topic7_timestamp.txt", ios::out);
                        if (!outfile_topic7_.is_open()) 
                        {
                            cout<<"Error opening file topic7_timestamp.txt! "<<endl;
                            return false;
                        }

                        // set output accuracy
                        outfile_topic7_.setf(ios::fixed, ios::floatfield);
                        outfile_topic7_.precision(9);
                        if(topic_num > 7)
                        {
                            outfile_topic8_.open(result_path + "/topic8_timestamp.txt", ios::out);
                            if (!outfile_topic8_.is_open()) 
                            {
                                cout<<"Error opening file topic8_timestamp.txt! "<<endl;
                                return false;
                            }

                            // set output accuracy
                            outfile_topic8_.setf(ios::fixed, ios::floatfield);
                            outfile_topic8_.precision(9);
                            if(topic_num > 8)
                            {
                                outfile_topic9_.open(result_path + "/topic9_timestamp.txt", ios::out);
                                if (!outfile_topic9_.is_open()) 
                                {
                                    cout<<"Error opening file topic9_timestamp.txt! "<<endl;
                                    return false;
                                }

                                // set output accuracy
                                outfile_topic9_.setf(ios::fixed, ios::floatfield);
                                outfile_topic9_.precision(9);
                            }    
                        }    
                    }    
                }    
            }    
        }    
    }
    return true;
}

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

    switch(channel_num)
    {
    case 2:
    {
        open_file(2, result_path);

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

            auto sub_node = std::make_shared<SubscriberTopic2>(real_period1, real_period2, outfile_lat_, outfile_mdl_, outfile_topic1_, outfile_topic2_);
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
        open_file(3, result_path);

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

            auto sub_node = std::make_shared<SubscriberTopic3>(real_period1, real_period2, real_period3, outfile_lat_, outfile_mdl_, outfile_topic1_, outfile_topic2_, outfile_topic3_);
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
        open_file(4, result_path);

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

            auto sub_node = std::make_shared<SubscriberTopic4>(real_period1, real_period2, real_period3, real_period4, outfile_lat_, outfile_mdl_, outfile_topic1_, outfile_topic2_, outfile_topic3_, outfile_topic4_);
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
        open_file(5, result_path);

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

            auto sub_node = std::make_shared<SubscriberTopic5>(real_period1, real_period2, real_period3, real_period4, real_period5, outfile_lat_, outfile_mdl_, outfile_topic1_, outfile_topic2_, outfile_topic3_, outfile_topic4_, outfile_topic5_);
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
        open_file(6, result_path);

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

            auto sub_node = std::make_shared<SubscriberTopic6>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, outfile_lat_, outfile_mdl_, outfile_topic1_, outfile_topic2_, outfile_topic3_, outfile_topic4_, outfile_topic5_, outfile_topic6_);
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
        open_file(7, result_path);

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

            auto sub_node = std::make_shared<SubscriberTopic7>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, outfile_lat_, outfile_mdl_, outfile_topic1_, outfile_topic2_, outfile_topic3_, outfile_topic4_, outfile_topic5_, outfile_topic6_, outfile_topic7_);
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
        open_file(8, result_path);

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

            auto sub_node = std::make_shared<SubscriberTopic8>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, real_period8, outfile_lat_, outfile_mdl_, outfile_topic1_, outfile_topic2_, outfile_topic3_, outfile_topic4_, outfile_topic5_, outfile_topic6_, outfile_topic7_, outfile_topic8_);
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
        open_file(9, result_path);

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

            auto sub_node = std::make_shared<SubscriberTopic9>(real_period1, real_period2, real_period3, real_period4, real_period5, real_period6, real_period7, real_period8, real_period9, outfile_lat_, outfile_mdl_, outfile_topic1_, outfile_topic2_, outfile_topic3_, outfile_topic4_, outfile_topic5_, outfile_topic6_, outfile_topic7_, outfile_topic8_, outfile_topic9_);
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
