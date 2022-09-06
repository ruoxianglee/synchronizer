#include <string>
#include <fstream>

#include <message_filters/subscriber.h>  
#include <message_filters/synchronizer.h>  
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/approximate_time_model.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "signal.h"
#include <unistd.h>

using namespace std;
using namespace message_filters;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using ImuMsgT = sensor_msgs::msg::Imu;
using PointCloud2MsgT = sensor_msgs::msg::PointCloud2;
using CompressedImageMsgT = sensor_msgs::msg::CompressedImage;

#define buffer_size 10000
string result_path;

class SynchronizerTestCamImu
    : public rclcpp::Node
{
    public:
    SynchronizerTestCamImu() :
        Node("synchronizer_test_cam_imu"), alg_sync_(buffer_size), mdl_sync_()
    {
        image_sub_ = this->create_subscription<CompressedImageMsgT>("/simulator/camera_node/image/compressed", 10, 
                                                                        std::bind(&SynchronizerTestCamImu::image_callback, this, _1));
        imu_sub_   = this->create_subscription<ImuMsgT>("/imu_raw", 10, 
                                                            std::bind(&SynchronizerTestCamImu::imu_callback, this, _1));

        alg_sync_.registerCallback(std::bind(&SynchronizerTestCamImu::alg_callback, this, _1, _2));
        mdl_sync_.registerCallback(std::bind(&SynchronizerTestCamImu::mdl_callback, this, _1, _2));

        outfile_alg_.open(result_path + "/cam_imu/timestamp_alg.txt", ios::out);
        if (!outfile_alg_.is_open()) 
        {
            cout<<"Error opening file timestamp_alg.txt for cam + imu! "<<endl;
        }

        // set output accuracy
        outfile_alg_.setf(ios::fixed, ios::floatfield);
        outfile_alg_.precision(9);

        outfile_mdl_.open(result_path + "/cam_imu/timestamp_mdl.txt", ios::out);
        if (!outfile_mdl_.is_open()) 
        {
            cout<<"Error opening file timestamp_mdl.txt for cam + imu! "<<endl;
        }

        // set output accuracy
        outfile_mdl_.setf(ios::fixed, ios::floatfield);
        outfile_mdl_.precision(9);

        // Thread = new std::thread(&SynchronizerTestCamImu::timer_callback,this);
    }

    ~SynchronizerTestCamImu()
    {
        outfile_alg_.close();
        outfile_mdl_.close();
    }

    private:
    void alg_callback(const CompressedImageMsgT::ConstSharedPtr& image, const ImuMsgT::ConstSharedPtr& imu)
    {
        double image_timestamp = (double)image->header.stamp.sec + 1e-9*(double)image->header.stamp.nanosec;
        double imu_timestamp = (double)imu->header.stamp.sec + 1e-9*(double)imu->header.stamp.nanosec;
        outfile_alg_ << image_timestamp << " " << imu_timestamp << " " << endl;
    }

    void mdl_callback(const CompressedImageMsgT::ConstSharedPtr& image, const ImuMsgT::ConstSharedPtr& imu)
    {
        double image_timestamp = (double)image->header.stamp.sec + 1e-9*(double)image->header.stamp.nanosec;
        double imu_timestamp = (double)imu->header.stamp.sec + 1e-9*(double)imu->header.stamp.nanosec;
        outfile_mdl_ << image_timestamp << " " << imu_timestamp << " " << endl;
    }

    void image_callback(const CompressedImageMsgT::ConstSharedPtr image)
    {
        alg_sync_.add<0>(image);
        mdl_sync_.add<0>(image);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    void imu_callback(const ImuMsgT::ConstSharedPtr imu)
    {
        alg_sync_.add<1>(imu);
        mdl_sync_.add<1>(imu);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    // void timer_callback()
    // { 
    //     while(!time_out)
    //     {   
    //         std::lock_guard<std::mutex> lk(mtx);
    //         time_out = true;

    //         rclcpp::sleep_for(std::chrono::milliseconds(10000));
    //     }

    //     kill(getpid(),SIGINT);
    // }

    rclcpp::Subscription<CompressedImageMsgT>::SharedPtr image_sub_;
    rclcpp::Subscription<ImuMsgT>::SharedPtr imu_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<CompressedImageMsgT, ImuMsgT> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<CompressedImageMsgT, ImuMsgT> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    ofstream outfile_alg_;
    ofstream outfile_mdl_;

    // bool time_out;
    // std::thread* Thread;
    // std::mutex mtx;
};

class SynchronizerTestCamLidar
    : public rclcpp::Node
{
    public:
    SynchronizerTestCamLidar() :
        Node("synchronizer_test_cam_lidar"), alg_sync_(buffer_size), mdl_sync_()
    {
        image_sub_ = this->create_subscription<CompressedImageMsgT>("/simulator/camera_node/image/compressed", 10, 
                                                                        std::bind(&SynchronizerTestCamLidar::image_callback, this, _1));
        lidar_sub_ = this->create_subscription<PointCloud2MsgT>("/points_raw", 10, 
                                                            std::bind(&SynchronizerTestCamLidar::lidar_callback, this, _1));

        alg_sync_.registerCallback(std::bind(&SynchronizerTestCamLidar::alg_callback, this, _1, _2));
        mdl_sync_.registerCallback(std::bind(&SynchronizerTestCamLidar::mdl_callback, this, _1, _2));

        outfile_alg_.open(result_path + "/cam_lidar/timestamp_alg.txt", ios::out);
        if (!outfile_alg_.is_open()) 
        {
            cout<<"Error opening file timestamp_alg.txt for cam + lidar! "<<endl;
        }

        // set output accuracy
        outfile_alg_.setf(ios::fixed, ios::floatfield);
        outfile_alg_.precision(9);

        outfile_mdl_.open(result_path + "/cam_lidar/timestamp_mdl.txt", ios::out);
        if (!outfile_mdl_.is_open()) 
        {
            cout<<"Error opening file timestamp_mdl.txt for cam + lidar! "<<endl;
        }

        // set output accuracy
        outfile_mdl_.setf(ios::fixed, ios::floatfield);
        outfile_mdl_.precision(9);

        // Thread = new std::thread(&SynchronizerTestCamLidar::timer_callback,this);
    }

    ~SynchronizerTestCamLidar()
    {
        outfile_alg_.close();
        outfile_mdl_.close();
    }

    private:
    void alg_callback(const CompressedImageMsgT::ConstSharedPtr& image, const PointCloud2MsgT::ConstSharedPtr& points)
    {
        double image_timestamp = (double)image->header.stamp.sec + 1e-9*(double)image->header.stamp.nanosec;
        double lidar_timestamp = (double)points->header.stamp.sec + 1e-9*(double)points->header.stamp.nanosec;
        outfile_alg_ << image_timestamp << " " << lidar_timestamp << " " << endl;
    }

    void mdl_callback(const CompressedImageMsgT::ConstSharedPtr& image, const PointCloud2MsgT::ConstSharedPtr& points)
    {
        double image_timestamp = (double)image->header.stamp.sec + 1e-9*(double)image->header.stamp.nanosec;
        double lidar_timestamp = (double)points->header.stamp.sec + 1e-9*(double)points->header.stamp.nanosec;
        outfile_mdl_ << image_timestamp << " " << lidar_timestamp << " " << endl;
    }

    void image_callback(const CompressedImageMsgT::ConstSharedPtr image)
    {
        alg_sync_.add<0>(image);
        mdl_sync_.add<0>(image);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    void lidar_callback(const PointCloud2MsgT::ConstSharedPtr points)
    {
        alg_sync_.add<1>(points);
        mdl_sync_.add<1>(points);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    // void timer_callback()
    // { 
    //     while(!time_out)
    //     {   
    //         std::lock_guard<std::mutex> lk(mtx);
    //         time_out = true;

    //         rclcpp::sleep_for(std::chrono::milliseconds(10000));
    //     }

    //     kill(getpid(),SIGINT);
    // }

    rclcpp::Subscription<CompressedImageMsgT>::SharedPtr image_sub_;
    rclcpp::Subscription<PointCloud2MsgT>::SharedPtr lidar_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<CompressedImageMsgT, PointCloud2MsgT> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<CompressedImageMsgT, PointCloud2MsgT> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    ofstream outfile_alg_;
    ofstream outfile_mdl_;

    // bool time_out;
    // std::thread* Thread;
    // std::mutex mtx;
};

class SynchronizerTestImuLidar
    : public rclcpp::Node
{
    public:
    SynchronizerTestImuLidar() :
        Node("synchronizer_test_imu_lidar"), alg_sync_(buffer_size), mdl_sync_()
    {
        imu_sub_   = this->create_subscription<ImuMsgT>("/imu_raw", 10, 
                                                            std::bind(&SynchronizerTestImuLidar::imu_callback, this, _1));
        lidar_sub_ = this->create_subscription<PointCloud2MsgT>("/points_raw", 10, 
                                                            std::bind(&SynchronizerTestImuLidar::lidar_callback, this, _1));

        alg_sync_.registerCallback(std::bind(&SynchronizerTestImuLidar::alg_callback, this, _1, _2));
        mdl_sync_.registerCallback(std::bind(&SynchronizerTestImuLidar::mdl_callback, this, _1, _2));

        outfile_alg_.open(result_path + "/imu_lidar/timestamp_alg.txt", ios::out);
        if (!outfile_alg_.is_open()) 
        {
            cout<<"Error opening file timestamp_alg.txt for imu + lidar! "<<endl;
        }

        // set output accuracy
        outfile_alg_.setf(ios::fixed, ios::floatfield);
        outfile_alg_.precision(9);

        outfile_mdl_.open(result_path + "/imu_lidar/timestamp_mdl.txt", ios::out);
        if (!outfile_mdl_.is_open()) 
        {
            cout<<"Error opening file timestamp_mdl.txt for imu + lidar! "<<endl;
        }

        // set output accuracy
        outfile_mdl_.setf(ios::fixed, ios::floatfield);
        outfile_mdl_.precision(9);

        // Thread = new std::thread(&SynchronizerTestImuLidar::timer_callback,this);
    }

    ~SynchronizerTestImuLidar()
    {
        outfile_alg_.close();
        outfile_mdl_.close();
    }

    private:
    void alg_callback(const ImuMsgT::ConstSharedPtr& imu, const PointCloud2MsgT::ConstSharedPtr& points)
    {
        double imu_timestamp = (double)imu->header.stamp.sec + 1e-9*(double)imu->header.stamp.nanosec;
        double lidar_timestamp = (double)points->header.stamp.sec + 1e-9*(double)points->header.stamp.nanosec;
        outfile_alg_ << imu_timestamp << " " << lidar_timestamp << " " << endl;
    }

    void mdl_callback(const ImuMsgT::ConstSharedPtr& imu, const PointCloud2MsgT::ConstSharedPtr& points)
    {
        double imu_timestamp = (double)imu->header.stamp.sec + 1e-9*(double)imu->header.stamp.nanosec;
        double lidar_timestamp = (double)points->header.stamp.sec + 1e-9*(double)points->header.stamp.nanosec;
        outfile_mdl_ << imu_timestamp << " " << lidar_timestamp << " " << endl;
    }

    void imu_callback(const ImuMsgT::ConstSharedPtr imu)
    {
        alg_sync_.add<0>(imu);
        mdl_sync_.add<0>(imu);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    void lidar_callback(const PointCloud2MsgT::ConstSharedPtr points)
    {
        alg_sync_.add<1>(points);
        mdl_sync_.add<1>(points);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    // void timer_callback()
    // { 
    //     while(!time_out)
    //     {
    //         std::lock_guard<std::mutex> lk(mtx);
    //         time_out = true;

    //         rclcpp::sleep_for(std::chrono::milliseconds(10000));
    //     }

    //     kill(getpid(),SIGINT);
    // }

    rclcpp::Subscription<ImuMsgT>::SharedPtr imu_sub_;
    rclcpp::Subscription<PointCloud2MsgT>::SharedPtr lidar_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<ImuMsgT, PointCloud2MsgT> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<ImuMsgT, PointCloud2MsgT> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    ofstream outfile_alg_;
    ofstream outfile_mdl_;

    // bool time_out;
    // std::thread* Thread;
    // std::mutex mtx;
};

class SynchronizerTestCamImuLidar
    : public rclcpp::Node
{
    public:
    SynchronizerTestCamImuLidar() :
        Node("synchronizer_test_cam_imu_lidar"), alg_sync_(buffer_size), mdl_sync_()
    {
        image_sub_ = this->create_subscription<CompressedImageMsgT>("/simulator/camera_node/image/compressed", 10, 
                                                                        std::bind(&SynchronizerTestCamImuLidar::image_callback, this, _1));

        imu_sub_   = this->create_subscription<ImuMsgT>("/imu_raw", 10, 
                                                            std::bind(&SynchronizerTestCamImuLidar::imu_callback, this, _1));
        lidar_sub_ = this->create_subscription<PointCloud2MsgT>("/points_raw", 10, 
                                                            std::bind(&SynchronizerTestCamImuLidar::lidar_callback, this, _1));

        alg_sync_.registerCallback(std::bind(&SynchronizerTestCamImuLidar::alg_callback, this, _1, _2, _3));
        mdl_sync_.registerCallback(std::bind(&SynchronizerTestCamImuLidar::mdl_callback, this, _1, _2, _3));

        outfile_alg_.open(result_path + "/cam_imu_lidar/timestamp_alg.txt", ios::out);
        if (!outfile_alg_.is_open()) 
        {
            cout<<"Error opening file timestamp_alg.txt for cam + imu + lidar! "<<endl;
        }

        // set output accuracy
        outfile_alg_.setf(ios::fixed, ios::floatfield);
        outfile_alg_.precision(9);

        outfile_mdl_.open(result_path + "/cam_imu_lidar/timestamp_mdl.txt", ios::out);
        if (!outfile_mdl_.is_open()) 
        {
            cout<<"Error opening file timestamp_mdl.txt for cam + imu + lidar! "<<endl;
        }

        // set output accuracy
        outfile_mdl_.setf(ios::fixed, ios::floatfield);
        outfile_mdl_.precision(9);

        // Thread = new std::thread(&SynchronizerTestCamImuLidar::timer_callback,this);

        // outfile_image_.open(result_path + "image_timestamp.txt", ios::out);
        // if (!outfile_image_.is_open()) 
        // {
        //     cout<<"Error opening file image_timestamp.txt! "<<endl;
        // }

        // // set output accuracy
        // outfile_image_.setf(ios::fixed, ios::floatfield);
        // outfile_image_.precision(9);

        // outfile_imu_.open(result_path + "imu_timestamp.txt", ios::out);
        // if (!outfile_imu_.is_open()) 
        // {
        //     cout<<"Error opening file imu_timestamp.txt! "<<endl;
        // }

        // // set output accuracy
        // outfile_imu_.setf(ios::fixed, ios::floatfield);
        // outfile_imu_.precision(9);

        // outfile_lidar_.open(result_path + "lidar_timestamp.txt", ios::out);
        // if (!outfile_lidar_.is_open()) 
        // {
        //     cout<<"Error opening file lidar_timestamp.txt! "<<endl;
        // }

        // // set output accuracy
        // outfile_lidar_.setf(ios::fixed, ios::floatfield);
        // outfile_lidar_.precision(9);
    }

    ~SynchronizerTestCamImuLidar()
    {
        outfile_alg_.close();
        outfile_mdl_.close();
        // outfile_image_.close();
        // outfile_imu_.close();
        // outfile_lidar_.close();
    }

    private:
    void alg_callback(const CompressedImageMsgT::ConstSharedPtr& image, const ImuMsgT::ConstSharedPtr& imu, const PointCloud2MsgT::ConstSharedPtr& points)
    {
        double image_timestamp = (double)image->header.stamp.sec + 1e-9*(double)image->header.stamp.nanosec;
        double imu_timestamp = (double)imu->header.stamp.sec + 1e-9*(double)imu->header.stamp.nanosec;
        double lidar_timestamp = (double)points->header.stamp.sec + 1e-9*(double)points->header.stamp.nanosec;
        outfile_alg_ << image_timestamp << " " << imu_timestamp << " " << lidar_timestamp << " " << endl;
    }

    void mdl_callback(const CompressedImageMsgT::ConstSharedPtr& image, const ImuMsgT::ConstSharedPtr& imu, const PointCloud2MsgT::ConstSharedPtr& points)
    {
        double image_timestamp = (double)image->header.stamp.sec + 1e-9*(double)image->header.stamp.nanosec;
        double imu_timestamp = (double)imu->header.stamp.sec + 1e-9*(double)imu->header.stamp.nanosec;
        double lidar_timestamp = (double)points->header.stamp.sec + 1e-9*(double)points->header.stamp.nanosec;
        outfile_mdl_ << image_timestamp << " " << imu_timestamp << " " << lidar_timestamp << " " << endl;
    }

    void image_callback(const CompressedImageMsgT::ConstSharedPtr image)
    {
        // double image_timestamp = (double)image->header.stamp.sec + 1e-9*(double)image->header.stamp.nanosec;

        // outfile_image_ << image_timestamp << endl;
        
        alg_sync_.add<0>(image);
        mdl_sync_.add<0>(image);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    void imu_callback(const ImuMsgT::ConstSharedPtr imu)
    {
        // double imu_timestamp = (double)imu->header.stamp.sec + 1e-9*(double)imu->header.stamp.nanosec;
        // outfile_imu_ << imu_timestamp << endl;
        alg_sync_.add<1>(imu);
        mdl_sync_.add<1>(imu);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    void lidar_callback(const PointCloud2MsgT::ConstSharedPtr points)
    {
        // double lidar_timestamp = (double)points->header.stamp.sec + 1e-9*(double)points->header.stamp.nanosec;
        // outfile_lidar_ << lidar_timestamp << endl;
        alg_sync_.add<2>(points);
        mdl_sync_.add<2>(points);

        // std::lock_guard<std::mutex> lk(mtx);
        // time_out = false;
    }

    // void timer_callback()
    // { 
    //     while(!time_out)
    //     {
    //         std::lock_guard<std::mutex> lk(mtx);
    //         time_out = true;

    //         rclcpp::sleep_for(std::chrono::milliseconds(10000));
    //     }

    //     kill(getpid(),SIGINT);
    // }

    rclcpp::Subscription<CompressedImageMsgT>::SharedPtr image_sub_;
    rclcpp::Subscription<ImuMsgT>::SharedPtr imu_sub_;
    rclcpp::Subscription<PointCloud2MsgT>::SharedPtr lidar_sub_;

    typedef Synchronizer<sync_policies::ApproximateTime<CompressedImageMsgT, ImuMsgT, PointCloud2MsgT> > AlgSync;
    typedef Synchronizer<sync_policies::ApproximateTimeModel<CompressedImageMsgT, ImuMsgT, PointCloud2MsgT> > MdlSync;

    AlgSync alg_sync_;
    MdlSync mdl_sync_;

    ofstream outfile_alg_;
    ofstream outfile_mdl_;
    // ofstream outfile_image_;
    // ofstream outfile_imu_;
    // ofstream outfile_lidar_;

    // bool time_out;
    // std::thread* Thread;
    // std::mutex mtx;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    result_path = argv[1];
    
    rclcpp::executors::SingleThreadedExecutor executor;
    auto cam_imu_node       = std::make_shared<SynchronizerTestCamImu>();
    auto cam_lidar_node     = std::make_shared<SynchronizerTestCamLidar>();
    auto imu_lidar_node     = std::make_shared<SynchronizerTestImuLidar>();
    auto cam_imu_lidar_node = std::make_shared<SynchronizerTestCamImuLidar>();

    executor.add_node(cam_imu_node);
    executor.add_node(cam_lidar_node);
    executor.add_node(imu_lidar_node);
    executor.add_node(cam_imu_lidar_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
