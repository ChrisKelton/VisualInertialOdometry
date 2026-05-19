#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
queue<sensor_msgs::msg::Image::ConstSharedPtr> img_buf;

rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

static rclcpp::Node::SharedPtr g_node;

void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
    double stamp_sec = rclcpp::Time(img_msg->header.stamp).seconds();

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = stamp_sec;
        last_image_time = stamp_sec;
        return;
    }
    // detect unstable camera stream
    if (stamp_sec - last_image_time > 1.0 || stamp_sec < last_image_time)
    {
        RCLCPP_WARN(g_node->get_logger(), "image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }
    last_image_time = stamp_sec;
    // frequency control
    if (round(1.0 * pub_count / (stamp_sec - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (stamp_sec - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = stamp_sec;
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;

    // Convert ROS image to cv::Mat without cv_bridge to avoid OpenCV version ABI mismatch.
    // Both mono8 and 8UC1 are single-channel uint8 — wrap the raw data directly.
    cv::Mat gray_img(img_msg->height, img_msg->width, CV_8UC1,
                     const_cast<uint8_t*>(img_msg->data.data()),
                     img_msg->step);
    // Clone so we own the data independent of the message lifetime.
    cv::Mat tracked_img = gray_img.clone();

    cv::Mat show_img = tracked_img;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(tracked_img.rowRange(ROW * i, ROW * (i + 1)), stamp_sec);
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(tracked_img.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = tracked_img.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        // skip the first image; since no optical speed on first image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img->publish(*feature_points);

        static bool first_img_published = false;
        if (SHOW_TRACK)
        {
            if (!first_img_published) {
                RCLCPP_INFO(g_node->get_logger(), "Publishing first feature_img frame");
                first_img_published = true;
            }
            cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img.rowRange(i * ROW, (i + 1) * ROW), tmp_img, cv::COLOR_GRAY2BGR);

                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                }
            }
            // Publish without cv_bridge: pack the bgr8 cv::Mat into a ROS image message.
            sensor_msgs::msg::Image match_msg;
            match_msg.header = img_msg->header;
            match_msg.height = stereo_img.rows;
            match_msg.width  = stereo_img.cols;
            match_msg.encoding = "bgr8";
            match_msg.is_bigendian = 0;
            match_msg.step = stereo_img.cols * 3;
            match_msg.data.assign(stereo_img.datastart, stereo_img.dataend);
            pub_match->publish(match_msg);
        }
    }
    RCLCPP_INFO(g_node->get_logger(), "whole feature tracker processing costs: %f", t_r.toc());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("feature_tracker");

    readParameters(g_node);

    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                RCLCPP_INFO(g_node->get_logger(), "load mask fail");
            }
            else
                RCLCPP_INFO(g_node->get_logger(), "load mask success");
        }
    }

    auto sub_img = g_node->create_subscription<sensor_msgs::msg::Image>(
        IMAGE_TOPIC, 100, img_callback);

    pub_img = g_node->create_publisher<sensor_msgs::msg::PointCloud>("feature", 1000);
    pub_match = g_node->create_publisher<sensor_msgs::msg::Image>("feature_img", 1000);
    pub_restart = g_node->create_publisher<std_msgs::msg::Bool>("restart", 1000);
    RCLCPP_INFO(g_node->get_logger(), "Publishers created. IMAGE_TOPIC=%s SHOW_TRACK=%d FREQ=%d ROW=%d COL=%d",
                IMAGE_TOPIC.c_str(), SHOW_TRACK, FREQ, ROW, COL);

    rclcpp::spin(g_node);
    rclcpp::shutdown();
    return 0;
}
