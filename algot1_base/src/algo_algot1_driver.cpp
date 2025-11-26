#include "algo_algot1_driver.h"

#include "uliti/AlgoOpenCv.h"
#include "uliti/AlgoTurbJpeg.h"
#include <algot1_msgs/GnssState.h>
#include <algot1_msgs/Gins.h>
bool use_ros_time = true;
std::string IMU_fram= "IMU_link";  //最初为global

namespace Algo1010
{
        AlgoT1RosDriver::AlgoT1RosDriver()
        {
#ifdef ROS1
                node_ = new ros::NodeHandle("algot1");
                ros::NodeHandle private_nh("~");
                private_nh.param<std::string>("topic_cam0", topic_cam0_, std::string("/cam0"));
                private_nh.param<std::string>("topic_cam1", topic_cam1_, std::string("/cam1"));
                private_nh.param<std::string>("topic_cam2", topic_cam2_, std::string("/cam2"));
                private_nh.param<std::string>("topic_imu", topic_imu_, std::string("/imu0"));
                private_nh.param<double>("ref_B", ref_B_, 0.0);
                private_nh.param<double>("ref_L", ref_L_, 0.0);
                private_nh.param<double>("ref_H", ref_H_, 0.0);
                cam0_publisher_ = node_->advertise<sensor_msgs::Image>(topic_cam0_, 10, true);
                cam1_publisher_ = node_->advertise<sensor_msgs::Image>(topic_cam1_, 10, true);
                cam2_publisher_ = node_->advertise<sensor_msgs::Image>(topic_cam2_, 10, true);
                imu_publisher_ = node_->advertise<sensor_msgs::Imu>(topic_imu_, 100, true);
                pose_inspvaxa_publisher_ = node_->advertise<sensor_msgs::NavSatFix>(topic_inspvaxa_, 100, true);
                pose_gpgga_publisher_ = node_->advertise<sensor_msgs::NavSatFix>(topic_gpgga_, 10, true);
                pose_gprmc_publisher_ = node_->advertise<sensor_msgs::NavSatFix>(topic_gprmc_, 10, true);
                pose_vins_publisher_ = node_->advertise<sensor_msgs::NavSatFix>(topic_vins_, 10, true);
                path_inspvaxa_publisher_ = node_->advertise<nav_msgs::Path>(topic_path_inspvaxa_, 10, true);
                path_gpgga_publisher_ = node_->advertise<nav_msgs::Path>(topic_path_gpgga_, 10, true);
                path_gprmc_publisher_ = node_->advertise<nav_msgs::Path>(topic_path_gprmc_, 10, true);
                path_vins_publisher_ = node_->advertise<nav_msgs::Path>(topic_path_vins_, 10, true);
                odom_gpgga_publisher_ = node_->advertise<nav_msgs::Odometry>("/odom_gpgga", 10, true);
                odom_inspvaxa_publisher_ = node_->advertise<nav_msgs::Odometry>("/odom_inspvaxa", 10, true);
                diy_gins_publisher_ = node_->advertise<algot1_msgs::Gins>(topic_diy_gins_, 100, true);
#endif

#ifdef ROS2
                node_ = new rclcpp::Node("algot1");

                // node_->declare_parameter("odom_frame", rclcpp::ParameterValue(""));
                // node_->get_parameter_or<std::string>("odom_frame", odom_frame, "");
                // node_->declare_parameter("base_frame", rclcpp::ParameterValue(""));
                // node_->get_parameter_or<std::string>("base_frame", base_frame, "");

                node_->declare_parameter("topic_cam0", rclcpp::ParameterValue(""));
                node_->get_parameter_or<std::string>("topic_cam0", topic_cam0_, "");
                node_->declare_parameter("topic_cam1", rclcpp::ParameterValue(""));
                node_->get_parameter_or<std::string>("topic_cam1", topic_cam1_, "");
                node_->declare_parameter("topic_cam2", rclcpp::ParameterValue(""));
                node_->get_parameter_or<std::string>("topic_cam2", topic_cam2_, "");

                node_->declare_parameter("topic_imu", rclcpp::ParameterValue(""));
                node_->get_parameter_or<std::string>("topic_imu", topic_imu_, "");

                node_->declare_parameter("ref_B", rclcpp::ParameterValue(0.0));
                node_->get_parameter_or<double>("ref_B", ref_B_, 0.0);
                node_->declare_parameter("ref_L", rclcpp::ParameterValue(0.0));
                node_->get_parameter_or<double>("ref_L", ref_L_, 0.0);
                node_->declare_parameter("ref_H", rclcpp::ParameterValue(0.0));
                node_->get_parameter_or<double>("ref_H", ref_H_, 0.0);

                cam0_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_cam0_, 10);
                cam1_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_cam1_, 10);
                cam2_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_cam2_, 10);
                imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_imu_, 100);
                pose_inspvaxa_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_inspvaxa_, 100);
                pose_gpgga_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_gpgga_, 10);
                pose_gprmc_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_gprmc_, 10);
                pose_vins_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(topic_vins_, 10);

                path_inspvaxa_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(topic_path_inspvaxa_, 10);
                path_gpgga_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(topic_path_gpgga_, 10);
                path_gprmc_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(topic_path_gprmc_, 10);
                path_vins_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(topic_path_vins_, 10);

                odom_gpgga_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom_gpgga", 10);
                odom_inspvaxa_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom_inspvaxa", 10);

                diy_gnss_state_publisher_ = node_->create_publisher<algot1_msgs::msg::GnssState>(topic_diy_gnssstate_, 10);
#endif

                m_coordTrans.setRefBLH(ref_B_ * DEG2RAD, ref_L_ * DEG2RAD, ref_H_);

                std::cout << "\n=======================================" << std::endl;
                std::cout << "topic_cam0 = " << topic_cam0_ << std::endl;
                std::cout << "topic_cam1 = " << topic_cam1_ << std::endl;
                std::cout << "topic_cam2 = " << topic_cam2_ << std::endl;
                std::cout << "topic_imu = " << topic_imu_ << std::endl;
                std::cout << std::setprecision(9);
                std::cout << "ref_B = " << ref_B_ << std::endl;
                std::cout << "ref_L = " << ref_L_ << std::endl;
                std::cout << "ref_H = " << ref_H_ << std::endl;
                std::cout << "=======================================\n"
                          << std::endl;
        }

        AlgoT1RosDriver::~AlgoT1RosDriver()
        {
        }

        double AlgoT1RosDriver::gps_to_unix(int gps_week, double week_seconds)
        {
                // GPS起始时间对应的Unix时间戳（1980年1月6日00:00:00 UTC）
                const double gps_epoch = 315964800.0;
                // return gps_epoch + static_cast<double>(gps_week) * 604800.0 + week_seconds;
                return gps_epoch + (gps_week) * 604800.0 + week_seconds - 18.0;
        }
        
        void AlgoT1RosDriver::publishCam0Image(AlgoCameraDataPacket *packet, char *data, int len)
        {
                printf("publish Cam0:index=%d,  hasPPS=%d, pps=%.3fms, w*h=(%d*%d), imageType=%d,len=%d\n", packet->imageIndex, packet->hasPPS, packet->cur_sec,
                       packet->imageWidth, packet->imageHeight, (int)packet->imageType, packet->pbufSize);

#ifdef USE_OPENCV

                // Get our image of Cam0
                cv::Mat frame = cv::Mat::zeros(packet->imageHeight, packet->imageWidth, CV_8UC3);
                if (packet->imageType == 0)
                { // raw
                        memcpy(frame.data, (unsigned char *)packet->pbuf, packet->pbufSize);
                }
                else
                { // jpeg
                        AlgoOpenCv::toJpegImage(packet, frame);
                }

                if (frame.empty())
                {
                        std::cout << "publishCam0Image failed!" << std::endl;
                        return;
                }

// create and publish msg, ros1
#ifdef ROS1
                std_msgs::Header header;
                double unix_time = gps_to_unix(packet->cur_week, packet->cur_sec);
                if(use_ros_time)
                	header.stamp = ros::Time::now();
                else
                	header.stamp = RosTime(unix_time);
                header.frame_id = "base_link";
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                cam0_publisher_.publish(img_msg);
#endif

// create and publish msg, ros2
#ifdef ROS2
                std_msgs::msg::Header header;
                if(use_ros_time)
                	header.stamp = ros::Time::now();
                else
                	header.stamp = RosTime(packet->cur_sec);
                header.frame_id = "base_link";
                sensor_msgs::msg::Image img_msg;
                cv_bridge::CvImage _cv_bridge = cv_bridge::CvImage(header, "bgr8", frame);
                _cv_bridge.toImageMsg(img_msg);
                cam0_publisher_->publish(img_msg);
#endif

#endif //! USE_OPENCV
        }

        void AlgoT1RosDriver::publishCam1Image(AlgoCameraDataPacket *packet, char *data, int len)
        {
                printf("publish Cam1:index=%d,  hasPPS=%d, pps=%.3fms, w*h=(%d*%d), imageType=%d,len=%d\n", packet->imageIndex, packet->hasPPS, packet->cur_sec,
                       packet->imageWidth, packet->imageHeight, (int)packet->imageType, packet->pbufSize);

#ifdef USE_OPENCV
                // Get our image of Cam0
                cv::Mat frame = cv::Mat::zeros(packet->imageHeight, packet->imageWidth, CV_8UC3);
                if (packet->imageType == 0)
                { // raw
                        memcpy(frame.data, (unsigned char *)packet->pbuf, packet->pbufSize);
                }
                else
                { // jpeg
                        AlgoOpenCv::toJpegImage(packet, frame);
                }

                if (frame.empty())
                {
                        std::cout << "publishCam1Image failed!" << std::endl;
                        return;
                }

                // create and publish msg, ros1
#ifdef ROS1
                std_msgs::Header header;
                double unix_time = gps_to_unix(packet->cur_week, packet->cur_sec);
                // nav_msg->header.stamp = RosTime(unix_time);
                if(use_ros_time)
                	header.stamp = ros::Time::now();
                else
                	header.stamp = RosTime(unix_time);
                header.frame_id = "base_link";
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                cam1_publisher_.publish(img_msg);
#endif

                // create and publish msg, ros2
#ifdef ROS2
                std_msgs::msg::Header header;
                if(use_ros_time)
                	header.stamp = ros::Time::now();
                else
                	header.stamp = RosTime(packet->cur_sec);
                header.frame_id = "base_link";
                sensor_msgs::msg::Image img_msg;
                cv_bridge::CvImage _cv_bridge = cv_bridge::CvImage(header, "bgr8", frame);
                _cv_bridge.toImageMsg(img_msg);
                cam1_publisher_->publish(img_msg);
#endif

#endif
        }

        void AlgoT1RosDriver::publishCam2Image(AlgoCameraDataPacket *packet, char *data, int len)
        {
                printf("publish Cam2:index=%d,  hasPPS=%d, pps=%.3fms, w*h=(%d*%d), imageType=%d,len=%d\n", packet->imageIndex, packet->hasPPS, packet->cur_sec,
                       packet->imageWidth, packet->imageHeight, (int)packet->imageType, packet->pbufSize);

#ifdef USE_OPENCV
                // Get our image of Cam0
                cv::Mat frame = cv::Mat::zeros(packet->imageHeight, packet->imageWidth, CV_8UC3);
                if (packet->imageType == 0)
                { // raw
                        memcpy(frame.data, (unsigned char *)packet->pbuf, packet->pbufSize);
                }
                else
                { // jpeg
                        AlgoOpenCv::toJpegImage(packet, frame);
                }

                if (frame.empty())
                {
                        std::cout << "publishCam1Image failed!" << std::endl;
                        return;
                }

                // create and publish msg, ros1
#ifdef ROS1
                std_msgs::Header header;
                double unix_time = gps_to_unix(packet->cur_week, packet->cur_sec);
                // nav_msg->header.stamp = RosTime(unix_time);
                if(use_ros_time)
                	header.stamp = ros::Time::now();
                else
                	header.stamp = RosTime(unix_time);
                header.frame_id = "base_link";
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
                cam2_publisher_.publish(img_msg);
#endif

                // create and publish msg, ros2
#ifdef ROS2
                std_msgs::msg::Header header;
                if(use_ros_time)
                	header.stamp = ros::Time::now();
                else
                	header.stamp = RosTime(packet->cur_sec);
                header.frame_id = "base_link";
                sensor_msgs::msg::Image img_msg;
                cv_bridge::CvImage _cv_bridge = cv_bridge::CvImage(header, "bgr8", frame);
                _cv_bridge.toImageMsg(img_msg);
                cam2_publisher_->publish(img_msg);
#endif

#endif
        }

        void AlgoT1RosDriver::publishInspvaxaData(AlgoINSPVAXADataPacket *packet, char *data, int len)
        {
                mutex_inspvaxa_.lock();
                memcpy(&packet_inspvaxa_, packet, sizeof(AlgoINSPVAXADataPacket));
                mutex_inspvaxa_.unlock();

                Algo1010::ENU res = m_coordTrans.BLH2ENU(packet->posLat, packet->posLon, packet->posHgt);

                printf("publish INSPVAXA:index=%d,  hasPPS=%d, pps=%.3fms, insStatus=%d, posType=%d, \n                 BLH=(%0.9f, %0.9f, %0.3f) ,ENU=(%0.3f, %0.3f, %0.3f), rpy=(%0.3f, %0.3f, %0.3f)\n",
                       packet->index, packet->hasPPS, packet->cur_sec, packet->insStatus, packet->posType,
                       packet->posLat * RAD2DEG, packet->posLon * RAD2DEG, packet->posHgt,
                       res.E, res.N, res.U, packet->roll, packet->pitch, packet->yaw);

                // if (packet->insStatus <= 1) return;

#ifdef ROS1

                sensor_msgs::NavSatFixPtr nav_msg(new sensor_msgs::NavSatFix());
                nav_msg->header.frame_id = IMU_fram;
                if(use_ros_time)
                	nav_msg->header.stamp = ros::Time::now();
                else
                	nav_msg->header.stamp = RosTime(packet->cur_sec);
                nav_msg->latitude = packet->posLat;                                // rad
                nav_msg->longitude = packet->posLon;                               // rad
                nav_msg->altitude = packet->posHgt;                                // rad
                nav_msg->position_covariance[4] = packet->sigLat * packet->sigLat; // m*m
                nav_msg->position_covariance[0] = packet->sigLon * packet->sigLon; // m*m
                nav_msg->position_covariance[8] = packet->sigHgt * packet->sigHgt; // m*m

                sensor_msgs::NavSatStatus nav_status_msg;
                nav_status_msg.status = packet->posType; // RTK status
                nav_status_msg.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
                nav_msg->status = nav_status_msg;
                pose_inspvaxa_publisher_.publish(nav_msg);

                static double last_stamp = 0;
                if (packet->cur_sec - last_stamp > 1.0)
                {
                        // publish path_inspvaxa_
                        path_inspvaxa_.header.frame_id = IMU_fram;
                        if(use_ros_time)
                		path_inspvaxa_.header.stamp = ros::Time::now();
              		 else
                        	path_inspvaxa_.header.stamp = RosTime(packet->cur_sec);
                        geometry_msgs::PoseStamped pose;
                        pose.header = path_inspvaxa_.header;
                        pose.pose.position.x = res.E;
                        pose.pose.position.y = res.N;
                        pose.pose.position.z = res.U;
                        path_inspvaxa_.poses.push_back(pose);
                        path_inspvaxa_publisher_.publish(path_inspvaxa_);
                        last_stamp = packet->cur_sec;
                }

                // publish odom
                geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw((packet->yaw) * DEG2RAD);
                nav_msgs::Odometry odo_msg;
                if(use_ros_time)
                	odo_msg.header.stamp = ros::Time::now();
              	else
                	odo_msg.header.stamp = RosTime(packet->cur_sec);
                odo_msg.header.frame_id = IMU_fram;
                odo_msg.pose.pose.position.x = res.E;
                odo_msg.pose.pose.position.y = res.N;
                odo_msg.pose.pose.position.z = res.U;
                odo_msg.pose.pose.orientation = odom_quat;
                odom_inspvaxa_publisher_.publish(odo_msg);

                // add by xuxin for fastlio2
                {
                        //==gins msg
                        algot1_msgs::Gins gins;
                        gins.header.frame_id = IMU_fram;
                        double unix_time = gps_to_unix(packet->cur_week, packet->cur_sec);
                        if(use_ros_time)
                		gins.header.stamp = ros::Time::now();
               	 else
                        	gins.header.stamp = RosTime(unix_time);

                        // state
                        gins.ins_state = packet->insStatus;
                        gins.pos_state = packet->posType;

                        // llh
                        gins.latitude = packet->posLat;  // rad
                        gins.longitude = packet->posLon; // rad
                        gins.altitude = packet->posHgt;  // rad

                        // ENU
                        gins.x = res.E;
                        gins.y = res.N;
                        gins.z = res.U;

                        // rpy
                        gins.roll = packet->roll;
                        gins.pitch = packet->pitch;
                        gins.yaw = packet->yaw;

                        // sigema
                        gins.covariance[0] = packet->sigLat;
                        gins.covariance[1] = packet->sigLon;
                        gins.covariance[2] = packet->sigHgt;
                        gins.covariance[3] = packet->sigVelN;
                        gins.covariance[4] = packet->sigVelE;
                        gins.covariance[5] = packet->sigVelU;
                        gins.covariance[6] = packet->sigYaw;
                        gins.covariance[7] = packet->sigRoll;
                        gins.covariance[8] = packet->sigPitch;

                        mutex_gpgga_.lock();
                        gins.rtk_state = packet_gpgga_.posType;
                        mutex_gpgga_.unlock();

                        diy_gins_publisher_.publish(gins);
                }
#endif

#ifdef ROS2
                // publish pose and path
                sensor_msgs::msg::NavSatFix nav_msg;
                nav_msg.header.frame_id = IMU_fram;
                if(use_ros_time)
                	nav_msg.header.stamp = ros::Time::now();
                else
                	nav_msg.header.stamp = RosTime(packet->cur_sec);
                nav_msg.latitude = packet->posLat;                                // rad
                nav_msg.longitude = packet->posLon;                               // rad
                nav_msg.altitude = packet->posHgt;                                // rad
                nav_msg.position_covariance[4] = packet->sigLat * packet->sigLat; // m*m
                nav_msg.position_covariance[0] = packet->sigLon * packet->sigLon; // m*m
                nav_msg.position_covariance[8] = packet->sigHgt * packet->sigHgt; // m*m
                pose_inspvaxa_publisher_->publish(nav_msg);

                static double last_stamp = 0;
                if (packet->cur_sec - last_stamp > 1.0)
                {
                        path_inspvaxa_.header.frame_id = IMU_fram;
                        if(use_ros_time)
                		path_inspvaxa_.header.stamp = ros::Time::now();
              	        else
                        	path_inspvaxa_.header.stamp = RosTime(packet->cur_sec);
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = path_inspvaxa_.header;
                        pose.pose.position.x = res.E;
                        pose.pose.position.y = res.N;
                        pose.pose.position.z = res.U;
                        path_inspvaxa_.poses.push_back(pose);
                        path_inspvaxa_publisher_->publish(path_inspvaxa_);
                        last_stamp = packet->cur_sec;
                }

                // publish odom
                geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw((360 - packet->yaw) * DEG2RAD);
                nav_msgs::msg::Odometry odo_msg;
                if(use_ros_time)
                	odo_msg.header.stamp = ros::Time::now();
                else
                	odo_msg.header.stamp = RosTime(packet->cur_sec);
                odo_msg.header.frame_id = IMU_fram;
                odo_msg.pose.pose.position.x = res.E;
                odo_msg.pose.pose.position.y = res.N;
                odo_msg.pose.pose.position.z = res.U;
                odo_msg.pose.pose.orientation = odom_quat;
                odom_inspvaxa_publisher_->publish(odo_msg);
#endif
        }

        void AlgoT1RosDriver::publishGpggaData(AlgoGPGGADataPacket *packet, char *data, int len)
        {
                mutex_gpgga_.lock();
                memcpy(&packet_gpgga_, packet, sizeof(AlgoGPGGADataPacket));
                mutex_gpgga_.unlock();

                Algo1010::ENU res = m_coordTrans.BLH2ENU(packet->posLat, packet->posLon, packet->posHgt);

                printf("publish GPGGA:index=%d, hasPPS=%d, pps=%.3fms, posType=%d, BLH=(%0.9f, %0.9f, %0.3f) ,ENU=(%0.3f, %0.3f, %0.3f)\n",
                       packet->index, packet->hasPPS, packet->cur_sec, packet->posType,
                       packet->posLat * RAD2DEG, packet->posLon * RAD2DEG, packet->posHgt,
                       res.E, res.N, res.U);

                if (packet->posType == 0)
                        return;

#ifdef ROS1
                // publish pose
                sensor_msgs::NavSatFixPtr nav_msg(new sensor_msgs::NavSatFix());
                double unix_time = gps_to_unix(packet->cur_week, packet->cur_sec);
                if(use_ros_time)
                	nav_msg->header.stamp = ros::Time::now();
                else
                	nav_msg->header.stamp = RosTime(unix_time);
                nav_msg->latitude = packet->posLat;
                nav_msg->longitude = packet->posLon;
                nav_msg->altitude = packet->posHgt;
                pose_gpgga_publisher_.publish(nav_msg);
                // publish path_gpgga_
                path_gpgga_.header.frame_id = IMU_fram;
                if(use_ros_time)
                	path_gpgga_.header.stamp = ros::Time::now();
                else
                	path_gpgga_.header.stamp = RosTime(packet->cur_sec);
                geometry_msgs::PoseStamped pose;
                pose.header = path_gpgga_.header;
                pose.pose.position.x = res.E;
                pose.pose.position.y = res.N;
                pose.pose.position.z = res.U;
                path_gpgga_.poses.push_back(pose);
                path_gpgga_publisher_.publish(path_gpgga_);
                // publish odom
                nav_msgs::Odometry odo_msg;
                if(use_ros_time)
                	odo_msg.header.stamp = ros::Time::now();
                else
                	odo_msg.header.stamp = RosTime(packet->cur_sec);
                odo_msg.header.frame_id = IMU_fram;
                odo_msg.pose.pose.position.x = res.E;
                odo_msg.pose.pose.position.y = res.N;
                odo_msg.pose.pose.position.z = res.U;
                odom_gpgga_publisher_.publish(odo_msg);
#endif

#ifdef ROS2
                // publish pose
                sensor_msgs::msg::NavSatFix nav_msg;
                if(use_ros_time)
                	nav_msg.header.stamp = ros::Time::now();
                else
                	nav_msg.header.stamp = RosTime(packet->cur_sec);
                nav_msg.latitude = packet->posLat;
                nav_msg.longitude = packet->posLon;
                nav_msg.altitude = packet->posHgt;
                pose_gpgga_publisher_->publish(nav_msg);
                // publish path_gpgga_
                path_gpgga_.header.frame_id = IMU_fram;
                if(use_ros_time)
                	path_gpgga_.header.stamp = ros::Time::now();
                else
                	path_gpgga_.header.stamp = RosTime(packet->cur_sec);
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_gpgga_.header;
                pose.pose.position.x = res.E;
                pose.pose.position.y = res.N;
                pose.pose.position.z = res.U;
                path_gpgga_.poses.push_back(pose);
                path_gpgga_publisher_->publish(path_gpgga_);
                // publish odom
                nav_msgs::msg::Odometry odo_msg;
                if(use_ros_time)
                	odo_msg.header.stamp = ros::Time::now();
                else
                	odo_msg.header.stamp = RosTime(packet->cur_sec);
                odo_msg.header.frame_id = IMU_fram;
                odo_msg.pose.pose.position.x = res.E;
                odo_msg.pose.pose.position.y = res.N;
                odo_msg.pose.pose.position.z = res.U;
                odom_gpgga_publisher_->publish(odo_msg);
#endif
        }

        void AlgoT1RosDriver::publishGprmcData(AlgoGPRMCDataPacket *packet, char *data, int len)
        {
                Algo1010::ENU res = m_coordTrans.BLH2ENU(packet->posLat, packet->posLon, packet->posHgt);

                printf("publish GPRMC:index=%d, hasPPS=%d, pps=%.3fms, posType=%d, BLH=(%0.9f, %0.9f, %0.3f) ,ENU=(%0.3f, %0.3f, %0.3f)\n",
                       packet->index, packet->hasPPS, packet->cur_sec, packet->posType,
                       packet->posLat * RAD2DEG, packet->posLon * RAD2DEG, packet->posHgt,
                       res.E, res.N, res.U);

                if (packet->posType == 0)
                        return;

#ifdef ROS1
                // publish pose
                sensor_msgs::NavSatFixPtr nav_msg(new sensor_msgs::NavSatFix());
                RosTime timestamp_ros = RosTime(packet->cur_sec);
                if(use_ros_time)
                	nav_msg->header.stamp = ros::Time::now();
                else
                	nav_msg->header.stamp = timestamp_ros;
                nav_msg->latitude = packet->posLat;
                nav_msg->longitude = packet->posLon;
                nav_msg->altitude = packet->posHgt;
                pose_gprmc_publisher_.publish(nav_msg);

                // publish path_gprmc_
                path_gprmc_.header.frame_id = IMU_fram;
                if(use_ros_time)
                	path_gprmc_.header.stamp = ros::Time::now();
                else
                	path_gprmc_.header.stamp = RosTime(packet->cur_sec);
                geometry_msgs::PoseStamped pose;
                pose.header = path_gprmc_.header;
                pose.pose.position.x = res.E;
                pose.pose.position.y = res.N;
                pose.pose.position.z = res.U;
                path_gprmc_.poses.push_back(pose);
                path_gprmc_publisher_.publish(path_gprmc_);
#endif

#ifdef ROS2
                // publish pose
                sensor_msgs::msg::NavSatFix nav_msg;
                RosTime timestamp_ros = RosTime(packet->cur_sec);
                if(use_ros_time)
                	nav_msg.header.stamp = ros::Time::now();
                else
                	nav_msg.header.stamp = timestamp_ros;
                nav_msg.latitude = packet->posLat;
                nav_msg.longitude = packet->posLon;
                nav_msg.altitude = packet->posHgt;
                pose_gprmc_publisher_->publish(nav_msg);

                // publish path_gprmc_
                path_gprmc_.header.frame_id = IMU_fram;
                if(use_ros_time)
                	path_gprmc_.header.stamp = ros::Time::now();
                else
                	path_gprmc_.header.stamp = RosTime(packet->cur_sec);
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_gprmc_.header;
                pose.pose.position.x = res.E;
                pose.pose.position.y = res.N;
                pose.pose.position.z = res.U;
                path_gprmc_.poses.push_back(pose);
                path_gprmc_publisher_->publish(path_gprmc_);
#endif
        }

        void AlgoT1RosDriver::publishMemsData(AlgoImuDataPacket *packet, char *data, int len)
        {
                // print 1/10
                if (packet->index % 10 == 0)
                {
                        printf("publish IMU:index=%d, hasPPS=%d, pps=%.3f ms, accel_gyro(%.6f,%.6f,%.6f,%.6f,%.6f,%.6f)\n",
                               packet->index, packet->hasPPS, packet->cur_sec,
                               packet->accel_x, packet->accel_y, packet->accel_z, packet->gyro_x, packet->gyro_y, packet->gyro_z);
                }

#ifdef ROS1
                sensor_msgs::Imu imu_msg;
                double unix_time = gps_to_unix(packet->cur_week, packet->cur_sec);
                if(use_ros_time)
                	imu_msg.header.stamp = ros::Time::now();
                else
                	imu_msg.header.stamp = RosTime(unix_time);
                imu_msg.header.frame_id = IMU_fram;
                imu_msg.linear_acceleration.x = packet->accel_x * 9.80665;
                imu_msg.linear_acceleration.y = packet->accel_y * 9.80665;
                imu_msg.linear_acceleration.z = packet->accel_z * 9.80665;
                imu_msg.angular_velocity.x = packet->gyro_x * DEG2RAD;
                imu_msg.angular_velocity.y = packet->gyro_y * DEG2RAD;
                imu_msg.angular_velocity.z = packet->gyro_z * DEG2RAD;
                // imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(eular[0], eular[1], eular[2]);
                imu_publisher_.publish(imu_msg);
#endif

#ifdef ROS2
                sensor_msgs::msg::Imu imu_msg;
                if(use_ros_time)
                	imu_msg.header.stamp = ros::Time::now();
                else
                	imu_msg.header.stamp = RosTime(packet->cur_sec);
                imu_msg.header.frame_id = IMU_fram;
                imu_msg.linear_acceleration.x = packet->accel_x * 9.80665;
                imu_msg.linear_acceleration.y = packet->accel_y * 9.80665;
                imu_msg.linear_acceleration.z = packet->accel_z * 9.80665;
                imu_msg.angular_velocity.x = packet->gyro_x * DEG2RAD;
                imu_msg.angular_velocity.y = packet->gyro_y * DEG2RAD;
                imu_msg.angular_velocity.z = packet->gyro_z * DEG2RAD;
                // imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(eular[0], eular[1], eular[2]);
                imu_publisher_->publish(imu_msg);
#endif
        }

        void AlgoT1RosDriver::publishVinsData(AlgoVINSPOSDataPacket *packet, char *data, int len)
        {

                printf("publish VINS:index=%d,  hasPPS=%d, pps=%.3fms, P=(%0.3f, %0.3f, %0.3f), V=(%0.3f, %0.3f, %0.3f)\n",
                       packet->index, packet->hasPPS, packet->cur_sec, packet->P[0], packet->P[1], packet->P[2],
                       packet->V[0], packet->V[1], packet->V[2]);

#ifdef ROS1
                // publish pose
                sensor_msgs::NavSatFixPtr nav_msg(new sensor_msgs::NavSatFix());
                nav_msg->header.frame_id = IMU_fram;
                if(use_ros_time)
                	nav_msg->header.stamp = ros::Time::now();
                else
                	nav_msg->header.stamp = RosTime(packet->cur_sec);
                nav_msg->latitude = packet->P[0];
                nav_msg->longitude = packet->P[1];
                nav_msg->altitude = packet->P[2];
                pose_vins_publisher_.publish(nav_msg);
                static double last_vins_stamp = 0;
                if (packet->cur_sec - last_vins_stamp > 0.1)
                {
                        // publish path_inspvaxa_
                        path_vins_.header.frame_id = IMU_fram;
                        if(use_ros_time)
                		path_vins_.header.stamp = ros::Time::now();
                	 else
                        	path_vins_.header.stamp = RosTime(packet->cur_sec);
                        geometry_msgs::PoseStamped pose;
                        pose.header = path_inspvaxa_.header;
                        pose.pose.position.x = packet->P[0];
                        pose.pose.position.y = packet->P[1];
                        pose.pose.position.z = packet->P[2];
                        path_vins_.poses.push_back(pose);
                        path_vins_publisher_.publish(path_vins_);
                        last_vins_stamp = packet->cur_sec;
                }
#endif

#ifdef ROS2
                // publish pose
                sensor_msgs::msg::NavSatFix nav_msg;
                nav_msg.header.frame_id = IMU_fram;
                if(use_ros_time)
                	nav_msg.header.stamp = ros::Time::now();
                else
                	nav_msg.header.stamp = RosTime(packet->cur_sec);
                nav_msg.latitude = packet->P[0];
                nav_msg.longitude = packet->P[1];
                nav_msg.altitude = packet->P[2];
                pose_vins_publisher_->publish(nav_msg);

                static double last_vins_stamp = 0;
                if (packet->cur_sec - last_vins_stamp > 0.1)
                {
                        // publish path_inspvaxa_
                        path_vins_.header.frame_id = IMU_fram;
                        if(use_ros_time)
                		path_vins_.header.stamp = ros::Time::now();
               	 else
                       	path_vins_.header.stamp = RosTime(packet->cur_sec);
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = path_inspvaxa_.header;
                        pose.pose.position.x = packet->P[0];
                        pose.pose.position.y = packet->P[1];
                        pose.pose.position.z = packet->P[2];
                        path_vins_.poses.push_back(pose);
                        path_vins_publisher_->publish(path_vins_);

                        last_vins_stamp = packet->cur_sec;
                }
#endif
        }

        void AlgoT1RosDriver::publishGnssState(AlgoGPGGADataPacket *packet, char *data, int len)
        {

#ifdef ROS1
                printf("publish GnssState:index=%d, hasPPS=%d, pps=%.3fms\n",
                       packet->index, packet->hasPPS, packet->cur_sec);

                // algot1_msgs::Gins gins;
                // gins.header.stamp = RosTime(packet->cur_sec);
#endif

#ifdef ROS2
                // publish GnssState
                algot1_msgs::msg::GnssState gs;

                // from gga
                if(use_ros_time)
                	gs.header.stamp = ros::Time::now();
                else
                	gs.header.stamp = RosTime(packet->cur_sec);
                gs.header.frame_id = IMU_fram;
                gs.pos_state = packet->posType;
                gs.diff_age = (short)packet->diffAge;

                // from gins
                mutex_inspvaxa_.lock();
                gs.ins_state = packet_inspvaxa_.insStatus;
                gs.heading_state = packet_inspvaxa_.insStatus;
                gs.state = packet_inspvaxa_.insStatus;
                mutex_inspvaxa_.unlock();

                diy_gnss_state_publisher_->publish(gs);

                printf("publish GnssState:index=%d, hasPPS=%d, pps=%.3fms, state=%d,ins_state=%d,pos_state=%d,heading_state=%d,diff_age=%.1fs\n",
                       packet->index, packet->hasPPS, packet->cur_sec,
                       gs.state, gs.ins_state, gs.pos_state, gs.heading_state, gs.diff_age);
#endif
        }

#ifdef ROS2
        geometry_msgs::msg::Quaternion AlgoT1RosDriver::createQuaternionMsgFromYaw(double yaw)
        {
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                return tf2::toMsg(q);
        }
#endif

}
