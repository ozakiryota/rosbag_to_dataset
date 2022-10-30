#include <filesystem>
#include <nlohmann/json.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RosbagToDataset{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*buffer*/
        std::vector<std::string> topic_name_list_;
        size_t num_save_ = 0;
        ros::Time last_saved_stamp_;
        nav_msgs::Odometry last_odom_;
        /*file*/
        std::ofstream file_list_csv_;
        /*message*/
        struct Topic{
            std::string topic_name;
            bool is_buffered = false;
            ros::Publisher debug_pub;
        };
        struct CompressedImageTopic : Topic{
            sensor_msgs::CompressedImageConstPtr msg_ptr;
        };
        struct PcTopic : Topic{
            sensor_msgs::PointCloud2ConstPtr msg_ptr;
        };
        struct ImuTopic : Topic{
            sensor_msgs::ImuConstPtr msg_ptr;
        };
        struct OdomTopic : Topic{
            nav_msgs::OdometryConstPtr msg_ptr;
        };
        std::vector<CompressedImageTopic> compressedimage_topic_list_;
        std::vector<PcTopic> pc_topic_list_;
        std::vector<ImuTopic> imu_topic_list_;
        OdomTopic odom_topic_;
		/*parameter*/
		std::string rosbag_path_;
		std::string save_dir_;
		std::string save_csv_path_;
		std::string save_txt_path_;
        float min_time_diff_sec_;
        float min_odom_diff_m_;
        float min_odom_diff_deg_;
        float debug_hz_;
		std::string debug_frame_;
        /*function*/
        std::string getDefaultSaveDir();
        void getTopicList();
        bool isReadyToSave(const rosbag::View::iterator& view_itr);
        void getOdomDiff(float& diff_m, float& diff_deg);
		void save();
		void publishDebugMsg();

	public:
		RosbagToDataset();
        void createDirectory();
        void execute();
};

RosbagToDataset::RosbagToDataset()
	: nh_private_("~")
{
	std::cout << "----- rosbag_to_dataset -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("rosbag_path", rosbag_path_)){
        std::cerr << "Set rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "rosbag_path_ = " << rosbag_path_ << std::endl;
    nh_private_.param("save_dir", save_dir_, getDefaultSaveDir());
	std::cout << "save_dir_ = " << save_dir_ << std::endl;
    nh_private_.param("save_csv_path", save_csv_path_, std::string(save_dir_ + "/file_list.csv"));
	std::cout << "save_csv_path_ = " << save_csv_path_ << std::endl;
    nh_private_.param("save_txt_path", save_txt_path_, std::string(save_dir_ + "/param.txt"));
	std::cout << "save_txt_path_ = " << save_txt_path_ << std::endl;

    nh_private_.param("min_time_diff_sec", min_time_diff_sec_, float(-1));
	std::cout << "min_time_diff_sec_ = " << min_time_diff_sec_ << std::endl;
    nh_private_.param("min_odom_diff_m", min_odom_diff_m_, float(-1));
	std::cout << "min_odom_diff_m_ = " << min_odom_diff_m_ << std::endl;
    nh_private_.param("min_odom_diff_deg", min_odom_diff_deg_, float(-1));
	std::cout << "min_odom_diff_deg_ = " << min_odom_diff_deg_ << std::endl;

    nh_private_.param("debug_hz", debug_hz_, float(-1));
	std::cout << "debug_hz_ = " << debug_hz_ << std::endl;
    nh_private_.param("debug_frame", debug_frame_, std::string("debug"));
	std::cout << "debug_frame_ = " << debug_frame_ << std::endl;

    getTopicList();
}

std::string RosbagToDataset::getDefaultSaveDir()
{
    const char *tmp = getenv("ROS_WORKSPACE");
    std::string save_path(tmp ? tmp : "");
    if(save_path.empty()){
        std::cerr << "Cannot get $ROS_WORKSPACE." << std::endl;
        exit(true);
    }
    save_path += std::string("/src/rosbag_to_dataset/save/") + rosbag_path_;
    return save_path;
}

void RosbagToDataset::getTopicList()
{
    for(size_t i = 0; ; i++){
        CompressedImageTopic tmp_topic;
        if(!nh_private_.getParam("compressedimage_" + std::to_string(i), tmp_topic.topic_name))  break;
        tmp_topic.debug_pub = nh_.advertise<sensor_msgs::CompressedImage>(tmp_topic.topic_name, 1);;
        compressedimage_topic_list_.push_back(tmp_topic);
        topic_name_list_.push_back(tmp_topic.topic_name);
        std::cout << "compressedimage_topic_list_[" << i << "].topic_name = " << compressedimage_topic_list_[i].topic_name << std::endl;
    }
    for(size_t i = 0; ; i++){
        PcTopic tmp_topic;
        if(!nh_private_.getParam("pc_" + std::to_string(i), tmp_topic.topic_name))  break;
        tmp_topic.debug_pub = nh_.advertise<sensor_msgs::PointCloud2>(tmp_topic.topic_name, 1);;
        pc_topic_list_.push_back(tmp_topic);
        topic_name_list_.push_back(tmp_topic.topic_name);
        std::cout << "pc_topic_list_[" << i << "].topic_name = " << pc_topic_list_[i].topic_name << std::endl;
    }
    for(size_t i = 0; ; i++){
        ImuTopic tmp_topic;
        if(!nh_private_.getParam("imu_" + std::to_string(i), tmp_topic.topic_name))  break;
        tmp_topic.debug_pub = nh_.advertise<sensor_msgs::Imu>(tmp_topic.topic_name, 1);;
        imu_topic_list_.push_back(tmp_topic);
        topic_name_list_.push_back(tmp_topic.topic_name);
        std::cout << "imu_topic_list_[" << i << "].topic_name = " << imu_topic_list_[i].topic_name << std::endl;
    }
    if(nh_private_.getParam("odom", odom_topic_.topic_name)){
        odom_topic_.debug_pub = nh_.advertise<nav_msgs::Odometry>(odom_topic_.topic_name, 1);;
        topic_name_list_.push_back(odom_topic_.topic_name);
        std::cout << "odom_topic_.topic_name = " << odom_topic_.topic_name << std::endl;
    }
}

void RosbagToDataset::createDirectory()
{
    std::filesystem::remove_all(save_dir_);
    std::filesystem::create_directory(save_dir_);

    file_list_csv_.open(save_csv_path_, std::ios::app);
	if(!file_list_csv_){
		std::cout << "Cannot open " << save_csv_path_ << std::endl;
		exit(true);
	}

	std::ofstream param_txt;
    param_txt.open(save_txt_path_, std::ios::app);
	if(!param_txt){
		std::cout << "Cannot open " << save_txt_path_ << std::endl;
		exit(true);
	}
    param_txt << "rosbag_path_ = " << rosbag_path_ << std::endl
        << "min_time_diff_sec_ = " << min_time_diff_sec_ << std::endl
        << "min_odom_diff_m_ = " << min_odom_diff_m_ << std::endl
        << "min_odom_diff_deg_ = " << min_odom_diff_deg_ << std::endl;
	param_txt.close();
}

void RosbagToDataset::execute()
{
    rosbag::Bag bag;

    try{
        bag.open(rosbag_path_, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << rosbag_path_ << std::endl;
        exit(true);
    }

    rosbag::View view(bag, rosbag::TopicQuery(topic_name_list_));
    rosbag::View::iterator view_itr;
    view_itr = view.begin();

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        if(view_itr->getDataType() == "sensor_msgs/CompressedImage"){
            for(CompressedImageTopic& topic : compressedimage_topic_list_){
                if(view_itr->getTopic() == topic.topic_name){
                    topic.msg_ptr = view_itr->instantiate<sensor_msgs::CompressedImage>();
                    topic.is_buffered = true;
                    break;
                }
            }
        }
        else if(view_itr->getDataType() == "sensor_msgs/PointCloud2"){
            for(PcTopic& topic : pc_topic_list_){
                if(view_itr->getTopic() == topic.topic_name){
                    topic.msg_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
                    topic.is_buffered = true;
                    break;
                }
            }
        }
        else if(view_itr->getDataType() == "sensor_msgs/Imu"){
            for(ImuTopic& topic : imu_topic_list_){
                if(view_itr->getTopic() == topic.topic_name){
                    topic.msg_ptr = view_itr->instantiate<sensor_msgs::Imu>();
                    topic.is_buffered = true;
                    break;
                }
            }
        }
        else if(view_itr->getDataType() == "nav_msgs/Odometry"){
            if(view_itr->getTopic() == odom_topic_.topic_name){
                odom_topic_.msg_ptr = view_itr->instantiate<nav_msgs::Odometry>();
                odom_topic_.is_buffered = true;
            }
        }

        if(isReadyToSave(view_itr)){
            save();

            last_saved_stamp_ = view_itr->getTime();
            if(odom_topic_.topic_name != "")    last_odom_ = *odom_topic_.msg_ptr;
            num_save_++;

            publishDebugMsg();
            if(debug_hz_ > 0)    loop_rate.sleep();
        }
        view_itr++;
    }

    bag.close();
    file_list_csv_.close();
    std::cout << "Save: " << save_csv_path_ << std::endl;
}

bool RosbagToDataset::isReadyToSave(const rosbag::View::iterator& view_itr)
{
    for(CompressedImageTopic& topic : compressedimage_topic_list_){
        if(!topic.is_buffered)  return false;
    }
    for(PcTopic& topic : pc_topic_list_){
        if(!topic.is_buffered)  return false;
    }
    for(ImuTopic& topic : imu_topic_list_){
        if(!topic.is_buffered)  return false;
    }
    if(odom_topic_.topic_name != ""){
        if(!odom_topic_.is_buffered)  return false;
    }

    if(num_save_ == 0)  return true;

    if(min_time_diff_sec_ > 0){
        if((view_itr->getTime() - last_saved_stamp_).toSec() < min_time_diff_sec_)  return false;
    }
    if(odom_topic_.topic_name != ""){
        float diff_m, diff_deg;
        getOdomDiff(diff_m, diff_deg);
        if(min_odom_diff_m_ > 0 && diff_m < min_odom_diff_m_)   return false;
        if(min_odom_diff_deg_ > 0 && diff_deg < min_odom_diff_deg_)   return false;
    }

    return true;
}

void RosbagToDataset::getOdomDiff(float& diff_m, float& diff_deg)
{
    const nav_msgs::Odometry& curr_odom_ = *odom_topic_.msg_ptr;

    float diff_x = curr_odom_.pose.pose.position.x - last_odom_.pose.pose.position.x;
    float diff_y = curr_odom_.pose.pose.position.y - last_odom_.pose.pose.position.y;
    float diff_z = curr_odom_.pose.pose.position.z - last_odom_.pose.pose.position.z;
    diff_m = std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    tf::Quaternion curr_q, last_q;
    quaternionMsgToTF(curr_odom_.pose.pose.orientation, curr_q);
    quaternionMsgToTF(last_odom_.pose.pose.orientation, last_q);
    diff_deg = last_q.angleShortestPath(curr_q) / M_PI * 180.0;
}

void RosbagToDataset::save()
{
    std::string save_sub_dir = save_dir_ + "/" + std::to_string(num_save_);
    std::filesystem::create_directory(save_sub_dir);

    std::string save_json_path = save_sub_dir + "/data.json";
    std::ofstream data_json(save_json_path);
    nlohmann::json json_data;

    for(CompressedImageTopic& topic : compressedimage_topic_list_){
        std::string save_path = topic.topic_name;
        std::replace(save_path.begin() + 1, save_path.end(), '/', '_');
        save_path = save_sub_dir + save_path + ".jpeg";

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(topic.msg_ptr, sensor_msgs::image_encodings::BGR8);
        cv::imwrite(save_path, cv_ptr->image);
        file_list_csv_ << save_path << ",";
        std::cout << "Save: " << save_path << std::endl;

        topic.is_buffered = false;
    }
    for(PcTopic& topic : pc_topic_list_){
        std::string save_path = topic.topic_name;
        std::replace(save_path.begin() + 1, save_path.end(), '/', '_');
        save_path = save_sub_dir + save_path + ".pcd";

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*topic.msg_ptr, *pcl_pc);
        pcl::io::savePCDFileASCII(save_path, *pcl_pc);
        file_list_csv_ << save_path << ",";
        std::cout << "Save: " << save_path << std::endl;

        topic.is_buffered = false;
    }
    for(ImuTopic& topic : imu_topic_list_){
        json_data[topic.topic_name]["stamp"] = topic.msg_ptr->header.stamp.toSec();
        json_data[topic.topic_name]["angular_velocity"]["x"] = topic.msg_ptr->angular_velocity.x;
        json_data[topic.topic_name]["angular_velocity"]["y"] = topic.msg_ptr->angular_velocity.y;
        json_data[topic.topic_name]["angular_velocity"]["z"] = topic.msg_ptr->angular_velocity.z;
        json_data[topic.topic_name]["linear_acceleration"]["x"] = topic.msg_ptr->linear_acceleration.x;
        json_data[topic.topic_name]["linear_acceleration"]["y"] = topic.msg_ptr->linear_acceleration.y;
        json_data[topic.topic_name]["linear_acceleration"]["z"] = topic.msg_ptr->linear_acceleration.z;

        topic.is_buffered = false;
    }
    if(odom_topic_.topic_name != ""){
        json_data[odom_topic_.topic_name]["stamp"] = odom_topic_.msg_ptr->header.stamp.toSec();
        json_data[odom_topic_.topic_name]["position"]["x"] = odom_topic_.msg_ptr->pose.pose.position.x;
        json_data[odom_topic_.topic_name]["position"]["y"] = odom_topic_.msg_ptr->pose.pose.position.y;
        json_data[odom_topic_.topic_name]["position"]["z"] = odom_topic_.msg_ptr->pose.pose.position.z;
        json_data[odom_topic_.topic_name]["orientation"]["x"] = odom_topic_.msg_ptr->pose.pose.orientation.x;
        json_data[odom_topic_.topic_name]["orientation"]["y"] = odom_topic_.msg_ptr->pose.pose.orientation.y;
        json_data[odom_topic_.topic_name]["orientation"]["z"] = odom_topic_.msg_ptr->pose.pose.orientation.z;

        odom_topic_.is_buffered = false;
    }

    data_json << json_data;
    data_json.close();
    std::cout << "Save: " << save_json_path << std::endl;
    file_list_csv_ << save_json_path << std::endl;
}

void RosbagToDataset::publishDebugMsg()
{
    for(CompressedImageTopic& topic : compressedimage_topic_list_){
        sensor_msgs::CompressedImage debug_msg = *topic.msg_ptr;
        debug_msg.header.frame_id = debug_frame_;
        topic.debug_pub.publish(debug_msg);
    }
    for(PcTopic& topic : pc_topic_list_){
        sensor_msgs::PointCloud2 debug_msg = *topic.msg_ptr;
        debug_msg.header.frame_id = debug_frame_;
        topic.debug_pub.publish(debug_msg);
    }
    for(ImuTopic& topic : imu_topic_list_){
        sensor_msgs::Imu debug_msg = *topic.msg_ptr;
        debug_msg.header.frame_id = debug_frame_;
        topic.debug_pub.publish(debug_msg);
    }
    if(odom_topic_.topic_name != ""){
        nav_msgs::Odometry debug_msg = *odom_topic_.msg_ptr;
        debug_msg.header.frame_id = debug_frame_;
        odom_topic_.debug_pub.publish(debug_msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_to_dataset");
	
	RosbagToDataset rosbag_to_dataset;
    rosbag_to_dataset.createDirectory();
    rosbag_to_dataset.execute();
}