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
        /*publisher*/
		ros::Publisher image_pub_;
		ros::Publisher pc_pub_;
		ros::Publisher imu_pub_;
		ros::Publisher odom_pub_;
        /*buffer*/
        size_t num_save_ = 0;
        ros::Time last_saved_stamp_;
        nav_msgs::Odometry curr_odom_;
        nav_msgs::Odometry last_odom_;
        /*file*/
        std::ofstream file_list_csv_;
        /*message*/
        struct Topic{
            std::string msg_type;
            bool is_buffered = false;
            sensor_msgs::CompressedImageConstPtr image_ptr;
            sensor_msgs::PointCloud2ConstPtr pc_ptr;
            sensor_msgs::ImuConstPtr imu_ptr;
            nav_msgs::OdometryConstPtr odom_ptr;
        };
        std::vector<Topic> msg_ptr_list_;
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
        std::vector<std::string> topic_name_list_;
        /*function*/
        std::string getDefaultSaveDir();
        bool isReadyToSave(const rosbag::View::iterator& view_itr);
        void getOdomDiff(float& largest_diff_m, float& largest_diff_deg);
		void save();
		void publishDebugMsg();
        float anglePiToPi(const float& angle);

	public:
		RosbagToDataset();
        void createDirectory();
        void load();
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

    for(size_t i = 0; ; i++){
        std::string tmp_topic_name;
        if(!nh_private_.getParam("topic_" + std::to_string(i), tmp_topic_name))  break;
        topic_name_list_.push_back(tmp_topic_name);
        std::cout << "topic_name_list_[" << i << "] = " << topic_name_list_[i] << std::endl;
    }
    msg_ptr_list_.resize(topic_name_list_.size());

    /*publisher*/
	image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/debug/image/compressed", 1);
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/point_cloud", 1);
	imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/debug/imu", 1);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/debug/odom", 1);
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

void RosbagToDataset::load()
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
    view.addQuery(bag, rosbag::TypeQuery("tf2_msgs/TFMessage"));
    view_itr = view.begin();

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        for(size_t i = 0; i < topic_name_list_.size(); i++){
            if(view_itr->getTopic() == topic_name_list_[i]){
                msg_ptr_list_[i].msg_type = view_itr->getDataType();
                msg_ptr_list_[i].is_buffered = true;
                msg_ptr_list_[i].image_ptr = view_itr->instantiate<sensor_msgs::CompressedImage>();
                msg_ptr_list_[i].pc_ptr = view_itr->instantiate<sensor_msgs::PointCloud2>();
                msg_ptr_list_[i].imu_ptr = view_itr->instantiate<sensor_msgs::Imu>();
                msg_ptr_list_[i].odom_ptr = view_itr->instantiate<nav_msgs::Odometry>();
                if(msg_ptr_list_[i].msg_type == "nav_msgs/Odometry")    curr_odom_ = *view_itr->instantiate<nav_msgs::Odometry>();
            }
        }
        if(isReadyToSave(view_itr)){
            save();

            last_saved_stamp_ = view_itr->getTime();
            last_odom_ = curr_odom_;
            num_save_++;
            for(size_t i = 0; i < msg_ptr_list_.size(); i++)    msg_ptr_list_[i].is_buffered = false;

            publishDebugMsg();
            if(debug_hz_ > 0)    loop_rate.sleep();
        }
        view_itr++;
    }

    file_list_csv_.close();
    std::cout << "Save: " << save_csv_path_ << std::endl;
}

bool RosbagToDataset::isReadyToSave(const rosbag::View::iterator& view_itr)
{
    for(size_t i = 0; i < msg_ptr_list_.size(); i++){
        if(!msg_ptr_list_[i].is_buffered)    return false;
    }
    if(num_save_ == 0)  return true;
    if(min_time_diff_sec_ > 0){
        if((view_itr->getTime() - last_saved_stamp_).toSec() < min_time_diff_sec_)  return false;
    }
    float largest_diff_m, largest_diff_deg;
    getOdomDiff(largest_diff_m, largest_diff_deg);
    if(min_odom_diff_m_ > 0 && largest_diff_m < min_odom_diff_m_)   return false;
    if(min_odom_diff_deg_ > 0 && largest_diff_deg < min_odom_diff_deg_)   return false;

    return true;
}

void RosbagToDataset::getOdomDiff(float& largest_diff_m, float& largest_diff_deg)
{
    float diff_x = std::abs(curr_odom_.pose.pose.position.x - last_odom_.pose.pose.position.x);
    float diff_y = std::abs(curr_odom_.pose.pose.position.y - last_odom_.pose.pose.position.y);
    float diff_z = std::abs(curr_odom_.pose.pose.position.z - last_odom_.pose.pose.position.z);
    largest_diff_m = std::max(std::max(diff_x, diff_y), diff_z);

    tf::Quaternion curr_q, last_q;
    quaternionMsgToTF(curr_odom_.pose.pose.orientation, curr_q);
    quaternionMsgToTF(last_odom_.pose.pose.orientation, last_q);
    largest_diff_deg = last_q.angleShortestPath(curr_q) / M_PI * 180.0;
}

void RosbagToDataset::save()
{
    std::string save_sub_dir = save_dir_ + "/" + std::to_string(num_save_);
    std::filesystem::create_directory(save_sub_dir);

    std::string save_json_path = save_sub_dir + "/data.json";
    std::ofstream data_json(save_json_path);
    nlohmann::json json_data;

    for(size_t i = 0; i < msg_ptr_list_.size(); i++){
        std::string save_path = topic_name_list_[i];
        std::replace(save_path.begin() + 1, save_path.end(), '/', '_');
        save_path = save_sub_dir + save_path;
        if(msg_ptr_list_[i].msg_type == "sensor_msgs/CompressedImage"){
            save_path += ".jpeg";
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg_ptr_list_[i].image_ptr, sensor_msgs::image_encodings::BGR8);
            cv::imwrite(save_path, cv_ptr->image);
            file_list_csv_ << save_path << ",";
            std::cout << "Save: " << save_path << std::endl;
        }
        else if(msg_ptr_list_[i].msg_type == "sensor_msgs/PointCloud2"){
            save_path += ".pcd";
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*msg_ptr_list_[i].pc_ptr, *pcl_pc);
            pcl::io::savePCDFileASCII(save_path, *pcl_pc);
            file_list_csv_ << save_path << ",";
            std::cout << "Save: " << save_path << std::endl;
        }
        else if(msg_ptr_list_[i].msg_type == "sensor_msgs/Imu"){
            json_data[topic_name_list_[i]]["stamp"] = msg_ptr_list_[i].imu_ptr->header.stamp.toSec();
            json_data[topic_name_list_[i]]["angular_velocity"]["x"] = msg_ptr_list_[i].imu_ptr->angular_velocity.x;
            json_data[topic_name_list_[i]]["angular_velocity"]["y"] = msg_ptr_list_[i].imu_ptr->angular_velocity.y;
            json_data[topic_name_list_[i]]["angular_velocity"]["z"] = msg_ptr_list_[i].imu_ptr->angular_velocity.z;
            json_data[topic_name_list_[i]]["linear_acceleration"]["x"] = msg_ptr_list_[i].imu_ptr->linear_acceleration.x;
            json_data[topic_name_list_[i]]["linear_acceleration"]["y"] = msg_ptr_list_[i].imu_ptr->linear_acceleration.y;
            json_data[topic_name_list_[i]]["linear_acceleration"]["z"] = msg_ptr_list_[i].imu_ptr->linear_acceleration.z;
        }
        else if(msg_ptr_list_[i].msg_type == "nav_msgs/Odometry"){
            json_data[topic_name_list_[i]]["stamp"] = msg_ptr_list_[i].odom_ptr->header.stamp.toSec();
            json_data[topic_name_list_[i]]["position"]["x"] = msg_ptr_list_[i].odom_ptr->pose.pose.position.x;
            json_data[topic_name_list_[i]]["position"]["y"] = msg_ptr_list_[i].odom_ptr->pose.pose.position.y;
            json_data[topic_name_list_[i]]["position"]["z"] = msg_ptr_list_[i].odom_ptr->pose.pose.position.z;
            json_data[topic_name_list_[i]]["orientation"]["x"] = msg_ptr_list_[i].odom_ptr->pose.pose.orientation.x;
            json_data[topic_name_list_[i]]["orientation"]["y"] = msg_ptr_list_[i].odom_ptr->pose.pose.orientation.y;
            json_data[topic_name_list_[i]]["orientation"]["z"] = msg_ptr_list_[i].odom_ptr->pose.pose.orientation.z;
        }
    }
    data_json << json_data;
    data_json.close();
    std::cout << "Save: " << save_json_path << std::endl;
    file_list_csv_ << save_json_path << std::endl;
}

void RosbagToDataset::publishDebugMsg()
{
    for(size_t i = 0; i < msg_ptr_list_.size(); i++){
        if(msg_ptr_list_[i].msg_type == "sensor_msgs/CompressedImage"){
            sensor_msgs::CompressedImage debug_image = *msg_ptr_list_[i].image_ptr;
            debug_image.header.frame_id = debug_frame_;
            image_pub_.publish(debug_image);
        }
        else if(msg_ptr_list_[i].msg_type == "sensor_msgs/PointCloud2"){
            sensor_msgs::PointCloud2 debug_pc = *msg_ptr_list_[i].pc_ptr;
            debug_pc.header.frame_id = debug_frame_;
            pc_pub_.publish(debug_pc);
        }
        else if(msg_ptr_list_[i].msg_type == "sensor_msgs/Imu"){
            sensor_msgs::Imu debug_imu = *msg_ptr_list_[i].imu_ptr;
            debug_imu.header.frame_id = debug_frame_;
            imu_pub_.publish(debug_imu);
        }
        else if(msg_ptr_list_[i].msg_type == "nav_msgs/Odometry"){
            nav_msgs::Odometry debug_odom = *msg_ptr_list_[i].odom_ptr;
            debug_odom.header.frame_id = debug_frame_;
            odom_pub_.publish(debug_odom);
        }
    }
}

float RosbagToDataset::anglePiToPi(const float& angle)
{
	return atan2(sin(angle), cos(angle)); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_to_dataset");
	
	RosbagToDataset rosbag_to_dataset;
    rosbag_to_dataset.createDirectory();
    rosbag_to_dataset.load();
}