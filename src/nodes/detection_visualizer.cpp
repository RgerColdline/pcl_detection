/**
 * @file detection_visualizer.cpp
 * @brief RViz 可视化节点
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_detection/ObjectDetectionResult.h>

class DetectionVisualizer
{
public:
    DetectionVisualizer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh)
        , pnh_(pnh)
    {
        std::string result_topic;
        std::string marker_topic;
        
        pnh_.param("result_topic", result_topic, std::string("/pcl_detection/result"));
        pnh_.param("marker_topic", marker_topic, std::string("/pcl_detection/visualization/marker"));

        result_sub_ = nh_.subscribe(result_topic, 10, &DetectionVisualizer::resultCallback, this);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 10);

        ROS_INFO("DetectionVisualizer initialized");
    }

private:
    void resultCallback(const pcl_detection::ObjectDetectionResult::ConstPtr& msg)
    {
        visualization_msgs::MarkerArray marker_array;
        
        for (size_t i = 0; i < msg->objects.size(); ++i) {
            const auto& obj = msg->objects[i];
            visualization_msgs::Marker marker;
            
            marker.header = msg->header;
            marker.ns = "detected_objects";
            marker.id = i;
            marker.action = visualization_msgs::Marker::ADD;
            marker.lifetime = ros::Duration(1.0);
            
            marker.pose.position = obj.position;
            marker.pose.orientation.w = 1.0;
            
            switch (obj.type) {
                case 0:  // Wall
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.scale.x = obj.width;
                    marker.scale.y = 0.05;
                    marker.scale.z = obj.height;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.5;
                    break;
                    
                case 1:  // Cylinder
                    marker.type = visualization_msgs::Marker::CYLINDER;
                    marker.scale.x = obj.radius * 2;
                    marker.scale.y = obj.radius * 2;
                    marker.scale.z = obj.height;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.5;
                    break;
                    
                case 2:  // Circle
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.scale.x = obj.radius * 2;
                    marker.scale.y = obj.radius * 2;
                    marker.scale.z = 0.1;
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    marker.color.a = 0.7;
                    break;
            }
            
            marker_array.markers.push_back(marker);
        }
        
        if (!marker_array.markers.empty()) {
            // 每 10 次发布输出一次日志
            static int publish_count = 0;
            publish_count++;
            if (publish_count % 10 == 0) {
                ROS_INFO("Publishing %lu markers", marker_array.markers.size());
            }
            marker_pub_.publish(marker_array);
        }
    }

private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    
    ros::Subscriber result_sub_;
    ros::Publisher marker_pub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    DetectionVisualizer visualizer(nh, pnh);
    
    ros::spin();
    
    return 0;
}
