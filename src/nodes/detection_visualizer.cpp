/**
 * @file detection_visualizer.cpp
 * @brief RViz 可视化节点
 * 
 * 功能：发布物体 Marker（墙体、圆柱、方环、障碍物）
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Quaternion.h>
#include <pcl_detection/ObjectDetectionResult.h>

#include <string>

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
        ROS_INFO("  Marker topic: %s", marker_topic.c_str());
    }

private:
    /**
     * @brief 从法向量计算四元数（用于墙体旋转）
     * 
     * 墙体法向量 (nx, ny, nz) 需要转换为绕 Z 轴的旋转
     * 假设墙体是竖直的，法向量在 XY 平面内
     */
    geometry_msgs::Quaternion quaternionFromNormal(float nx, float ny, float /*nz*/)
    {
        // 计算法向量在 XY 平面的投影角度
        float yaw = std::atan2(ny, nx);
        
        // 墙体法向量垂直于墙面，而 CUBE 的 X 轴是长边，所以需要旋转 90 度
        // 让 CUBE 的长边沿着墙体方向
        yaw += M_PI_2;
        
        // 转换为四元数（只绕 Z 轴旋转）
        geometry_msgs::Quaternion q;
        q.w = std::cos(yaw / 2.0f);
        q.x = 0.0f;
        q.y = 0.0f;
        q.z = std::sin(yaw / 2.0f);
        
        return q;
    }

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
            marker.lifetime = ros::Duration(2.0);  // 延长生命周期到 2 秒

            marker.pose.position = obj.position;
            marker.pose.orientation.w = 1.0;

            switch (obj.type) {
                case 0:  // Wall
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.scale.x = obj.width;
                    marker.scale.y = 0.05;  // 墙体厚度
                    marker.scale.z = obj.height;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.6;
                    
                    // 根据法向量计算旋转
                    if (obj.plane_coeffs.size() >= 4) {
                        float nx = obj.plane_coeffs[0];
                        float ny = obj.plane_coeffs[1];
                        marker.pose.orientation = quaternionFromNormal(nx, ny, 0);
                    }
                    break;

                case 1:  // Cylinder (包括障碍物)
                    marker.type = visualization_msgs::Marker::CYLINDER;
                    marker.scale.x = obj.radius * 2;
                    marker.scale.y = obj.radius * 2;
                    marker.scale.z = obj.height > 0 ? obj.height : 1.0;
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
                
                case 3:  // Rectangle (方环)
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.scale.x = obj.width;   // 长边
                    marker.scale.y = obj.height;  // 短边
                    marker.scale.z = 0.1;         // 厚度
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.6;
                    
                    // 方环的 rotation 存储在 radius 字段中（角度制）
                    if (obj.radius != 0.0f) {
                        float yaw = obj.radius * M_PI / 180.0f;
                        geometry_msgs::Quaternion q;
                        q.w = std::cos(yaw / 2.0f);
                        q.x = 0.0f;
                        q.y = 0.0f;
                        q.z = std::sin(yaw / 2.0f);
                        marker.pose.orientation = q;
                    }
                    break;
            }

            marker_array.markers.push_back(marker);
        }

        if (!marker_array.markers.empty()) {
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
