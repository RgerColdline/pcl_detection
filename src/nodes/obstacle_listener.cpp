/**
 * @file obstacle_listener.cpp
 * @brief 障碍物检测监听测试节点
 * 
 * 功能：订阅 /pcl_detection/result 话题，打印检测到的障碍物信息
 * 用途：测试障碍物检测功能，验证数据是否正确发布
 */

#include <ros/ros.h>
#include <pcl_detection/ObjectDetectionResult.h>
#include <pcl_detection/DetectedObject.h>

#include <clocale>  // 支持中文输出

class ObstacleListener
{
public:
    ObstacleListener(ros::NodeHandle& nh)
    {
        // 订阅检测结果
        result_sub_ = nh.subscribe("/pcl_detection/result", 10,
                                   &ObstacleListener::resultCallback, this);
        
        ROS_INFO("===========================================");
        ROS_INFO("  障碍物监听节点已启动");
        ROS_INFO("  订阅话题：/pcl_detection/result");
        ROS_INFO("===========================================");
    }
    
    void run()
    {
        ros::spin();
    }

private:
    void resultCallback(const pcl_detection::ObjectDetectionResult::ConstPtr& msg)
    {
        // 检查检测是否成功
        if (!msg->success)
        {
            ROS_WARN("检测失败：%s", msg->status_message.c_str());
            return;
        }
        
        // 打印统计信息
        ROS_INFO(" ");
        ROS_INFO("========== 检测结果 ==========");
        ROS_INFO("墙体：%d 个，圆柱：%d 个，圆环：%d 个",
                 msg->wall_count, msg->cylinder_count, msg->circle_count);
        ROS_INFO("总耗时：%.2f ms", msg->total_time);
        ROS_INFO("  - 下采样：%.2f ms", msg->downsample_time);
        ROS_INFO("  - 法向量：%.2f ms", msg->normals_time);
        ROS_INFO("  - 墙体：%.2f ms", msg->wall_time);
        ROS_INFO("  - 圆柱：%.2f ms", msg->cylinder_time);
        ROS_INFO("  - 圆环：%.2f ms", msg->circle_time);
        
        // 遍历所有物体
        if (msg->objects.empty())
        {
            ROS_INFO("未检测到任何物体");
        }
        else
        {
            for (size_t i = 0; i < msg->objects.size(); ++i)
            {
                printObjectInfo(msg->objects[i], i + 1);
            }
        }
        ROS_INFO("================================\n");
    }
    
    void printObjectInfo(const pcl_detection::DetectedObject& obj, int index)
    {
        std::string type_name;
        switch (obj.type)
        {
            case 0: type_name = "墙体"; break;
            case 1: type_name = "圆柱"; break;
            case 2: type_name = "圆环"; break;
            default: type_name = "未知"; break;
        }
        
        ROS_INFO("【物体 %d】%s (%s)", index, obj.name.c_str(), type_name.c_str());
        ROS_INFO("  位置：(%.3f, %.3f, %.3f)", 
                 obj.position.x, obj.position.y, obj.position.z);
        ROS_INFO("  尺寸：%.3f x %.3f x %.3f m", 
                 obj.width, obj.height, obj.depth);
        ROS_INFO("  点数：%d", obj.point_count);
        ROS_INFO("  耗时：%.3f ms", obj.extraction_time);
        
        // 根据类型打印特有信息
        switch (obj.type)
        {
            case 0:  // 墙体
                printWallInfo(obj);
                break;
            case 1:  // 圆柱
                printCylinderInfo(obj);
                break;
            case 2:  // 圆环
                printCircleInfo(obj);
                break;
        }
    }
    
    void printWallInfo(const pcl_detection::DetectedObject& obj)
    {
        if (obj.plane_coeffs.size() >= 4)
        {
            double A = obj.plane_coeffs[0];
            double B = obj.plane_coeffs[1];
            double C = obj.plane_coeffs[2];
            double D = obj.plane_coeffs[3];
            
            ROS_INFO("  平面方程：%.3fx + %.3fy + %.3fz + %.3f = 0", A, B, C, D);
            
            // 计算法向量
            double norm = std::sqrt(A*A + B*B + C*C);
            double nx = A / norm;
            double ny = B / norm;
            double nz = C / norm;
            ROS_INFO("  法向量：(%.3f, %.3f, %.3f)", nx, ny, nz);
            
            // 计算距离
            double dist = std::abs(D) / norm;
            ROS_INFO("  距原点：%.3f m", dist);
        }
    }
    
    void printCylinderInfo(const pcl_detection::DetectedObject& obj)
    {
        ROS_INFO("  半径：%.3f m", obj.radius);
        
        // 计算水平距离
        double h_dist = std::sqrt(obj.position.x * obj.position.x + 
                                  obj.position.y * obj.position.y);
        ROS_INFO("  水平距离：%.3f m", h_dist);
    }
    
    void printCircleInfo(const pcl_detection::DetectedObject& obj)
    {
        ROS_INFO("  半径：%.3f m", obj.radius);
        
        // 计算距离
        double dist = std::sqrt(obj.position.x * obj.position.x + 
                               obj.position.y * obj.position.y + 
                               obj.position.z * obj.position.z);
        ROS_INFO("  距原点：%.3f m", dist);
    }

private:
    ros::Subscriber result_sub_;
};

int main(int argc, char** argv)
{
    // 设置中文 locale 支持
    setlocale(LC_ALL, "");
    
    ros::init(argc, argv, "obstacle_listener");
    ros::NodeHandle nh;
    
    ObstacleListener listener(nh);
    listener.run();
    
    return 0;
}
