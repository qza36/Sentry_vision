#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include <math.h>
#include <vector>
#include <chrono>
#include <iomanip> 
#define _USE_MATH_DEFINES 
#include <cmath>



std::chrono::steady_clock::time_point last_time_ = std::chrono::steady_clock::now();

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double orientationToYaw(const geometry_msgs::msg::Quaternion& orientation) {
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double yaw, pitch, roll;
    m.getEulerYPR(yaw, pitch, roll);
    return yaw;
}


void ex(double& a,double& min,double& max)
{
    if(a<min)
    {
        min = a;
    }
    if(a>max)
    {
        max = a;
    }
        
    

}


class TopicSubscribe01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicSubscribe01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点开始构建.", name.c_str());

        command_subscribe_ = this->create_subscription<rm_interfaces::msg::Armors>(
            "armor_detector/armors", rclcpp::SensorDataQoS(),std::bind(&TopicSubscribe01::command_callback,this,std::placeholders::_1));
     
    }

private:
    // 声明节点
    rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr command_subscribe_;
    std::vector<rm_interfaces::msg::Armors> datas;
    double R_x = 0.0;
    double R_y = 0.0;
    double R_z = 0.0;
    double R_yaw = 0.0;
    double time = 0.0;

    double s2qx=0.0;
    double s2qy=0.0;
    double s2qz=0.0;
    double s2qyaw=0.0;


    double s2qx_min = 0.1;
    double s2qx_max = 100.0;
    double s2qy_min = 0.1;
    double s2qy_max = 100.0;
    double s2qz_min = 0.1;
    double s2qz_max = 100.0;
    double s2qyaw_min = 0.1;
    double s2qyaw_max = 100.0;
    double last_x_ = 0.0, last_y_ = 0.0, last_z_ = 0.0, last_yaw_ = 0.0;
    //std::chrono::steady_clock::time_point last_time_;


    


    void command_callback(const rm_interfaces::msg::Armors::SharedPtr msg)
    {
        std::cout<<"已经收集了"<<datas.size()<<"个数据"<<std::endl;
        datas.push_back(*msg);
        auto current_time = std::chrono::steady_clock::now();
        auto delta_time = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time_).count()/1000.0;
        //last_time_ = current_time;

        if (!msg->armors.empty()&&delta_time>0.5) {
            last_time_ = current_time;
            if(datas.size()==1)
            {
                last_x_ = msg->armors[0].pose.position.x;
                last_y_ = msg->armors[0].pose.position.y;
                last_z_ = msg->armors[0].pose.position.z;
                last_yaw_ = orientationToYaw(msg->armors[0].pose.orientation);

            }
            else{

                    double current_x = msg->armors[0].pose.position.x;
                    double current_y = msg->armors[0].pose.position.y;
                    double current_z = msg->armors[0].pose.position.z;
                    double current_yaw = orientationToYaw(msg->armors[0].pose.orientation);

                    if (delta_time > 0) {
                        double v_x = (current_x - last_x_) / delta_time;
                        double v_y = (current_y - last_y_) / delta_time;
                        double v_z = (current_z - last_z_) / delta_time;
                        double v_yaw = (current_yaw - last_yaw_) / delta_time;

                        // 调整Q的计算逻辑，例如速度增大时Q增大
                        s2qx = exp(-(abs(v_x) + 0.5*abs(v_yaw))) * (s2qx_max - s2qx_min) + s2qx_min;
                        ex(s2qx,s2qx_min,s2qx_max);
                        s2qy = exp(-(abs(v_y) + 0.5*abs(v_yaw))) * (s2qy_max - s2qy_min) + s2qy_min;
                        ex(s2qy,s2qy_min,s2qy_max);
                        s2qz = exp(-(abs(v_z) + 0.5*abs(v_yaw))) * (s2qz_max - s2qz_min) + s2qz_min;
                        ex(s2qz,s2qz_min,s2qz_max);
                        s2qyaw = exp(-(abs(v_x) + 0.5*abs(v_z))) * (s2qyaw_max - s2qyaw_min) + s2qyaw_min;
                        ex(s2qyaw,s2qyaw_min,s2qyaw_max);
                    }

                    // 更新前一次状态
                    last_x_ = current_x;
                    last_y_ = current_y;
                    last_z_ = current_z;
                    last_yaw_ = current_yaw;
                }
        }


        if(datas.size()==5000)
        {
            double all_x = 0.0;
            double all_y = 0.0;
            double all_z = 0.0;
            double all_yaw = 0.0;
    
          for (int i = 0; i < 5000; i++)
            {
                std::cout << i << std::endl;

                // 检查 datas[i].armors 是否为空
                if (!datas[i].armors.empty())
                {
                    time = time +1;
                    all_x += datas[i].armors[0].pose.position.x;
                    all_y += datas[i].armors[0].pose.position.y;
                    all_z += datas[i].armors[0].pose.position.z;
                    all_yaw += orientationToYaw(datas[i].armors[0].pose.orientation);

                }
                else
                {
                    // 如果 armors 为空，输出警告或跳过
                    RCLCPP_WARN(this->get_logger(), "No armors found in datas[%d]", i);
                }
            }
            double mean_x = all_x/time;
            double mean_y= all_y/time;
            double mean_z = all_z/time;
            double mean_yaw = all_yaw/time;
            

            double variance_x = 0.0;
            double variance_y = 0.0;
            double variance_z = 0.0;
            double variance_yaw = 0.0;
           
            for(int i = 0 ;i<5000;i++)
            {
                 if (!datas[i].armors.empty())
                 {
                 // 修改这里：使用 datas[i].armors[0] 访问第一个 Armor 对象
                    variance_x += pow(mean_x - datas[i].armors[0].pose.position.x, 2);
                    variance_y += pow(mean_y - datas[i].armors[0].pose.position.y, 2);
                    variance_z += pow(mean_z - datas[i].armors[0].pose.position.z, 2);
                    variance_yaw += pow(mean_yaw - orientationToYaw(datas[i].armors[0].pose.orientation), 2);
                 }
                
            }

            R_x = variance_x/time;
            R_y = variance_y/time;
            R_z = variance_z/time;
            R_yaw = variance_yaw/time;
            

        }

        if(datas.size()>5000)
        {
             // 设置输出格式为固定小数点，并设置小数点后的精度
            std::cout << std::fixed << std::setprecision(10);
            std::cout<<"R_x:"<<R_z<<std::endl;
            std::cout<<"R_y:"<<R_x<<std::endl;
            std::cout<<"R_z:"<<R_y<<std::endl;
            std::cout<<"R_yaw:"<<R_yaw<<std::endl;

            std::cout<<"s2qx:"<<s2qz<<std::endl;
            std::cout<<"s2qy:"<<s2qx<<std::endl;
            std::cout<<"s2qz:"<<s2qy<<std::endl;
            std::cout<<"s2qyaw:"<<s2qyaw<<std::endl;
        }

    
    }
};

int main(int argc, char **argv)
{
    
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicSubscribe01>("calculation_subscribe");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
