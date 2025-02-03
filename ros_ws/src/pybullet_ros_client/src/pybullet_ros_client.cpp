#include <ros/ros.h>
#include <sensor_msgs/JointState.h>  // Сообщение с данными о шарнирах
#include <sensor_msgs/Joy.h>         // Сообщение с данными от джойстика
#include <geometry_msgs/Twist.h>     // Сообщение с командами по скорости
#include <geometry_msgs/Quaternion.h>  // Сообщение с кватернионом
#include <nav_msgs/Odometry.h>      // Сообщение с данными об одометрии
#include <tf/transform_broadcaster.h> // Для публикации tf
#include <tf/transform_datatypes.h>  // Для createQuaternionMsgFromYaw

class PyBulletRosClient
{
public:
    PyBulletRosClient()
    {
        ros::NodeHandle nh;

        // Подписка на сообщения джойстика
        joy_sub = nh.subscribe("/joy", 10, &PyBulletRosClient::joyCallback, this);

        // Подписка на сообщения о шарниры
        joint_state_sub = nh.subscribe("/joint_states", 10, &PyBulletRosClient::jointStateCallback, this);

        // Публикация команд движения
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Публикация данных об одометрии
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

        // Максимальные скорости
        max_linear_velocity = 4.0;
        max_angular_velocity = 2.0;

        last_time = ros::Time::now();
        x = 0.0;
        y = 0.0;
        theta = 0.0;
    }

    // Callback функция для обработки сообщений от джойстика
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        geometry_msgs::Twist cmd_vel_msg;

        // Преобразование данных джойстика в команду скорости
        cmd_vel_msg.linear.x = msg->axes[1]*max_linear_velocity;  // Линейная скорость по оси X
        cmd_vel_msg.angular.z = msg->axes[0]*max_angular_velocity; // Угловая скорость по оси Z
        cmd_vel_pub.publish(cmd_vel_msg);  // Публикация команду движения
    }

    // Callback функция для обработки данных о шарнирах
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        // Вычисление скорости колес на основе полученных данных
        double left_wheel_vel = (msg->velocity[0]) / 13.325;
        double right_wheel_vel = (msg->velocity[2]) / 13.325;

        // Расчет линейной и угловой скорости
        linear_velocity = (left_wheel_vel + right_wheel_vel) / 2.0;
        angular_velocity = (right_wheel_vel - left_wheel_vel) / wheel_distance;

        // Получение углов поворота колес из joint_states
        double l_f_wheel_angle = msg->position[0];  // Угол левого колеса
        double l_r_wheel_angle = msg->position[1];  // Угол левого колеса
        double r_f_wheel_angle = msg->position[2]; // Угол правого колеса
        double r_r_wheel_angle = msg->position[3]; // Угол правого колеса

        // Вычисление времени между обновлениями
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        updateOdometry(dt);  // Обновление данных о положении робота
        publishOdometry(current_time);  // Публикация одометрии
        broadcastTF(current_time);  // Публикация TF
        broadcastWheelTransforms(current_time, l_f_wheel_angle, l_r_wheel_angle, r_f_wheel_angle, r_r_wheel_angle);  // Публикация TF для колес с углом

    }

    // Обновление данных о положении робота
    void updateOdometry(double dt)
    {
        x += linear_velocity * dt * cos(theta);  // Обновление позиции по X
        y += linear_velocity * dt * sin(theta);  // Обновление позиции по Y
        theta += angular_velocity * dt;  // Обновление ориентации робота
    }

    // Публикация сообщения об одометрии
    void publishOdometry(ros::Time current_time)
    {
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "root_link";

        odom.pose.pose.position.x = x;  // Позиция по оси X
        odom.pose.pose.position.y = y;  // Позиция по оси Y
        odom.pose.pose.position.z = 0.0;  // Позиция по оси Z (плоская модель)
        
        // Создание кватерниона ориентации
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        odom.pose.pose.orientation = odom_quat;

        // Линейная и угловая скорость
        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.angular.z = angular_velocity;

        odom_pub.publish(odom);  // Публикация сообщения об одометрии
    }

    // Публикация TF
    void broadcastTF(ros::Time current_time)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;

        // Определение положения и ориентации
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, theta);
        transform.setRotation(q);

        // Отправка TF
        br.sendTransform(tf::StampedTransform(transform, current_time, "odom", "root_link"));
    }

    // Публикация TF для колес
    void broadcastWheelTransforms(ros::Time current_time, double l_f_wheel_angle, double l_r_wheel_angle, double r_f_wheel_angle, double r_r_wheel_angle)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;

        // Преобразования для левого и правого колес относительно base_link
        tf::Quaternion wheel_quat_l_f;
        tf::Quaternion wheel_quat_l_r;
        tf::Quaternion wheel_quat_r_f;
        tf::Quaternion wheel_quat_r_r;

        // Преобразуем углы колес в кватернионы
        wheel_quat_l_f.setRPY(0, l_f_wheel_angle, 0);  // Угол поворота левого колеса
        wheel_quat_l_r.setRPY(0, l_r_wheel_angle, 0);  // Угол поворота левого колеса
        wheel_quat_r_f.setRPY(0, r_f_wheel_angle, 0); // Угол поворота правого колеса
        wheel_quat_r_r.setRPY(0, r_f_wheel_angle, 0); // Угол поворота правого колеса

        tf::Vector3 wheel_offset_l_f(wheel_distance / 2, wheel_distance / 2, -0.08);  // Сдвиг для левого колеса
        tf::Vector3 wheel_offset_l_r(-wheel_distance / 2, wheel_distance / 2, -0.08);  // Сдвиг для левого колеса
        tf::Vector3 wheel_offset_r_f(wheel_distance / 2, -wheel_distance / 2, -0.08);  // Сдвиг для правого колеса
        tf::Vector3 wheel_offset_r_r(-wheel_distance / 2, -wheel_distance / 2, -0.08);  // Сдвиг для правого колеса

        transform.setOrigin(wheel_offset_l_f);
        transform.setRotation(wheel_quat_l_f);
        br.sendTransform(tf::StampedTransform(transform, current_time, "base_link", "base_to_l_f_wheel"));

        transform.setOrigin(wheel_offset_l_r);
        transform.setRotation(wheel_quat_l_r);
        br.sendTransform(tf::StampedTransform(transform, current_time, "base_link", "base_to_l_r_wheel"));

        transform.setOrigin(wheel_offset_r_f);
        transform.setRotation(wheel_quat_r_f);
        br.sendTransform(tf::StampedTransform(transform, current_time, "base_link", "base_to_r_f_wheel"));

        transform.setOrigin(wheel_offset_r_r);
        transform.setRotation(wheel_quat_r_r);
        br.sendTransform(tf::StampedTransform(transform, current_time, "base_link", "base_to_r_r_wheel"));
    }

private:
    ros::Subscriber joy_sub;  // Подписка на сообщения джойстика
    ros::Subscriber joint_state_sub;  // Подписка на сообщения о шарнирах
    ros::Publisher cmd_vel_pub;  // Публикация команд движения
    ros::Publisher odom_pub;  // Публикация данных об одометрии

    float max_linear_velocity, max_angular_velocity;  // Максимальная линейная и угловая скорости
    const float wheel_distance = 0.45;  // Расстояние между колесами робота
    double x, y, theta;  // Положение и ориентация робота
    ros::Time last_time;  // Время последнего обновления
    double linear_velocity, angular_velocity;  // Текущие линейная и угловая скорость
};

int main(int argc, char** argv)
{

    ros::init(argc, argv, "pybullet_ros_client_cpp");
    PyBulletRosClient pybullet_ros_client;
    ros::spin();

    return 0;
}
