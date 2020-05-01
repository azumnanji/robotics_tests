#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

struct enc{
    int64_t prev_tick;
    int64_t curr_tick;
    enc(){
        prev_tick = 0;
        curr_tick = 0;
    }
};

class Odom{
    public:
        
        Odom(ros::NodeHandle &nh, unsigned int max_ticks, unsigned int min_ticks, double base_width, 
                 int freq, unsigned int ticks_per_meter) {
            _heading = 0;
            _x = 0;
            _y = 0;
            _base_width = base_width;
            _ticks_per_meter = ticks_per_meter;

            right = enc();
            left = enc();

            enc_right.subscribe(nh, "encoders/right_wheel", 5);
            enc_left.subscribe(nh, "encoders/left_wheel", 5);
            message_filters::TimeSynchronizer<std_msgs::Int64, std_msgs::Int64> sync(enc_right, enc_left, 10);
            sync.registerCallback(boost::bind(&Position::update, _1, _2));

            odometry = nh.advertise<nav_msgs::Odometry>("turtle_odom", freq);
        }

        void update(const std_msgs::Int64 & right_tick, const std_msgs::Int64 & left_tick) {
            curr_time = ros::Time::now().toSec();
            double time = curr_time - prev_time;
            right.curr_tick = right_tick.data;
            left.curr_tick = left_tick.data;

            for (int i = 0; i < 2; ++i) {
                enc & temp = i == 0 ? right:left;
                update_encoder(temp);
            }

            double dist_right = static_cast<double>(right.curr_tick) / _ticks_per_meter;
            double dist_left = static_cast<double>(left.curr_tick) / _ticks_per_meter;

            double heading_inc = (dist_right - dist_left) / _base_width;
            double x_inc = cos(heading_inc) * (dist_left + dist_right) / 2;
            double y_inc = sin(heading_inc) * (dist_left + dist_right) / 2;

            update_heading(heading_inc);

            double x_fixed = cos(_heading) * x_inc - sin(_heading) * y_inc;
            double y_fixed = sin(_heading) * x_inc + cos(_heading) * y_inc;
            _x += x_fixed;
            _y += y_fixed;

            _x_speed = x_fixed / time;
            _y_speed = y_fixed / time;
            _rot_speed = heading_inc / time;

            publish_odom();

            // need instantaneous speed and instantaneous rotational speed
            prev_time = curr_time;
        }

        void update_encoder(enc & temp) {
            if (temp.prev_tick > temp.curr_tick)
                temp.curr_tick = _max_ticks + (temp.curr_tick - temp.prev_tick);
            else if (temp.prev_tick < temp.curr_tick)
                temp.curr_tick = -1 * (_max_ticks + (temp.prev_tick - temp.curr_tick));
            else
                temp.curr_tick = temp.curr_tick - temp.prev_tick;
        }

        void update_heading(double heading_inc) {
            _heading += heading_inc;
            if (_heading > 180)
                _heading = _heading - 360;
            else if (_heading < -180)
                _heading = _heading + 360;
        }

        void publish_odom() {
            nav_msgs::Odometry msg;
            geometry_msgs::Quaternion quat_msg;
            tf2::Quaternion q;
            q.setRPY(0, 0, _heading);

            msg.pose.pose.position.x = _x;
            msg.pose.pose.position.y = _y;
            msg.pose.pose.position.z = 0;
            msg.pose.pose.orientation = tf2::convert(quat_msg , q);

            msg.twist.twist.linear.x = _x_speed;
            msg.twist.twist.linear.y = _y_speed;
            msg.twist.twist.linear.z = 0;
            msg.twist.twist.angular.x = 0;
            msg.twist.twist.angular.y = 0;
            msg.twist.twist.angular.z = _rot_speed;

            odometry.publish(msg);
        }

        message_filters::Subscriber<std_msgs::Int64> enc_right;
        message_filters::Subscriber<std_msgs::Int64> enc_left;
        ros::Publisher odometry;

    protected:
        enc right, left;
        double _heading;
        double _x, _y;
        double _x_speed, _y_speed;
        double _rot_speed;
        static unsigned int _max_ticks;
        static unsigned int _min_ticks;
        static unsigned int _ticks_per_meter;
        static double _base_width;
        double prev_time, curr_time;
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "odom");

    ros::NodeHandle n;

    double base_width;
    int ticks_per_m, freq, min_ticks, max_ticks;

    n.getParam("base_width", base_width);
    n.getParam("ticks_per_meter", ticks_per_m);
    n.getParam("fixed_frequency", freq);
    n.getParam("min_ticks", min_ticks);
    n.getParam("max_ticks", max_ticks);

    Odom odom(n, max_ticks, min_ticks, base_width, freq, ticks_per_m);

    ros::spin();

}