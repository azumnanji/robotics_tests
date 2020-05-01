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

class Position{
    public:
        
        Position(ros::NodeHandle &nh, unsigned int max_ticks, unsigned int min_ticks, double base_width, 
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
            right.curr_tick = right_tick.data;
            left.curr_tick = left_tick.data;

            for (int i = 0; i < 2; ++i) {
                enc temp = i == 0 ? right:left;
                update_encoder(temp);
            }

            double dist_right = static_cast<double>(right.curr_tick) / _ticks_per_meter;
            double dist_left = static_cast<double>(left.curr_tick) / _ticks_per_meter;

            double heading_inc = (dist_right - dist_left) / _base_width;
            double x_inc = cos(heading_inc) * (dist_left + dist_right) / 2;
            double y_inc = sin(heading_inc) * (dist_left + dist_right) / 2;

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

    protected:
        enc right, left;
        double _heading;
        double _x;
        double _y;
        static unsigned int _max_ticks;
        static unsigned int _min_ticks;
        static unsigned int _ticks_per_meter;
        static double _base_width;
        message_filters::Subscriber<std_msgs::Int64> enc_right;
        message_filters::Subscriber<std_msgs::Int64> enc_left;
        ros::Publisher odometry;
        ros::Time prev_time, curr_time;
};


int main(int argc, char** argv) {
    return 0;
}