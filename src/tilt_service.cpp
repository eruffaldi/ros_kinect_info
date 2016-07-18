#include <libfreenect.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include <string>
#include <sstream>

int user_device_number;
ros::Publisher imu_publisher;

bool sample_imu(std_srvs::Empty::Request& request,std_srvs::Empty::Request&  response)
{
    freenect_context *f_ctx = 0;
    freenect_device *f_dev = 0;

    sensor_msgs::Imu imu_msg_;
    memset(&imu_msg_,0,sizeof(imu_msg_));
    //response = imu_msg_;

    if (freenect_init (&f_ctx, NULL) < 0) {
        ROS_INFO("freenect_init() failed\n");
        return false;
    }
    freenect_select_subdevices(f_ctx,(freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA | FREENECT_DEVICE_AUDIO));
    freenect_set_log_level (f_ctx, FREENECT_LOG_INFO);
    // Scan for kinect devices
    int nr_devices = freenect_num_devices (f_ctx);
    ROS_INFO("Number of devices found: %d\n", nr_devices);
    // Get the device number
    if (nr_devices < 1) return false;
    // Calculate the kinect node base name
    // Open the base portion of the Kinect
    if (freenect_open_device (f_ctx, &f_dev, user_device_number) < 0) {
        ROS_INFO("Could not open device\n");
        return false;
    }

    double aX = 0.0, aY = 0.0, aZ = 0.0;

    freenect_raw_tilt_state *state;
    freenect_update_tilt_state (f_dev);
    state = freenect_get_tilt_state (f_dev);
    freenect_get_mks_accel (state, &aX, &aY, &aZ);

    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.linear_acceleration.x = aX;
    imu_msg_.linear_acceleration.y = aY;
    imu_msg_.linear_acceleration.z = aZ;
    imu_msg_.linear_acceleration_covariance[0] = imu_msg_.linear_acceleration_covariance[4]
        = imu_msg_.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
    imu_msg_.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
    imu_msg_.orientation_covariance[0] = -1; // indicates orientation not provided
    imu_publisher.publish(imu_msg_);

    //response = imu_msg_;
    freenect_close_device(f_dev);
    freenect_shutdown(f_ctx);
    return true;
}


int main (int argc, char **argv) {
    // Initalize the ROS node
    ros::init(argc, argv, "kinect_info_node");
    ros::NodeHandle n;

    freenect_context *f_ctx = 0;
    if (argc > 1) user_device_number = atoi (argv[1]);
    if (freenect_init (&f_ctx, NULL) < 0) {
        ROS_INFO("freenect_init() failed\n");
        return 1;
    }
    freenect_set_log_level (f_ctx, FREENECT_LOG_INFO);
    // Scan for kinect devices
    int nr_devices = freenect_num_devices (f_ctx);
    if (nr_devices < 1) return 1;
    freenect_shutdown(f_ctx);

    std::stringstream kinect_node_base;
    kinect_node_base << "/kinect_info_node/" << user_device_number << "/";
    imu_publisher = n.advertise<sensor_msgs::Imu>(kinect_node_base.str() + "imu", 1000, true);
    ros::ServiceServer service = n.advertiseService<std_srvs::Empty::Request,std_srvs::Empty::Request>("sample_imu", sample_imu); // 
    ros::spin();
    return 0;
}