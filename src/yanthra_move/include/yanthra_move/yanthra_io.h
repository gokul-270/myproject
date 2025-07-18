
/*
   YANTHRA INPUT / OUTPUT
 */
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <pigpiod_if2.h>



#define OPEN	true
#define CLOSE	false
#define START	true
#define STOP	false
#define ON	true
#define OFF	false
#define CLOCK_WISE	true
#define ANTI_CLOCK_WISE	false
int pi;
unsigned gpio_pin_number;
unsigned pulsewidth;

class pi_gpio_control
{
	public:
	unsigned int gpio_pin_number;
	unsigned int gpio_second_pin_number;
	void relay_control(bool status){
		ROS_ERROR("relay control"); // end_effector_clockwise rotation

        	if(status == true){
          		if(gpio_write(pi,gpio_pin_number,0)!=0)
				ROS_ERROR("Failed to Switch ON the PIN"); // end_effector_clockwise rotation
        	}
        	if(status == false){

          		if(gpio_write(pi,gpio_pin_number,1)!=0); // end_effector_clockwise rotation
				ROS_ERROR("Failed to Switch OFF the PIN"); // end_effector_clockwise rotation
        	}
	}
	
	void servo_control(unsigned pwm){

		    	if(set_servo_pulsewidth(pi, gpio_pin_number, pulsewidth)!=0){
				
				ROS_ERROR("Failed to Switch PWM on  the PIN"); // end_effector_clockwise rotation
			};
	}
}; 
		


class d_out
{
	ros::Publisher pub;
	ros::Subscriber sub;
	std::string output_name;

	void callback(const std_msgs::StringConstPtr &msg)
	{
		//	ROS_INFO("%s status: %s", output_name.c_str(), msg->data.c_str());
	}
	public:
	void command(bool cmd)
	{
		std_msgs::Bool val;
		val.data = cmd;
		pub.publish(val);
		ros::spinOnce();
	}
	d_out(ros::NodeHandle nh, std::string name):
		output_name(name)
	{
		pub = nh.advertise<std_msgs::Bool>(output_name+"/command", 2);
		//sub = nh.subscribe<std_msgs::String>(output_name+"/status", 1, &d_out::callback, this);
	}
};

class a_out
{
	ros::Publisher pub;
	std::string output_name;
	public:
	void command(float cmd)
	{
		std_msgs::Float32 val;
		val.data = cmd;
		pub.publish(val);
		ros::spinOnce();
	}
	a_out(ros::NodeHandle nh, std::string name):
		output_name(name)
	{
		pub = nh.advertise<std_msgs::Float32>(output_name+"/command", 2);
	}
};

class d_in
{
	ros::Subscriber sub;
	std::string output_name;
	bool status;

	void callback(const std_msgs::Bool::ConstPtr &msg)
	{
		status = msg->data;
		//	ROS_INFO("%s status: %d", output_name.c_str(), status);
	}
	public:
	bool state(void)
	{
		bool return_status = false;
		ros::spinOnce();
		if(status == true)
		{
			return_status = status;
			status = false;
		}
		return return_status;
	}
	d_in(ros::NodeHandle nh, std::string name):
		output_name(name)
	{
		sub = nh.subscribe<std_msgs::Bool>(output_name+"/state", 1, &d_in::callback, this);
		status = false;
	}
};

class servo_out
{
	ros::Publisher pub_;
	std::string output_name_;
	double pos_;
	double min_;
	double max_;
	public:
	void init(void)
	{
		pos_ = 0.000;

		ROS_WARN("%s: Initialisation", output_name_.c_str());
		std_msgs::Float32 value;
		value.data = min_;
		pub_.publish(value);
		ros::spinOnce();
	}
	bool move(double val)
	{
		if(val == 0)
			return false;
		if(pos_ >= max_)
			return false;

		ROS_WARN("%s: Present Position: %lf", output_name_.c_str(), pos_);
		pos_ += val;
		std_msgs::Float32 value;
		value.data = (val*1000);
		pub_.publish(value);
		ros::spinOnce();
		return true;
	}


	servo_out(ros::NodeHandle nh, std::string name,
			double min,
			double max)
		: output_name_(name),
		min_(min),
		max_(max)
	{
		pub_ = nh.advertise<std_msgs::Float32>(output_name_, 2);
		pos_ = 0.0;
	}
};
