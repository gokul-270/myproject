
/*
   JOINT MOVEMENT
 */

#include <odrive_control/joint_homing.h>
//#include "cotton_detection/capture_cotton_srv.h"
#include <yanthra_move/arm_status.h>
#include <sensor_msgs/JointState.h> //TODO:MR


//#define JOINT_MOVE(id, pos)		move_joint_in_sync(joint_pub[id], pos, pos)
#define MOVE_JOINT(pos)		{\
	val.data = pos;\
	joint_pub.publish(val);\
	ros::spinOnce();\
}

#define WAIT	true
#define NO_WAIT	false

/* Motor Fail Conditions */
#define MOTOR_TEMP_MAX		(70)
#define MOTOR_LOAD_MAX		(0.50)
#define MOTOR_POS_ERROR_MAX	(0.2)
#define MOTOR_TIME_OUT		(3.0)

#define	NO_ERROR		(0x00)
#define	OVER_HEAT		(0x01)
#define	OVER_LOAD		(0x02)
#define	UNABLE_TO_REACH	(0x04)
#define	TIME_OUT		(0x08)
#define MOTOR_FAIL		(0x10)

/* Trajectory Types */
#define TRAJ_NONE		(0x00)
#define TRAJ_CIRCLE		(0x01)
#define TRAJ_HORIZONTAL	(0x02)
#define TRAJ_VERTICLE	(0x04)

/* Trajectory Speed */
#define SPD_LOW			(1.0)
#define SPD_MEDIUM		(5.0)
#define SPD_HIGH		(8.0)

class joint_move
{
	ros::Publisher joint_pub;
	ros::Subscriber joint_status;
	void joint_state_cb(const dynamixel_msgs::JointState::ConstPtr& msg) //TODO:MR
	{
		joint_state = *msg;
		//if(joint_state.load == 0xff) //TODO:dynamixel may have the parameter called load bu sensor_msgs might not have
                      //error_code |= MOTOR_FAIL;
	}
	public:
	static ros::Publisher joint_pub_trajectory;
	static ros::ServiceClient joint_homing_service;
	static ros::ServiceClient joint_idle_service;
	static ros::ServiceClient cotton_detection_ml_service;
	dynamixel_msgs::JointState joint_state;//TODO:MR
	unsigned int error_code;
	void move_joint(double, bool);
	static void move_joint_trajectory(int, double, double, double);
    int joint_id ;
	joint_move(ros::NodeHandle, std::string);
	joint_move(ros::NodeHandle, std::string, int OdriveJointID);
};

ros::Publisher joint_move::joint_pub_trajectory;
ros::ServiceClient joint_move::joint_homing_service;
ros::ServiceClient joint_move::joint_idle_service;
ros::ServiceClient joint_move::cotton_detection_ml_service;
ros::ServiceServer arm_status_srv_;

// This routine is called when a direct call to the motor instead of the publish subscribe
// This is added to avoid the ros control_loop since the thread seems to be in a DEADLOCK state
joint_move::joint_move(ros::NodeHandle nh, std::string name, int ODriveJointID)
{
	joint_pub = nh.advertise<std_msgs::Float64>(name + "command", 2);
 	//joint_status = nh.subscribe(name + "state", 10, &joint_move::joint_state_cb, this);//TODO:MR

    joint_id = ODriveJointID ;
	error_code = NO_ERROR;
}



joint_move::joint_move(ros::NodeHandle nh, std::string name)
{
	joint_pub = nh.advertise<std_msgs::Float64>(name + "command", 2);
 	//joint_status = nh.subscribe(name + "state", 10, &joint_move::joint_state_cb, this);

	error_code = NO_ERROR;
}

// This code was used for Dynamixel with actual feedback of the motor status after the action
#if MOVE_EN	== true
void joint_move::move_joint(double value, bool wait)
{
	std_msgs::Float64 val;

	ROS_INFO("Motor ID: %d, Target pos: %f", joint_state.motor_ids[0], value);

	if(error_code != NO_ERROR)
	{
		ROS_ERROR("Can't move the joint, error: %u", error_code);
		return;
	}
	if(joint_state.motor_temps[0] <= MOTOR_TEMP_MAX)
	{
		double begin = ros::Time::now().toSec();
		double time_diff = 0.0;
		MOVE_JOINT(value);
		ros::Duration(0.2).sleep();
		ROS_INFO("Is moving: %d", joint_state.is_moving);

		/* Wait while joint move in progress */
		while(joint_state.is_moving == true)
		{
			/* Check if motor is over-loaded */
			if(joint_state.load >= MOTOR_LOAD_MAX)
			{
				ROS_ERROR("Over Load: %f", joint_state.load);
				error_code |= OVER_LOAD;
				//MOVE_JOINT(joint_state.current_pos);
				break;
			}
			/* Break loop, if wait is not required */
			if(wait == NO_WAIT)
			{
				break;
			}
			/* Break loop, if it is timeout */
			time_diff = ros::Time::now().toSec() - begin;
			if(time_diff >= MOTOR_TIME_OUT)
			{
				ROS_ERROR("Time Out: %lf", time_diff);
				error_code |= TIME_OUT;
				break;
			}
			ros::spinOnce();
		}
		ROS_WARN("Time taken: %lf, Error: %lf", time_diff, joint_state.error);
		if((wait != NO_WAIT) && (std::abs(joint_state.error) >= MOTOR_POS_ERROR_MAX))
		{
			ROS_ERROR("Unable to reach: %f", joint_state.error);
			error_code |= UNABLE_TO_REACH;
		}
	}
	else
	{
		ROS_ERROR("Over Heat: %d", joint_state.motor_temps[0]);
		error_code |= OVER_HEAT;
	}
}

// This code was used by non DYNAMIXEL ie ODRIVEMotors currently, where motor status is NOT ready correctly before
#elif MOVE_EN == false
/*
void joint_move::move_joint(double value, bool wait)
{
	unsigned int error_code;
	std_msgs::Float64 val;
	error_code = NO_ERROR;
	//ROS_INFO("Joint_Value_Sent_To_Motor= %f",value);
	MOVE_JOINT(value);
        // 24 Jan 2023 , Manohar Sambandam commented the following line. Don't see the need for this sleep 
        //ros::Duration(0.2).sleep();
	ROS_WARN("joint_move.h::Moving the joint_TESTING");
}
*/

// This code is added to directly call the odrive_hw_interface::write() routine
// This is written to avoid the publish and subscribe loop of ROS control and eliminate the queue management
extern  boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface ;
void joint_move::move_joint(double value, bool wait)
{
 //boost::shared_ptr<std_msgs::Float64> msg;
 //std_msgs::Float64::Ptr msg;
 //std_msgs::Float64 msg;
 ROS_INFO("joint_move::move_joint >> JointID : %d Joint Value : %f",(int) joint_id, (float) value);
 //msg.data = value ;

 //   boost::shared_ptr<odrive_control::ODriveHWInterface> odrive_hw_interface
 //       (new odrive_control::ODriveHWInterface(n));
 // std_msgs::Float64  msg ;//( new std_msgs::Float64::Ptr );
 //  msg.data = value ;

 //236 void  ODriveHWInterface::Joint3PositionCallBack(const std_msgs::Float64::ConstPtr& msg)
 // This will update the variables used by odrive_serial_interface routines 
 if ( joint_id == 0)  odrive_hw_interface->Joint3PositionDirectValueCallBack(  value );
 if ( joint_id == 1)  odrive_hw_interface->Joint4PositionDirectValueCallBack(  value );
 if ( joint_id == 2)  odrive_hw_interface->Joint5PositionDirectValueCallBack(  value );
 if ( joint_id == 3)  odrive_hw_interface->Joint2PositionDirectValueCallBack(  value );
 /*
 if ( joint_id == 0)  odrive_hw_interface->Joint3PositionCallBack( (const std_msgs::Float64::ConstPtr&) &msg );
 if ( joint_id == 1)  odrive_hw_interface->Joint4PositionCallBack( (const std_msgs::Float64::ConstPtr&) &msg );
 if ( joint_id == 2)  odrive_hw_interface->Joint5PositionCallBack( (const std_msgs::Float64::ConstPtr&) &msg );
 if ( joint_id == 3)  odrive_hw_interface->Joint2PositionCallBack( (const std_msgs::Float64::ConstPtr&) &msg );
 */
 // TODO this write  Routine need not be called for every write 
 // but can be called once in a loopa

 // TODO Need a seperate function where we pass the motorid/jointid and the
 // routine directly calls the function to move the motor to the desired location
 // This will be a direct call whenever required for the specified motor.

 //Dummy no is passed since it is not used 
 ros::Duration elapsed_time(0);
 // TODO : Removed from here and called from the odrive_hw_interface->read() in  
 odrive_hw_interface->write(elapsed_time );

}

#endif
void joint_move::move_joint_trajectory(int speed,
		double target_pos_3,
		double target_pos_4,
		double target_pos_5)
{
	trajectory_msgs::JointTrajectory traj;
	trajectory_msgs::JointTrajectoryPoint point;

	traj.header.stamp = ros::Time::now();
	traj.header.frame_id = "/base_link";

	traj.joint_names.resize(3);
	traj.joint_names[0] = "joint3";
	traj.joint_names[1] = "joint4";
	traj.joint_names[2] = "joint5";

	point.positions.resize(3);
	point.positions[0] = target_pos_3;
	point.positions[1] = target_pos_4;
	point.positions[2] = target_pos_5;

	point.velocities.resize(3);
	point.velocities[0] = SPD_LOW;
	point.velocities[1] = SPD_LOW;
	point.velocities[2] = speed;
	point.time_from_start = ros::Duration(0.1);

	traj.points.resize(1);
	traj.points[0] = point;

	joint_move::joint_pub_trajectory.publish(traj);
	ros::spinOnce();

	ROS_INFO("move_joint_trajectory");
}
