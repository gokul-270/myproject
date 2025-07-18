
/*  Author: Saurabh Bansal
    Desc:   O-Drive Hardware control, to be used with ros-control
*/

#ifndef ODRIVE_CONTROL__ODRIVE_HW_INTERFACE_H
#define ODRIVE_CONTROL__ODRIVE_HW_INTERFACE_H

#include <odrive_control/generic_hw_interface.h>
#include <odrive_control/odrive_serial_interface.h>
#include <odrive_control/hardware_serial_interface.h>
#include <odrive_control/joint_homing.h>

/* Motor States */
#define NO_ERROR                (0x00)
#define OVER_HEAT               (0x01)
#define OVER_LOAD               (0x02)
#define UNABLE_TO_REACH         (0x04)
#define TIME_OUT                (0x08)
#define MOTOR_FAULT             (0x10)

#define m_MOTOR_SET_STATE(joint, state)     (state_[joint] |= state)
#define m_MOTOR_RESET_STATE(joint, state)   (state_[joint] &= ~state)

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>

#include <sensor_msgs/JointState.h>


/*  odrive_control
        Description:    Provide controls for each joint from ROS
*/
namespace odrive_control
{
    class ODriveHWInterface : public ros_control_boilerplate::GenericHWInterface
    {
        public:
            /* Constructor for odrive hardware interface */
            ODriveHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

            /* Read the state of robot hardware */
            virtual void read(ros::Duration &elapsed_time);

            /* Write the commands from topics to robot hardware */
            virtual void write(ros::Duration &elapsed_time);

            /* Enforce the limits, before writing to hardware */
            virtual void enforceLimits(ros::Duration &period);
            void  Joint2PositionCallBack(const std_msgs::Float64::ConstPtr& msg);
            void  Joint3PositionCallBack(const std_msgs::Float64::ConstPtr& msg);
            void  Joint4PositionCallBack(const std_msgs::Float64::ConstPtr& msg);
            void  Joint5PositionCallBack(const std_msgs::Float64::ConstPtr& msg);
            void  Joint2PositionDirectValueCallBack(double value);
            void  Joint3PositionDirectValueCallBack(double value);
            void  Joint4PositionDirectValueCallBack(double value);
            void  Joint5PositionDirectValueCallBack(double value);

            bool PublishJointStateValues();

        private:
            bool joint_init_to_home(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp);
            int baud_rate_;
            std::string odrive_serial_number_[2];
            comm_serial* odrive_serial_port_[2];
            bool odirve_configuration_required_;
            std::vector<ODriveSerial*> odrive_serial_interface_;
            std::vector<int> odrive_id_;
            std::vector<int> axis_id_;
            std::vector<int> limit_switch_id_;
            std::vector<float> transmission_factor_;
            std::vector<int> encoder_resolution_;
            std::vector<int> direction_;
            std::vector<float> p_gain_;
            std::vector<float> v_gain_;
            std::vector<float> v_int_gain_;
            std::vector<double> zero_offset_;
            std::vector<float> temp_threshold_;
            std::vector<float> error_threshold_;
            std::vector<float> current_threshold_;
            std::vector<float> max_velocity_;
            std::vector<float> min_velocity_;
            std::vector<std::string> switch_id_;
            std::vector<double> reference_target_;
            std::vector<double> homing_position_;
            std::vector<unsigned int> state_;
            std::vector<double> last_joint_position_command_;
            // std::vector<ros::Publisher> joint_state_pub_;
            std::vector<ros::Subscriber> joint_state_subscriber_;
            std::vector<ros::ServiceClient> joint_homing_switch_in_;
            std::vector<float>homing_pos_init_seq_; //asdded by ribin for jointsequance initilaisationa
            struct ODriveJointState {
                std::string name ;
                float  position ;
                ros::Publisher Publisher ;
            };
            std::vector<ODriveJointState> ODriveJointStatePublisher; // Array of Joinstate Publisher
            ros::Subscriber Joint2Subscriber,Joint3Subscriber,Joint4Subscriber,Joint5Subscriber;
            ros::ServiceServer joint_homing_srv_;
            ros::Publisher joint_pub ;
            int No_Of_Joint_initialised_to_home_;
            // class ODriveHWInterface
            //sensor_msgs::JointState joint_state;
            sensor_msgs::JointState joint_state; // Variable used by ODriveJointStatePublisher 
            bool joint_init_to_idle(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp);
            ros::ServiceServer joint_idle_srv_;
            ros::ServiceServer pause_robot_srv_;
            bool pause_joints(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp);
            void JointPositionCallBack(const std_msgs::Float64::ConstPtr &msg);
            //bool SetPointRequest(odrive_control::joint_homing::Request &req, odrive_control::joint_homing::Response &resp);

    };
}   // namespace odrive_control

#endif  // #ifndef ODRIVE_CONTROL__ODRIVE_HW_INTERFACE_H
