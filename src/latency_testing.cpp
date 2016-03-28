#include <ros/ros.h>
#include <ros/console.h>

#include <mavros/AslCtrlData.h>
#include <mavros/ActuatorControl.h>

class LatencyTesting
{
public:

	LatencyTesting()
	{
		/* publishers */
		act_sp_pub_ = n_.advertise<mavros::ActuatorControl>("/mavros/actuator_control", 10, true);

		/* subscribers */
		aslctrl_data_sub_ = n_.subscribe("/mavros/aslctrl/data", 10, &SaP::aslctrlDataCallback, this);
	}

	void aslctrlDataCallback(const mavros::AslCtrlData::ConstPtr& msg) {

		mavros::ActuatorControl act_ctrl_msg;
		act_ctrl_msg.timestamp = msg->timestamp;

		act_sp_pub_.publish(act_ctrl_msg);
	}

private:

	ros::NodeHandle n_;
	ros::Publisher 	act_sp_pub_;
	ros::Subscriber aslctrl_data_sub_;

}; // end of class LatencyTesting

int main(int argc, char **argv)
{

  ros::init(argc, argv, "latency_testing");

  LatencyTesting test_obj;

  ros::spin();

  return 0;
}
