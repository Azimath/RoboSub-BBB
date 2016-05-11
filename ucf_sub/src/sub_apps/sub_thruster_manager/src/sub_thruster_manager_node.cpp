#include "blue_robotics_t200/t200_thruster.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/SelfTest.h>

#define THRUSTER_UNDERVOLT 11.0
#define THRUSTER_OVERVOLT 20.0
#define THRUSTER_OVERCURRENT 25.0
#define THRUSTER_OVERTEMP 40.0

inline const std::string BoolToString(const bool b); //http://stackoverflow.com/a/29798

class ThrusterManager {
    ros::NodeHandle nh_;
    ros::Subscriber command_subscriber;
    ros::Publisher diagnostics_output;

    ros::AsyncSpinner spinner;

    T200Thruster thrusterr;
    T200Thruster thrusterl;

public:
    ThrusterManager() : thrusterl(2, 0x2D), thrusterr(2, 0x2E), spinner(1)
    {
        command_subscriber = nh_.subscribe("/thrustercommands", 1000, &ThrusterManager::thrusterCb, this);

        diagnostics_output = nh_.advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics", 1000);
    }

    void init()
    {
        if(spinner.canStart())
            spinner.start();
        else
            return;

        ros::Rate rate(4);
        while(ros::ok()) {
            //Publish diagnostic data here
            diagnostic_msgs::DiagnosticStatus status;
            status.name = "Thrusters";
            status.hardware_id = "Thrusters";

            thrusterr.updateStatus();
            thrusterl.updateStatus();

            bool thrustersAlive = thrusterr.isAlive() && thrusterl.isAlive();
            bool thrustersPowerOK = (THRUSTER_OVERVOLT > thrusterr.getVoltage() > THRUSTER_UNDERVOLT) &&
                                    (THRUSTER_OVERVOLT > thrusterl.getVoltage() > THRUSTER_UNDERVOLT);
            bool thrustersCurrentOK = (THRUSTER_OVERCURRENT > thrusterr.getCurrent()) &&
                                     (THRUSTER_OVERCURRENT > thrusterl.getCurrent());
            bool thrustersTemperatureOK = (THRUSTER_OVERTEMP > thrusterr.getTemperature()) &&
                                         (THRUSTER_OVERTEMP > thrusterl.getTemperature());
            if (thrustersAlive && thrustersPowerOK && thrustersCurrentOK && thrustersTemperatureOK)
                status.level = status.OK;
            else
                status.level = status.ERROR;

            PushDiagData(status, thrusterr, "Thruster R");
            PushDiagData(status, thrusterl, "Thruster L");

            rate.sleep();
        }
        spinner.stop();
    }

    void PushDiagData(diagnostic_msgs::DiagnosticStatus & statusmsg, T200Thruster thruster, std::string thrusterName)
    {
        diagnostic_msgs::KeyValue thrusterValue;

        thrusterValue.key = thrusterName + " Alive";
        thrusterValue.value = BoolToString(thruster.isAlive());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Voltage";
        thrusterValue.value = std::to_string(thruster.getVoltage());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Current";
        thrusterValue.value = std::to_string(thruster.getCurrent());
        statusmsg.values.push_back(thrusterValue);

        thrusterValue.key = thrusterName + " Temperature";
        thrusterValue.value = std::to_string(thruster.getTemperature());
        statusmsg.values.push_back(thrusterValue);
    }

    void thrusterCb(const geometry_msgs::Twist &msg)
    {
	//copy data out of the message so we can modify as necessary
        float angularX, angularY, angularZ, linearX, linearY, linearZ;

        angularX = msg.angular.x;
        angularY = msg.angular.y;
        angularZ = msg.angular.z;

        linearX = msg.linear.x;
        linearY = msg.linear.y;
        linearZ = msg.linear.z;

        //normalize linear/angular pairs that depend on the same thruster pairs
        /*
          We need this because trying to apply a tranlation force of +/-1 and a torque of +/-1 isnt possible
          with our thruster setup, so we normalize in order to approximate whatever mix the controller wants
        */

        if(magnitude(linearZ, angularY) > 1.0)
        {
            linearZ = linearZ / magnitude(linearZ, angularY);
            angularY = angularY /  magnitude(linearZ, angularY);
        }

        if(magnitude(linearX, angularZ) > 1.0)
        {
            linearX = linearX / magnitude(linearX, angularZ);
            angularZ = angularZ /  magnitude(linearX, angularZ);
        }

        if(magnitude(linearY, angularX) > 1.0)
        {
            linearY = linearY / magnitude(linearY, angularX);
            angularX = angularX /  magnitude(linearY, angularX);
        }

        //mix the twist message, limits for extra safety
        float tFrontUp, tRearUp, tLeftForward, tRightForward, tTopStrafe, tBottomStrafe;

        tFrontUp = std::max(-1.0f, std::min(1.0f, linearZ + angularY));
        tRearUp = std::max(-1.0f, std::min(1.0f, linearZ  - angularY));

        tLeftForward = std::max(-1.0f, std::min(1.0f, linearX - angularZ));
        tRightForward = std::max(-1.0f, std::min(1.0f, linearX + angularZ));

        tTopStrafe = std::max(-1.0f, std::min(1.0f, linearY - angularX));
        tBottomStrafe = std::max(-1.0f, std::min(1.0f, linearY + angularX));

        //command the thrusters to the desired velocity
        thrusterr.setVelocityRatio(fabs(tLeftForward), tLeftForward >0.0f ? T200ThrusterDirections::Forward : T200ThrusterDirections::Reverse);
        thrusterl.setVelocityRatio(fabs(tRightForward), tRightForward > 0.0f ? T200ThrusterDirections::Forward : T200ThrusterDirections::Reverse);
        
        //ROS_INFO("out: %f\n", tLeftForward);
    }
    float magnitude(float x, float y) //return the magnitude of a 2d vector
    {
        return sqrt(x*x + y*y);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thrust_manager");
    ThrusterManager tc;
    tc.init();

    return 0;
}

inline const std::string BoolToString(const bool b)
{
  return b ? "true" : "false";
}
