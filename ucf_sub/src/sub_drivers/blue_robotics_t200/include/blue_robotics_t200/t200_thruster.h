#ifndef T200_Thruster_H
#include "i2c_interface.h"

namespace T200ThrusterDirections
{
    enum T200ThrusterDirection
    {
        Forward,
        Reverse
    };
}
typedef T200ThrusterDirections::T200ThrusterDirection T200ThrusterDirection;

namespace T200ThrusterStatusIndices
{
    enum T200ThrusterStatusIndex
    {
        Pulse_Count_H,
        Pulse_Count_L,
        Voltage_H,
        Voltage_L,
        Temperature_H,
        Temperature_L,
        Current_H,
        Current_L,
        Identifier
    };
} 
typedef T200ThrusterStatusIndices::T200ThrusterStatusIndex T200ThrusterStatusIndex;

////////////////////////////////////////////////////////////////////////////////
/// \brief Class providing functionality for using the Blue Robotics Blue ESC T200
/// 
/// This class provides for control and monitoring of Blue Robotics T200 Thrusters
/// using the Blue ESC in I2C mode.  It supports velocity control as a ratio of 0.0
/// to 1.0 in forward and reverse directions.
///
////////////////////////////////////////////////////////////////////////////////
class T200Thruster
{
public:
////////////////////////////////////////////////////////////////////////////////
/// \brief Constructs a new T200Thruster interface.
/// 
/// \param bus_number   The I2C bus to use.
/// \param address      The 7-bit slave address of the thruster.
////////////////////////////////////////////////////////////////////////////////
T200Thruster(int bus_number, unsigned char address);

////////////////////////////////////////////////////////////////////////////////
// \brief Default T200Thruster destructor.
//
////////////////////////////////////////////////////////////////////////////////
~T200Thruster();

////////////////////////////////////////////////////////////////////////////////
/// \brief Function for setting the thruster's velocity.
///
/// Sets the thruster power as a ratio of the maximum supported velocity by the 
/// thruster and according to the direction supplied.
///
/// \param velocity_scale    The ratio between 0.0 and 1.0 for thruster power.
/// \param direction         The direction of the thruster.
////////////////////////////////////////////////////////////////////////////////
void setVelocityRatio(double  velocity_scale, T200ThrusterDirection direction);

////////////////////////////////////////////////////////////////////////////////
/// \brief Function for setting the thruster's velocity.
///
/// Overload to set thruster direction and power as a ratio of maximum supported velocity
///
/// \param velocity_ratio    The ratio between -1.0 and 1.0 for thruster power.
////////////////////////////////////////////////////////////////////////////////
void setVelocityRatio(double  velocity_ratio);

////////////////////////////////////////////////////////////////////////////////
/// \brief linearizes thruster output and almost eliminates deadzone
///
/// Function that takes desired thrust output as a fraction from 1.0 to -1.0 and
/// applies a linear function based on data from the manufacturer so that deadband is 
/// reduced to a range from 0.01 to -0.01 and maximum output is the same in both 
/// directions
///
/// \param velocity_desired desired thruster output
////////////////////////////////////////////////////////////////////////////////
double linearizeOutput(double velocity_desired);

////////////////////////////////////////////////////////////////////////////////
/// \brief Forces the T200Thruster to update its most recent status data.
///
/// Initiates communication to the thruster and retrieves all status bytes
/// supported by the Blue ESC for use in status exposure functions.
////////////////////////////////////////////////////////////////////////////////
void updateStatus();

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the number of commutation pulses since the last read.
///
/// For use in calculation of RPM according to Blue Robotics documentation.  
/// Suggested use is rpm = pulse_count/dt*60/motor_pole_count
////////////////////////////////////////////////////////////////////////////////
int getPulseCount();

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the current voltage supplied to the thruster.
///
////////////////////////////////////////////////////////////////////////////////
double getVoltage();

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the temperature of the ESC in degrees Celsius.
///
////////////////////////////////////////////////////////////////////////////////
double getTemperature();

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the current supplied to the thruster.
///
////////////////////////////////////////////////////////////////////////////////
double getCurrent();

////////////////////////////////////////////////////////////////////////////////
/// \brief Checks that the ESC is communicating as expected.
///
////////////////////////////////////////////////////////////////////////////////
bool isAlive();

private:

static const int STATUS_DATA_BYTES = 9;
static const int MAX_VELOCITY_VALUE = 32767;
static constexpr double VOLTAGE_SCALE_FACTOR = 0.0004921;
static constexpr double CURRENT_SCALE_FACTOR = 0.001122;
static const int CURRENT_SCALE_OFFSET = 32767;
static const int THERMISTOR_NOMINAL = 10000;
static const int TEMPERATURE_NOMINAL = 25;
static const int B_COEFFICIENT = 3900;
static const int SERIES_RESISTOR = 3300;
static const int MOTOR_POLE_COUNT = 14;
static const unsigned char THROTTLE_REGISTER_START = 0x00;
static const unsigned char DATA_REGISTER_START = 0x02;
static const unsigned char GOOD_IDENTIFIER = 0xAB;

ByteBuffer status_data_;
I2C_Interface i2c_interface_;

void setVelocity(int velocity_command);
int getRawPulseCountMeasurement();
int getRawVoltageMeasurement();
int getRawTemperatureMeasurement();
int getRawCurrentMeasurement();
unsigned char getIdentifier();
};

#endif
