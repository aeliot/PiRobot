#ifndef __ROBOT_SERIAL__
#define __ROBOT_SERIAL__

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace boost;
using namespace boost::asio;

#define DEFAULT_BAUD 57600

//Opcodes
#define OPCODE_START           ((unsigned char)128)
#define OPCODE_BAUD            ((unsigned char)129)
#define OPCODE_SAFE            ((unsigned char)131)
#define OPCODE_FULL            ((unsigned char)132)
#define OPCODE_DEMO            ((unsigned char)136)
#define OPCODE_COVER           ((unsigned char)135)
#define OPCODE_COVER_DOCK      ((unsigned char)143)
#define OPCODE_SPOT            ((unsigned char)134)
#define OPCODE_DRIVE           ((unsigned char)137)
#define OPCODE_DRIVE_DIRECT    ((unsigned char)145)
#define OPCODE_LEDS            ((unsigned char)139)
#define OPCODE_DIGITAL_OUT     ((unsigned char)147)
#define OPCODE_LOW_SIDE_PWM    ((unsigned char)144)
#define OPCODE_LOW_SIDE        ((unsigned char)138)
#define OPCODE_SEND_IR         ((unsigned char)151)
#define OPCODE_SONG            ((unsigned char)140)
#define OPCODE_SONG_PLAY       ((unsigned char)141)
#define OPCODE_SENSORS         ((unsigned char)142)
#define OPCODE_QUERY_LIST      ((unsigned char)149)
#define OPCODE_STREAM          ((unsigned char)148)
#define OPCODE_STREAM_PAUSE    ((unsigned char)150)
#define OPCODE_SCRIPT          ((unsigned char)152)
#define OPCODE_SCRIPT_PLAY     ((unsigned char)153)
#define OPCODE_SCRIPT_SHOW     ((unsigned char)154)
#define OPCODE_WAIT_TIME       ((unsigned char)155)
#define OPCODE_WAIT_DISTANCE   ((unsigned char)156)
#define OPCODE_WAIT_ANGLE      ((unsigned char)157)
#define OPCODE_WAIT_EVENT      ((unsigned char)158)

#define BAUD_300        (char)0
#define BAUD_600        (char)1
#define BAUD_1200       (char)2
#define BAUD_2400       (char)3
#define BAUD_4800       (char)4
#define BAUD_9600       (char)5
#define BAUD_14400      (char)6
#define BAUD_19200      (char)7
#define BAUD_28800      (char)8
#define BAUD_38400      (char)9
#define BAUD_57600      (char)10
#define BAUD_115200     (char)11

#define DEMO_ABORT          (char)255
#define DEMO_COVER          (char)0
#define DEMO_COVER_DOCK     (char)1
#define DEMO_SPOT_COVER     (char)2
#define DEMO_MOUSE          (char)3
#define DEMO_FIGURE_EIGHT   (char)4
#define DEMO_WIMP           (char)5
#define DEMO_HOME           (char)6
#define DEMO_TAG            (char)7
#define DEMO_PACHELBEL      (char)8
#define DEMO_BANJO          (char)9

#define DRIVE_STRAIGHT      0x8000
#define DRIVE_TURN_RIGHT    0xFFFF
#define DRIVE_TURN_LEFT     0x0001

#define LED_ADVANCE		0x08
#define LED_PLAY		0x02

//Scripting events
#define EVENT_WHEEL_DROP             ((unsigned char)1)
#define EVENT_WHEEL_DROP_FRONT       ((unsigned char)2)
#define EVENT_WHEEL_DROP_LEFT        ((unsigned char)3)
#define EVENT_WHEEL_DROP_RIGHT       ((unsigned char)4)
#define EVENT_BUMP                   ((unsigned char)5)
#define EVENT_BUMP_LEFT              ((unsigned char)6)
#define EVENT_BUMP_RIGHT             ((unsigned char)7)
#define EVENT_VIRTUAL_WALL           ((unsigned char)8)
#define EVENT_WALL                   ((unsigned char)9)
#define EVENT_CLIFF                  ((unsigned char)10)
#define EVENT_CLIFF_LEFT             ((unsigned char)11)
#define EVENT_CLIFF_LEFT_FRONT       ((unsigned char)12)
#define EVENT_CLIFF_RIGHT_FRONT      ((unsigned char)13)
#define EVENT_CLIFF_RIGHT            ((unsigned char)14)
#define EVENT_HOME_BASE              ((unsigned char)15)
#define EVENT_BUTTON_ADVANCE         ((unsigned char)16)
#define EVENT_BUTTON_PLAY            ((unsigned char)17)
#define EVENT_INPUT_0                ((unsigned char)18)
#define EVENT_INPUT_1                ((unsigned char)19)
#define EVENT_INPUT_2                ((unsigned char)20)
#define EVENT_INPUT_3                ((unsigned char)21)
#define EVENT_PASSIVE                ((unsigned char)22)

//Sensor opcodes
#define SENSOR_ID_BUMP_WHEELDROP           ((unsigned char)7)
#define SENSOR_ID_WALL                     ((unsigned char)8)
#define SENSOR_ID_CLIFF_LEFT               ((unsigned char)9)
#define SENSOR_ID_CLIFF_LEFT_FRONT         ((unsigned char)10)
#define SENSOR_ID_CLIFF_RIGHT_FRONT        ((unsigned char)11)
#define SENSOR_ID_CLIFF_RIGHT              ((unsigned char)12)
#define SENSOR_ID_VIRTUAL_WALL             ((unsigned char)13)
#define SENSOR_ID_OVERCURRENTS             ((unsigned char)14)
#define SENSOR_ID_INFRARED                 ((unsigned char)17)
#define SENSOR_ID_BUTTONS                  ((unsigned char)18)
#define SENSOR_ID_DISTANCE                 ((unsigned char)19)
#define SENSOR_ID_ANGLE                    ((unsigned char)20)
#define SENSOR_ID_CHARGING_STATE           ((unsigned char)21)
#define SENSOR_ID_VOLTAGE                  ((unsigned char)22)
#define SENSOR_ID_CURRENT                  ((unsigned char)23)
#define SENSOR_ID_BATTERY_TEMP             ((unsigned char)24)
#define SENSOR_ID_BATTERY_CHARGE           ((unsigned char)25)
#define SENSOR_ID_BATTERY_CAPACITY         ((unsigned char)26)
#define SENSOR_ID_SIGNAL_WALL              ((unsigned char)27)
#define SENSOR_ID_SIGNAL_CLIFF_LEFT        ((unsigned char)28)
#define SENSOR_ID_SIGNAL_CLIFF_LEFT_FRONT  ((unsigned char)29)
#define SENSOR_ID_SIGNAL_CLIFF_RIGHT_FRONT ((unsigned char)30)
#define SENSOR_ID_SIGNAL_CLIFF_RIGHT       ((unsigned char)31)
#define SENSOR_ID_BAY_DIGITAL_INPUTS       ((unsigned char)32)
#define SENSOR_ID_BAY_ANALOG_SIGNAL        ((unsigned char)33)
#define SENSOR_ID_CHARGIN_AVAILABLE        ((unsigned char)34)
#define SENSOR_ID_OI_MODE                  ((unsigned char)35)
#define SENSOR_ID_SONG_ID                  ((unsigned char)36)
#define SENSOR_ID_SONG_PLAYING             ((unsigned char)37)
#define SENSOR_ID_NUMBER_STREAM_PACKETS    ((unsigned char)38)
#define SENSOR_ID_REQUESTED_VELOCITY       ((unsigned char)39)
#define SENSOR_ID_REQUESTED_RADIUS         ((unsigned char)40)
#define SENSOR_ID_REQUESTED_VELOCITY_RIGHT ((unsigned char)41)
#define SENSOR_ID_REQUESTED_VELOCITY_LEFT  ((unsigned char)42)


class Note {
	Note(unsigned char num, unsigned char len);
	unsigned char number;
	unsigned char length;
};

class RobotSerial {
public:
	RobotSerial();
    ~RobotSerial();
	//System Controls
	void start();
	void full();
	void baud(char bps);
	void safe();

	//Demos
	void demo(unsigned char mode);
	void cover();
	void coverDock();
	void spot();

	//Actuator
	void drive(int vel, int rad);
	void drive(unsigned char velH, unsigned char velL, unsigned char radH, unsigned char radL);
	void driveDirect(int left, int right);
	void driveDirect(unsigned char leftH, unsigned char leftL, unsigned char rightH, unsigned char rightL);
	void led(unsigned char ledMask, unsigned char pColor, unsigned char pIntensity);
	void led(bool advance, bool play, unsigned char pColor, unsigned char pIntensity);
	void digitalOut(unsigned char outBits);
	void digitalOut(bool outA, bool outB, bool outC);
	void lowSideDrivePWM(unsigned char driveA, unsigned char driveB, unsigned char driveC);
	void lowSideDrive(bool driveA, bool driveB, bool driveC);
	void lowSideDrive(unsigned char drives);
	void sendIR(unsigned char value);
	void song(unsigned char id, vector<Note> notes);
	void playSong(unsigned char id);

	//Inputs
	vector<unsigned char> sensors(unsigned char id);
	vector<unsigned char> queryList(vector<unsigned char> list);
	///stream
	//pause/start stream

	//Script
	void script(vector<unsigned char> opcodes);
	void playScript();
	vector<unsigned char> showScript();

	//Wait commands
	void waitTime(unsigned char time);
	void waitDistance(int dist);
	void waitDistance(unsigned char distH, unsigned char distL);
	void waitAngle(int ang);
	void waitAngle(unsigned char angH, unsigned char angL);
	void waitEvent(unsigned char event);


private:
	io_service* ioSrv;
    serial_port* serPort;
	// unsigned int baudRate;
	string device;
};

#endif
