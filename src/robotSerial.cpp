#include "robotSerial.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <string>
#include <vector>
#include <stdexcept>
#include <syslog.h>

using namespace std;
using namespace boost;
using namespace boost::asio;

Note::Note(unsigned char num, unsigned char len)
{

}


RobotSerial::RobotSerial(string serialPath)
{
    ioSrv = new io_service();
    serPort = new serial_port(*ioSrv, serialPath);

    if(!serPort->is_open())
    {
        syslog(LOG_ERR, "Unable to open serial port. TTY [%s]", serialPath);
        throw runtime_error("Unable to open serial port.");
    }

    syslog(LOG_DEBUG, "Setting baud rate. Baud [%d]", DEFAULT_BAUD);
    serial_port_base::baud_rate baud_opt(DEFAULT_BAUD);
    serPort->set_option(baud_opt);
}


RobotSerial::~RobotSerial()
{
    delete serPort;
    delete ioSrv;
}


//System Controls
void RobotSerial::start()
{
    syslog(LOG_DEBUG, "Sending start signal.");
    unsigned char msg[1];
    msg[0] = OPCODE_START;
    write(*serPort, buffer(msg, 1));
}


void RobotSerial::full()
{
    syslog(LOG_DEBUG, "Sending full control signal.");
    unsigned char msg[1];
    msg[0] = OPCODE_FULL;
    write(*serPort, buffer(msg, 1));
}


void RobotSerial::baud(unsigned char bps)
{
    if(bps > 11)
    {
        syslog(LOG_ERR, "Invalid baud provided. Baud [%d]", bps);
        throw runtime_error("Invalid baud rate.");
    }

    syslog(LOG_NOTICE, "Changing baud rate. Baud [%d]", bps);
    unsigned char msg[2];
    msg[0] = OPCODE_BAUD;
    msg[1] = bps;

    write(*serPort, buffer(msg, 2));
}


void RobotSerial::safe()
{
    syslog(LOG_DEBUG, "Sending safe mode signal.");
    unsigned char msg[1];
    msg[0] = OPCODE_SAFE;
    write(*serPort, buffer(msg, 1));
}


//Demos
void RobotSerial::demo(unsigned char mode)
{
    if(mode != 255 && mode > 9)
    {
        syslog(LOG_ERR, "Invalid demo number. Demo [%d]", mode);
        throw runtime_error("Invalid demo number.");
    }

    syslog(LOG_DEBUG, "Sending demo mode signal. Demo [%d]", mode);
    unsigned char msg[2];
    msg[0] = OPCODE_DEMO;
    msg[1] = mode;
    write(*serPort, buffer(msg, 2));
}


void RobotSerial::cover()
{
    syslog(LOG_DEBUG, "Sending cover signal.");
    unsigned char msg[1];
    msg[0] = OPCODE_COVER;
    write(*serPort, buffer(msg, 1));
}


void RobotSerial::coverDock()
{
    syslog(LOG_DEBUG, "Sending cover dock signal.");
    unsigned char msg[1];
    msg[0] = OPCODE_COVER_DOCK;
    write(*serPort, buffer(msg, 1));
}


void RobotSerial::spot()
{
    syslog(LOG_DEBUG, "Sending spot signal.");
    unsigned char msg[1];
    msg[0] = OPCODE_SPOT;
    write(*serPort, buffer(msg, 1));
}


//Actuator
void RobotSerial::drive(int vel, int rad)
{
    if(vel < -500 || vel > 500)
    {
        syslog(LOG_ERR, "Invalid velocity. Velocity [%d]", vel);
        throw runtime_error("Invalid velocity.");
    }

    if((rad < -2000 || rad > 2000) &&
        rad != DRIVE_STRAIGHT &&
        rad != DRIVE_TURN_RIGHT &&
        rad != DRIVE_TURN_LEFT)
    {
        syslog(LOG_ERR, "Invalid turn radius. Radius [%d]", rad);
        throw runtime_error("Invalid turn radius.");
    }

    syslog(LOG_NOTICE, "Sending drive signal. Velocity [%d] Radius [%d]", vel, rad);

    this->drive((unsigned char)((vel >> 8)&0xFF),
                (unsigned char)(vel & 0xFF),
                (unsigned char)((rad >> 8)&0xFF),
                (unsigned char)(rad & 0xFF));
}


void RobotSerial::drive(unsigned char velH, unsigned char velL,
                        unsigned char radH, unsigned char radL)
{
    syslog(LOG_DEBUG, "Sending drive signal. VelH [%d] VelL [%d] RadH [%d] RadL [%d]",
           velH, velL, radH, radL);

    unsigned char msg[5];
    msg[0] = OPCODE_DRIVE;
    msg[1] = velH;
    msg[2] = velL;
    msg[3] = radH;
    msg[4] = radL;

    write(*serPort, buffer(msg, 5));
}


void RobotSerial::driveDirect(int left, int right)
{
    if(left > 500 || left < -500)
    {
        syslog(LOG_ERR, "Invalid left velocity. Velocity [%d]", left);
        throw runtime_error("Invalid left velocity.");
    }

    if(right > 500 || right < -500)
    {
        syslog(LOG_ERR, "Invalid right velocity. Velocity [%d]", right);
        throw runtime_error("Invalid right velocity.");
    }

    syslog(LOG_NOTICE, "Sending drive signal. Left [%d] Right [%d]", left, right);

    this->drive((unsigned char)((left >> 8)&0xFF),
                (unsigned char)(left&0xFF),
                (unsigned char)((right >> 8)&0xFF),
                (unsigned char)(right&0xFF));
}


void RobotSerial::driveDirect(unsigned char leftH, unsigned char leftL,
                         unsigned char rightH, unsigned char rightL)
{
    syslog(LOG_DEBUG, "Sending drive signal. LeftH [%d] LeftL [%d] RightH [%d] RightL [%d]",
           leftH, leftL, rightH, rightL);

    unsigned char msg[5];
    msg[0] = OPCODE_DRIVE_DIRECT;
    msg[1] = rightH;
    msg[2] = rightL;
    msg[3] = leftH;
    msg[4] = leftL;

    write(*serPort, buffer(msg, 5));
}


void RobotSerial::led(unsigned char ledMask, unsigned char pColor,
                 unsigned char pIntensity)
{
    syslog(LOG_DEBUG, "Sending led signal. Mask [%#x] Color [%d] Intensity [%d]",
           ledMask, pColor, pIntensity);

    unsigned char msg[4];
    msg[0] = OPCODE_LEDS;
    msg[1] = ledMask;
    msg[2] = pColor;
    msg[3] = pIntensity;

    write(*serPort, buffer(msg, 4));
}


void RobotSerial::led(bool advance, bool play, unsigned char pColor,
                 unsigned char pIntensity)
{
    unsigned char ledMask = 0;

    if (advance)
    {
        ledMask |= LED_ADVANCE;
    }

    if (play)
    {
        ledMask |= LED_PLAY;
    }

    this->led(ledMask, pColor, pIntensity);
}


void RobotSerial::digitalOut(unsigned char outBits)
{
    syslog(LOG_DEBUG, "Sending digital output signal. Out [%#x]", outBits);
    unsigned char msg[2];
    msg[0] = OPCODE_DIGITAL_OUT;
    msg[1] = outBits;

    write(*serPort, buffer(msg, 2));
}


void RobotSerial::digitalOut(bool outA, bool outB, bool outC)
{
    unsigned char outMask = 0;

    if (outA)
    {
        outMask |= 1;
    }

    if (outB)
    {
        outMask |= 2;
    }

    if (outC)
    {
        outMask |= 4;
    }

    this->digitalOut(outMask);
}


void RobotSerial::lowSideDrivePWM(unsigned char driveA, unsigned char driveB,
                                  unsigned char driveC)
{
    unsigned char msg[4];

    if(driveA > 128 || driveB > 128 || driveC > 128)
    {
        syslog(LOG_ERR, "Invalid low side drive value. DriveA [%d] DriveB [%d] DriveC [%d]",
               driveA, driveB, driveC);
        throw runtime_error("Invalid PWM duty cycle. Must be 0-128.");
    }

    syslog(LOG_DEBUG, "Sending low side drive PWM signal. DriveA [%d] DriveB [%d] DriveC[%d]",
           driveA, driveB, driveC);

    msg[0] = OPCODE_LOW_SIDE_PWM;
    msg[1] = driveC;
    msg[2] = driveB;
    msg[3] = driveA;

    write(*serPort, buffer(msg, 4));
}


void RobotSerial::lowSideDrive(unsigned char drives)
{
    unsigned char msg[2];

    syslog(LOG_DEBUG, "Sending low side drive signal. Drives [%#x]", drives);

    msg[0] = OPCODE_LOW_SIDE;
    msg[1] = drives;

    write(*serPort, buffer(msg, 2));
}


void RobotSerial::lowSideDrive(bool driveA, bool driveB, bool driveC)
{
    unsigned char driveMask = 0;

    if (driveA)
    {
        driveMask |= 1;
    }

    if (driveB)
    {
        driveMask |= 2;
    }

    if (driveC)
    {
        driveMask |= 4;
    }

    this->lowSideDrive(driveMask);
}


void RobotSerial::sendIR(unsigned char value)
{
    unsigned char msg[2];

    syslog(LOG_DEBUG, "Sending IR signal. Value [%#x]", value);

    msg[0] = OPCODE_SEND_IR;
    msg[1] = value;

    write(*serPort, buffer(msg, 2));
}


void RobotSerial::song(unsigned char id, vector<Note> notes)
{
    if(id > 15)
    {
        syslog(LOG_ERR, "Invalid song number. Song [%d]", id);
        throw runtime_error("Invalid song number. Must be 0-15.");
    }

    if(notes.size() > 16 || notes.size() == 0)
    {
        syslog(LOG_ERR, "Invalid number of notes. Length [%d]", notes.size());
        throw runtime_error("Invalid number of notes. Must be 1-16.");
    }

    unsigned int length = (notes.size()<<1) + 3;
    unsigned char* msg;
    msg = (unsigned char*)malloc(length);

    unsigned int i = 0;

    syslog(LOG_DEBUG, "Sending song signal. Song [%d] Length [%d]", 
           id, notes.size());

    msg[i++] = OPCODE_SONG;
    msg[i++] = id;
    msg[i++] = notes.size();

    for(vector<Note>::iterator it = notes.begin();
        it != notes.end(); it++)
    {
        msg[i++] = it->number;
        msg[i++] = it->length;
    }

    write(*serPort, buffer(msg, length));
    free(msg);
}


void RobotSerial::playSong(unsigned char id)
{
    if(id > 15)
    {
        syslog(LOG_ERR, "Invalid song number. Song [%d]", id);
        throw runtime_error("Invalid song number. Must be 0-15.");
    }

    unsigned char msg[2];

    syslog(LOG_DEBUG, "Sending play song signal. Song [%d]", id);

    msg[0] = OPCODE_SONG_PLAY;
    msg[1] = id;

    write(*serPort, buffer(msg, 2));
}


//Inputs
vector<unsigned char> RobotSerial::sensors(unsigned char id)
{
    vector<unsigned char> data;
    //TODO
    return data;
}


vector<unsigned char> RobotSerial::queryList(vector<unsigned char> list)
{
    vector<unsigned char> data;
    //TODO
    return data;
}


//TODO:
///stream
//pause/start stream


//Script
void RobotSerial::script(vector<unsigned char> opcodes)
{
    //TODO
}


void RobotSerial::playScript()
{
    //TODO
}


vector<unsigned char> RobotSerial::showScript()
{
    vector<unsigned char> data;
    //TODO
    return data;
}


//Wait commands
void RobotSerial::waitTime(unsigned char time)
{
    //TODO
}


void RobotSerial::waitDistance(int dist)
{
    //TODO
}


void RobotSerial::waitDistance(unsigned char distH, unsigned char distL)
{
    //TODO
}


void RobotSerial::waitAngle(int ang)
{
    //TODO
}


void RobotSerial::waitAngle(unsigned char angH, unsigned char angL)
{
    //TODO
}


void RobotSerial::waitEvent(unsigned char event)
{
    //TODO
}
