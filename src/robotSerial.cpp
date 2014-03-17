#include "robotSerial.hpp"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <string>
#include <vector>
#include <stdexcept>

using namespace std;
using namespace boost;
using namespace boost::asio;

Note::Note(unsigned char num, unsigned char len){
}

RobotSerial::RobotSerial(){
    ioSrv = new io_service();
    serPort = new serial_port(*ioSrv, "/dev/ttyAMA0");

    if(!serPort->is_open()){
        throw runtime_error("Unable to open serial port.");
    }

    serial_port_base::baud_rate baud_opt(57600);
    serPort->set_option(baud_opt);
}

RobotSerial::~RobotSerial(){
    delete serPort;
    delete ioSrv;
}

//System Controls
void RobotSerial::start(){
    unsigned char msg[1];
    msg[0] = OPCODE_START;
    write(*serPort, buffer(msg, 1));
}

void RobotSerial::full(){
    unsigned char msg[1];
    msg[0] = OPCODE_FULL;
    write(*serPort, buffer(msg, 1));
}

void RobotSerial::baud(char bps){

}

void RobotSerial::safe(){

}


//Demos
void RobotSerial::demo(unsigned char mode){

}

void RobotSerial::cover(){

}

void RobotSerial::coverDock(){

}

void RobotSerial::spot(){

}


//Actuator
void RobotSerial::drive(int vel, int rad){

}

void RobotSerial::drive(unsigned char velH, unsigned char velL,
                   unsigned char radH, unsigned char radL){

}

void RobotSerial::driveDirect(int left, int right){

}

void RobotSerial::driveDirect(unsigned char leftH, unsigned char leftL,
                         unsigned char rightH, unsigned char rightL){

}

void RobotSerial::led(unsigned char ledMask, unsigned char pColor,
                 unsigned char pIntensity){

}

void RobotSerial::led(bool advance, bool play, unsigned char pColor,
                 unsigned char pIntensity){

}

void RobotSerial::digitalOut(unsigned char outBits){

}

void RobotSerial::digitalOut(bool outA, bool outB, bool outC){

}

void RobotSerial::lowSideDrivePWM(unsigned char driveA, unsigned char driveB,
                             unsigned char driveC){

}

void RobotSerial::lowSideDrive(bool driveA, bool driveB, bool driveC){

}

void RobotSerial::lowSideDrive(unsigned char drives){

}

void RobotSerial::sendIR(unsigned char value){

}

void RobotSerial::song(unsigned char id, vector<Note> notes){

}

void RobotSerial::playSong(unsigned char id){

}


//Inputs
vector<unsigned char> RobotSerial::sensors(unsigned char id){

    vector<unsigned char> data;
    return data;
}

vector<unsigned char> RobotSerial::queryList(vector<unsigned char> list){

    vector<unsigned char> data;
    return data;
}

//TODO:
///stream
//pause/start stream


//Script
void RobotSerial::script(vector<unsigned char> opcodes){

}

void RobotSerial::playScript(){

}

vector<unsigned char> RobotSerial::showScript(){

    vector<unsigned char> data;
    return data;
}


//Wait commands
void RobotSerial::waitTime(unsigned char time){

}

void RobotSerial::waitDistance(int dist){

}

void RobotSerial::waitDistance(unsigned char distH, unsigned char distL){

}

void RobotSerial::waitAngle(int ang){

}

void RobotSerial::waitAngle(unsigned char angH, unsigned char angL){

}

void RobotSerial::waitEvent(unsigned char event){

}
