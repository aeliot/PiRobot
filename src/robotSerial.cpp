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

RobotSerial::RobotSerial(string serialPath){
    ioSrv = new io_service();
    serPort = new serial_port(*ioSrv, serialPath);

    if(!serPort->is_open()){
        throw runtime_error("Unable to open serial port.");
    }

    serial_port_base::baud_rate baud_opt(DEFAULT_BAUD);
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

    if(bps > 11 && bps < 0){
        throw runtime_error("Invalid baud rate.");
    }

    unsigned char msg[2];
    msg[0] = OPCODE_BAUD;
    msg[1] = bps;

    write(*serPort, buffer(msg, 2));
}

void RobotSerial::safe(){
    unsigned char msg[1];
    msg[0] = OPCODE_SAFE;
    write(*serPort, buffer(msg, 1));
}


//Demos
void RobotSerial::demo(unsigned char mode){

    if(mode != 255 && mode > 9 && mode < 0){
        throw runtime_error("Invalid demo number.");
    }

    unsigned char msg[2];
    msg[0] = OPCODE_DEMO;
    msg[1] = mode;
    write(*serPort, buffer(msg, 2));
}

void RobotSerial::cover(){
    unsigned char msg[1];
    msg[0] = OPCODE_COVER;
    write(*serPort, buffer(msg, 1));
}

void RobotSerial::coverDock(){
    unsigned char msg[1];
    msg[0] = OPCODE_COVER_DOCK;
    write(*serPort, buffer(msg, 1));
}

void RobotSerial::spot(){
    unsigned char msg[1];
    msg[0] = OPCODE_SPOT;
    write(*serPort, buffer(msg, 1));
}


//Actuator
void RobotSerial::drive(int vel, int rad){
    if(vel < -500 || vel > 500){
        throw runtime_error("Invalid velocity.");
    }
    if((rad < -2000 || rad > 2000) &&
        rad != DRIVE_STRAIGHT &&
        rad != DRIVE_TURN_RIGHT &&
        rad != DRIVE_TURN_LEFT){

        throw runtime_error("Invalid turn radius.");
    }

    this->drive((unsigned char)((vel >> 8)&0xFF),
                (unsigned char)(vel & 0xFF),
                (unsigned char)((rad >> 8)&0xFF),
                (unsigned char)(rad & 0xFF));
}

void RobotSerial::drive(unsigned char velH, unsigned char velL,
                   unsigned char radH, unsigned char radL){
    unsigned char msg[5];
    msg[0] = OPCODE_DRIVE;
    msg[1] = velH;
    msg[2] = velL;
    msg[3] = radH;
    msg[4] = radL;

    write(*serPort, buffer(msg, 5));
}

void RobotSerial::driveDirect(int left, int right){
    if(left > 500 || left < -500){
        throw runtime_error("Invalid left velocity.");
    }
    if(right > 500 || right < -500){
        throw runtime_error("Invalid right velocity.");
    }

    this->drive((unsigned char)((left >> 8)&0xFF),
                (unsigned char)(left&0xFF),
                (unsigned char)((right >> 8)&0xFF),
                (unsigned char)(right&0xFF));
}

void RobotSerial::driveDirect(unsigned char leftH, unsigned char leftL,
                         unsigned char rightH, unsigned char rightL){
    unsigned char msg[5];
    msg[0] = OPCODE_DRIVE_DIRECT;
    msg[1] = rightH;
    msg[2] = rightL;
    msg[3] = leftH;
    msg[4] = leftL;

    write(*serPort, buffer(msg, 5));

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
