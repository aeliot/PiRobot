#include "robotSerial.hpp"
#include <boost/thread/thread.hpp>

int main( int argc, char** argv){
    // RobotSerial robot("/dev/ttyAMA0");
    RobotSerial robot("/dev/ttyUSB0");
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    robot.start();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    robot.full();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));

    robot.led(true, true, 100, 255);
    robot.drive(500, 0x8000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    robot.drive(-500, 0x8000);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    robot.drive(0,0);

    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}
