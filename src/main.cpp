#include "robotSerial.hpp"
#include <boost/thread/thread.hpp>

int main( int argc, char** argv){
    RobotSerial robot;
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    robot.start();
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    robot.full();
}
