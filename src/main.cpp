#include "robotSerial.hpp"
#include <boost/thread/thread.hpp>
#include <syslog.h>
#include <map>

using namespace std;

enum class Args : int
{
    SRV_PORT,
    TTY
};


enum class Opts : int
{
    DEBUG,
    VERB
};


void printHelp()
{
    cout << "PiRobot" << endl;
    cout << "=======" << endl;
    cout << "Flag          Type      Default           Description     " << endl;
    cout << "----------------------------------------------------------" << endl;
    cout << "-p, --port    [int]     12000             TCP Server Port." << endl;
    cout << "-t, --tty     [string]  /dev/ttyAMA0      Serial port.    " << endl;
    cout << "-d, --debug                               Log debug level." << endl;
    cout << "-v, --verbose                             Log to stdout.  " << endl;
    cout << "-h, --help                                Print help.     " << endl;
}


void parsArgs(int argc, char** argv, map<Args, string>& args, 
             map<Opts, bool>& opts)
{
    args[Args::SRV_PORT] = "12000";
    args[Args::TTY] = "/dev/ttyAMA0";

    opts[Opts::DEBUG] = false;
    opts[Opts::VERB] = false;

    int i = 0;
    while(i < argc)
    {
        if(argv[i] == "-p" || argv[i] == "--port")
        {
            i++;
            if(i => argc)
            {
                cerr << "ERROR: Bad argument." << endl;
                printHelp();
                exit(-1);
            }
            args[Args::SRV_PORT] = argv[i];
            i++;
        }
        else if(argv[i] == "-t" || argv[i] == "--tty")
        {
            i++;
            if(i => argc)
            {
                cerr << "ERROR: Bad argument." << endl;
                printHelp();
                exit(-1);
            }
            args[Args::TTY] == argv[i];
            i++;
        }
        else if(argv[i] == "-d" || argv[i] == "--debug")
        {
            opts[Opts::DEBUG] = true;
            i++;
        }
        else if(argv[i] == "-v" || argv[i] == "--verbose")
        {
            opts[Opts::VERB] = true;
        }
        else if(argv[i] == "-h" || argv[i] == "--help")
        {
            printHelp();
            exit(0);
        }
        else
        {
            cerr << "ERROR: Unknown argument." << endl;
            printHelp();
            exit(-1);
        }
    }   
}


int main( int argc, char** argv)
{
    map<Args, string> args;
    map<Opts, string> opts;
    parsArgs(argc, argv, args, opts);

    if(opts[Opts::DEBUG])
    {
        setlogmask(LOG_UPTO(LOG_DEBUG));
    }
    else
    {
        setlogmask(LOG_UPTO(LOG_NOTICE));
    }

    int logOpts = LOG_CONS | LOG_PID | LOG_NDELAY;
    if(opts[Opts::VERB])
    {
        logOpts |= LOG_PERROR;
    }

    openlog("PiRobot", logOpts, LOG_LOCAL1);

    syslog(LOG_NOTICE, "Starting serial communication. TTY[%s]", args[Args::TTY]);
    RobotSerial robot(args[Args::TTY]);

    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    syslog(LOG_DEBUG, "Sending start signal.");
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
    closelog();
}
