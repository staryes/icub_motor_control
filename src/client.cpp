#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl2.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


class ClientMod : public yarp::os::RFModule
{
private:

    // define one output port to comunicate to the server and
    // one input port to receive the trigger to start moving the
    // arm
    // hint: think about which port you need, buffered? simple? both?
    // FILL IN THE CODE
    double angle, period;
    bool triggered;

        Port inPort;
        Port outPort;

        Bottle trigger_said;

public:

    // set the correct angle to send to the server and the period of the thread
    // FILL IN THE CODE
    ClientMod(): angle(0.0), period(0.0), triggered(false)
    {}
    /****************************************************/
    bool configPorts()
    {
        // open all ports and check that everything is fine
        // output port: /client/output
        // input port: /client/input
        // FILL IN THE CODE
            if (!inPort.open("/client/input"))
            {
                    return false;
            }
            inPort.addOutput("/trigger/output");

            if(!outPort.open("/client/output"))
            {
                    return false;
            }
            while (outPort.getOutputCount() == 0)
            {
                    yInfo()<<"Waiting the connection to the server...";
                    yarp::os::Time::delay(0.3);
            }
        return true;
    }

    /****************************************************/
    bool configure(ResourceFinder &rf)
    {
        // get "angle" input parameter
        // configure the ports
        bool conf = configPorts();

        // Check if angle is passed by command line argument
        if (rf.check("angle"))
        {
            angle  = rf.find("angle").asDouble();
        }

        // Check if period is passed by command line argument
        if (rf.check("period"))
        {
            period = rf.find("period").asDouble();
        }
        return conf;
    }

    /****************************************************/
    double getPeriod()
    {
        return period;
    }

    /****************************************************/
    bool close()
    {
        // close ports
        // FILL IN THE CODE
            yInfo() << "Calling close client";
            outPort.close();
            inPort.close();
        return true;
    }

    /****************************************************/
    bool interrupt()
    {
        // interrupt ports
        // FILL IN THE CODE
            yInfo() << "Interrupt happened";
        return true;
    }

    /****************************************************/
    bool updateModule()
    {
            static double angle = 0.0;
        if (!triggered)
        {
            // read from the input port the signal from the
            // trigger for starting to send data to the server
            // FILL IN CODE
                if(!inPort.read(trigger_said))
                {
                        yError() << "error while reading from trigger!";
                        return 0;
                }

                angle = trigger_said.get(0).asDouble();
                yInfo() << "angle " << angle;
            triggered = true;
        }

        // once triggered prepare the bottle containing the
        // angle and send it to the server through the
        // output port
        // FILL IN CODE

        Bottle output;

        output.addDouble(angle);
        output.addDouble(26.0);
        yInfo() << output.toString().c_str();

        outPort.write(output);

        return true;
    }

};

int main(int argc, char *argv[]) {
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.setDefaultContext("assignment_motor-control");
    rf.configure(argc,argv);

    ClientMod mod;
    return mod.runModule(rf);
}
