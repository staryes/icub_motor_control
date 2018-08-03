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
#include <yarp/dev/IControlLimits2.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class ServerMod : public yarp::os::RFModule
{
private:

    // define one input port to receive the angle from the client
    // hint: think about which port you need, buffered? simple?
    // FILL IN THE CODE
        Port sPort;
    PolyDriver                       driver;
    IControlMode2                   *imod;
    IEncoders                       *ienc;
    IPositionControl2               *ipos;
    IControlLimits2                 *ilim;

    double                          min, max;
    int                             nAxes;


    //Implement the BufferedPort callback, as soon as new data is coming
    void moveArm(Bottle* bot)
    {
        static bool invert = false;
        if (!bot->get(0).isDouble())
        {
            yError()<<"Expecting a double on read...";
            return;
        }
        double angle = bot->get(0).asDouble();
        if(ipos)
        {
            // the arm will move between -angle and angle
            if (invert)
            {
                angle = -1 * angle;
                invert = false;
            }
            else
            {
                invert = true;
            }

            // check that "angle" is inside the joint
            // limits before moving, if so move the arm
            // FILL IN CODE
            ilim->getLimits(2, &min, &max); // joint 2
            if(angle < max && angle > min)
                    ipos->positionMove(2, angle);

        }
    }

public:

    ServerMod() : imod(nullptr), ienc(nullptr), ipos(nullptr), ilim(nullptr),
      min(0.0), max(0.0), nAxes(0)
    {}
    /****************************************************/
    bool configPorts()
    {

        // open the input port /server/input
        // FILL IN THE CODE
            if (!sPort.open("/server/input"))
            {
                    yError() << "cannot open the server input port";
                    return -1;
            }

            sPort.addOutput("/client/output");
        return true;
    }

    /***************************************************/
    bool configDevice()
    {
        // configure the options for the driver
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/icubSim/left_arm");
        option.put("local","/server");

        // open the driver
        if (!driver.open(option))
        {
            yError()<<"Unable to open the device driver";
            return false;
        }

        // open the views
        if (!driver.view(imod) || !driver.view(ienc) || !driver.view(ipos) || !driver.view(ilim))
        {
           yError()<<"Failed to view one of the interfaces";
           return false;
        }

        // get joint's limits
        ilim->getLimits(2, &min, &max);
        // tell the device we aim to control
        // in position mode all the joints
        ienc->getAxes(&nAxes);

        // tell the device we aim to control
        // in position mode all the joints
        // FILL IN THE CODE
        imod->setControlMode(2, VOCAB_CM_POSITION);

        // set ref speed for the joint 2 to 40.0 deg/s
        // FILL IN THE CODE
        ipos->setRefSpeed(2, 40.0);

        return true;
    }

    /****************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool config_ok;

        // configure the device for controlling the head
        config_ok = configDevice();

        // configure the ports
        config_ok = configPorts() && config_ok;

        return config_ok;
    }

    /****************************************************/
    double getPeriod()
    {
        return 2.0;
    }

    /****************************************************/
    bool close()
    {
        // close the port and close the PolyDriver
        // FILL IN THE CODE
            yInfo() << "calling close server";
        return true;
    }

    /****************************************************/
    bool interrupt()
    {

        // interrupt the port
        // FILL IN THE CODE
            yInfo() << "Interrupt happened";
        return true;
    }

    /****************************************************/
    bool updateModule()
    {
        Bottle* bot = nullptr;
        // read from the input port passing "bot"
        // FILL IN THE CODE
        Bottle cmd;
        if(!sPort.read(cmd))
        {
                yError() << "error while receiving";
                return 0;
        }

        bot = &cmd;
        yInfo() << "the cmd is " << cmd.toString().c_str();

        if (bot)
        {
            // try to move the arm
            moveArm(bot);
        }

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

    ServerMod mod;
    return mod.runModule(rf);
}
