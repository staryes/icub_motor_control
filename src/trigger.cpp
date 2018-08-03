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

class TriggerMod : public yarp::os::RFModule
{
private:
    Port                             triggerPort;
    PolyDriver                       driver, driver2;
    IControlMode2                   *imod;
    IEncoders                       *ienc;
    IPositionControl2               *ipos, *ipos2;
    IControlLimits2                 *ilim;

    double                          min, max, angle, period;
    int                             nAxes;


    //Implement the BufferedPort callback, as soon as new data is coming
    void moveArm()
    {
        if(ipos)
        {

            angle = -1 * angle;

            if (angle < min || angle > max)
            {
                yError()<<"The angle is out the joint's limits";
                return;
            }

            ipos->positionMove(2, angle);
        }
    }

public:

    TriggerMod() : imod(nullptr), ienc(nullptr), ipos(nullptr), ipos2(nullptr),
      ilim(nullptr), min(0.0), max(0.0), angle(0.0), period(0.0), nAxes(0)
    {}

    /****************************************************/
    bool configDevice()
    {
        // configure the options for the driver
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/icubSim/right_arm");
        option.put("local","/triggerR");

        // open the driver
        if (!driver.open(option))
        {
            yError()<<"Unable to open the device driver";
            return false;
        }

        option.unput("remote");
        option.unput("local");

        option.put("remote","/icubSim/left_arm");
        option.put("local","/triggerL");

        // open the driver
        if (!driver2.open(option))
        {
            yError()<<"Unable to open the device driver";
            return false;
        }


        // open the views
        if (!driver.view(imod) || !driver.view(ienc) || !driver.view(ipos) || !driver.view(ilim)
                || !driver2.view(ipos2))
        {
           yError()<<"Failed to view one of the interfaces";
           return false;
        }

        // tell the device we aim to control
        // in position mode all the joints
        ienc->getAxes(&nAxes);
        vector<int> modes(nAxes,VOCAB_CM_POSITION);
        imod->setControlModes(modes.data());

        // set ref speed

        ipos->setRefSpeed(2, 40.0);

        ipos->setRefSpeed(0, 60.0);
        ipos2->setRefSpeed(0, 60.0);
        ipos->setRefSpeed(4, 60.0);
        ipos2->setRefSpeed(4, 60.0);

        // get joint's limits
        ilim->getLimits(2, &min, &max);

        // move iCub to the starting pose
        ipos->positionMove(0, -90.0);
        ipos2->positionMove(0, -90.0);
        ipos->positionMove(4, 60.0);
        ipos2->positionMove(4, 60.0);

        bool idle1 = false; bool idle2 = false;
        bool idle3 = false; bool idle4 = false;

        double start = yarp::os::Time::now();

        while (!idle1 || !idle2 || !idle3 || !idle4)
        {
            ipos->checkMotionDone(0, &idle1);
            ipos->checkMotionDone(4, &idle2);
            ipos2->checkMotionDone(0, &idle3);
            ipos2->checkMotionDone(4, &idle4);
            // 10 sec of timeout
            if ((yarp::os::Time::now() - start) > 10.0)
            {
                yError()<<"CheckMotionDone gone in timeout";
                return false;
            }
            yarp::os::Time::delay(0.4);

        }

        return true;
    }

    /****************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool configured = configDevice();

        if (!triggerPort.open("/trigger/output"))
        {
            return false;
        }
        while (triggerPort.getOutputCount() == 0)
        {
            yInfo()<<"Waiting the connection to the client...";
            yarp::os::Time::delay(0.3);
        }


        Bottle signal;
        signal.addInt(26);
        triggerPort.write(signal);

        yInfo() << "the signal is " << signal.toString().c_str();

        if (rf.check("magic-number"))
        {
            int magicNum = rf.find("magic-number").asInt();

            period = magicNum - 26000;
            angle = (magicNum - period)/1000.0;

            return true;
        }

        if (rf.check("angle"))
        {
            angle = rf.find("angle").asDouble();
        }

        if (rf.check("period"))
        {
            period = rf.find("period").asDouble();
        }

        return configured;
    }

    /****************************************************/
    double getPeriod()
    {
        return period;
    }

    /****************************************************/
    bool close()
    {
        return true;
    }

    /****************************************************/
    bool interrupt()
    {
        return true;
    }

    /****************************************************/
    bool updateModule()
    {

        moveArm();

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

    TriggerMod mod;
    return mod.runModule(rf);
}

