/*!
******************************************************************************
**  CarMaker - Version 9.0.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*
* Description:
* - Prototype/Proof of Concept
* - Unsupported ROS Example with CarMaker
* - Structure may change in future!
* - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
* - Basic communication with or without parameterizable synchronization
*
*
* ToDo:
* - C++!!!
* - ROS naming/way/namespaces
* - parameter: CarMaker read, ROS set by service?
*   -> ROS parameter mechanism seems better solution!
* - node/topic/... destruction to allow dynamic load/unload
*   when TestRun starts instead of initialization at CarMaker startup
* - New Param_Get() function to read parameters from Infofile
* - ...
*
*/


/* CarMaker
* - include other headers e.g. to access to vehicle data
*   - e.g. "Vehicle.h" or "Vehicle/Sensor_*.h".
* - additional headers can be found in "<CMInstallDir>/include/"
* - see Reference Manual, chapter "User Accessible Quantities" to find some variables
*   that are already defined in DataDictionary and their corresponding C-Code Name
*/
#include "Log.h"
#include "DataDict.h"
#include "SimCore.h"
#include "InfoUtils.h"

#include "apo.h"
#include "GuiCmd.h"

/* CarMaker Model */
#include "Vehicle.h"
#include "DrivMan.h"
#include "VehicleControl.h"
#include "Vehicle/Sensor_CameraRSI.h"
#include "Vehicle/Sensor_RadarRSI.h"
#include "Vehicle/Sensor_LidarRSI.h"
#include "Vehicle/Sensor_Inertial.h"
#include "Vehicle/Sensor_GNav.h"


/* ROS CM */
#include "cmrosutils/CMRosUtils.h"      /* Node independent templates, ...*/
#include "cmrosutils/CMRosIF_Utils.h"   /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/CMRemote.h"        /* Basic service for CarMaker remote from ROS */

/* ASLAN */
#include "sd_vehicle_interface/sd_vehicle_interface.h" /* ASLAN SD Vehicle interface */
#include "sensor_msgs/PointCloud2.h"    /* ROS PointCloud2 for sensor inputs */
#include "sensor_msgs/NavSatFix.h"      /* ROS Navigation Satellite fix */
#include "geometry_msgs/TwistStamped.h" /* ROS Twist command */

/* Following header from external ROS node can be used to get topic/service/... names
* Other mechanism:
* 1. Put names manually independently for each node
* 2. Using command line arguments or launch files and ROS remapping
* - Doing so, only general message headers are necessary
*/


/*! String and numerical version of this Node
*  - String:    e.g. <Major>.<Minor>.<Patch>
*  - Numerical: e.g. <nDigitsMajor><2DigitsMinor><2DigitsPatch>
*/
#define CMNODE_VERSION "0.9.0"
#define CMNODE_NUMVER  900


/* NDEBUG is set in CarMaker Makefile/MakeDefs in OPT_CFLAGS */
#if !defined NDEBUG
#  warning "Debug options are enabled!"
#  define DBLOG LOG
#else
#  define DBLOG(...)
#endif

/* Not beautiful but consistent with external ROS Node
* where ROS_INFO is used (implicit newline)*/
# define LOG(frmt, ...)  Log(frmt "\n", ##__VA_ARGS__)


/* General switches for CarMaker ROS Node */
typedef enum tCMNode_Mode {
    CMNode_Mode_Disabled  = 0,  /*!< Node is disabled. e.g. don't publish. */
    CMNode_Mode_Default   = 1,  /*!< Node is enabled, spinOnce is used  */
    CMNode_Mode_Threaded  = 2   /*!< Node is enabled, spin in parallel thread
    - Messages are received all the time
    - Data is updated at defined position, e.g. *_In()
    - Currently not implemented! */
} tCMNode_Mode;



/* Managing synchronization between CarMaker Node and other ROS nodes */
typedef enum tCMNode_SyncMode {
    CMNode_SyncMode_Disabled  = 0, /*!< No synchronization on CarMaker side */
    CMNode_SyncMode_Tpc       = 1  /*!< Buffer messages or Spin until external Topics are received */
} tCMNode_SyncMode;



/* Global struct for this Node */
static struct {
    unsigned long  CycleNoRel;  /*!< CarMaker relative cycle number, e.g. since start of TestRun */

    struct {
        double          Duration;      /*!< Time spent for synchronization task */
        int             nCycles;       /*!< Number of cycles in synchronization loop */
        int             CyclePrepDone; /*!< Last cycle when preparation was done */
        int             CycleJobDone;  /*!< Last cycle when job was done */
        double          SynthDelay;    /*!< Synthetic delay in seconds provided to external node to check sync */
    } Sync; /*!< Synchronization related information */

    struct {
        int             CycleNo;      /*!< Cycle number of external ROS Node (only for information) */

        /* For debugging */
        int             CycleLastOut;   /*!< Cycle number when Topic was published */
        int             CycleLastIn;    /*!< Cycle number when Topic from external ROS Node was received */
        int             CycleLastFlush; /*!< Cycle number when data from external ROS Node was provided to model */
    } Model; /*!< Model related information. ROS side! */

    struct {
        struct {
            tRosIF_TpcSub<aslan_msgs::SDControl> SDC;
        } Sub; /*!< Topics to be subscribed */

        struct {
            tRosIF_TpcPub<sensor_msgs::NavSatFix> GPS;
            tRosIF_TpcPub<sensor_msgs::PointCloud2> LidarRSI;
            tRosIF_TpcPub<sensor_msgs::PointCloud2> RadarRSI;
            tRosIF_TpcPub<geometry_msgs::TwistStamped> Velocity;


            /*!< CarMaker can be working as ROS Time Server providing simulation time
            *   starting at 0 for each TestRun */
            tRosIF_TpcPub<rosgraph_msgs::Clock> Clock;
        } Pub; /*!< Topics to be published */
    } Topics; /*!< ROS Topics used by this Node */

    struct {
        /*!< Initialization/Preparation of external ROS Node e.g. when simulation starts */

    } Services; /*!< ROS Services used by this Node (client and server)*/

    struct {
        int                 QueuePub;       /*!< Queue size for Publishers */
        int                 QueueSub;       /*!< Queue size for Subscribers */
        int                 nCyclesClock;   /*!< Number of cycles publishing /clock topic.
        CycleTime should be multiple of this value */
        tCMNode_Mode        Mode;
        tCMNode_SyncMode    SyncMode;
        double              SyncTimeMax;    /* Maximum Synchronization time */

        tRosIF_Cfg          Ros;
    } Cfg; /*!< General configuration for this Node */

    struct {
        int                 MaxSteerAng;    /*!< Maximum steering angle of the vehicle */
    } Vhcl; /*!< Vehicle parameters */

    struct {
        struct {
            double*          pos;           /*!< Mounting position on vehicle frame */
        } RL;
    } Wheel; /*!< Wheel parameters */

    struct {
        struct {
            int             Active;         /*!< Flag for engaging the GNav system */
            double*         pos;           /*!< Mounting position on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;
        } GNav;

        struct {
            int             N;              /*!< Number of LidarRSI sensors */
            double*         pos;            /*!< Mounting position on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;
        } LidarRSI;

        struct {
            int             N;              /*!< Number of RadarRSI sensors */
            double*         pos;            /*!< Mounting position on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;
            int             OutputType;     /*!< 1:Cartesian 2:Spherical 3:VRx */
        } RadarRSI;
    } Sensor; /*!< Sensor parameters */


} CMNode;



/*!
* Description:
* - Callback for ROS Topic published by external ROS Nodes
*
*/

//Subscriber Callback for SDControl from ASLAN
static void
cmnode_SDControl_CB_TpcIn(const aslan_msgs::SDControl::ConstPtr &msg) {

    int rv;
    auto in = &CMNode.Topics.Sub.SDC;

    in->Msg.torque          = msg->torque;
    in->Msg.steer           = msg->steer;

    /* Remember cycle for debugging */
    CMNode.Model.CycleLastIn = CMNode.CycleNoRel;
}



/*!
* Service callbacks
*/



/*****************************************************************************/
/**********          C-Code for interfacing with CarMaker!          **********/
/*****************************************************************************/


#ifdef __cplusplus
extern "C" {
    #endif



    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Get versions from shared library
    * - Set the returned Version to 0 if there is no dependency!
    * - Compatibility check should be done by calling procedure
    *   as early as possible(e.g. before CMRosIF_CMNode_Init()
    *
    * Arguments:
    * - CMRosIFVer = CMRosIF shared library version (User defined)
    *                - Initially filled with version of CMRosIF management library
    * - CMNumVer   = CarMaker version used for shared library at compile time (normally CM_NUMVER)
    *                - Initially filled with version of CMRosIF management library
    * - RosVersion = ROS version used for shared library at compile time (normally ROS_VERSION)
    *                - Initially filled with version requested by CMRosIF management library (0 if no request)
    *
    */
    int
    CMRosIF_CMNode_GetVersion (unsigned long *CMRosIFCMNodeNumVer, unsigned long *CMNumVer, unsigned long *RosNumVer)
    {

        *CMRosIFCMNodeNumVer = CMNODE_NUMVER;
        *CMNumVer            = CM_NUMVER;
        *RosNumVer           = ROS_VERSION;

        return 0;
    }



    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Basic Initialization
    * - e.g. create ROS Node, subscriptions, ...
    * - Return values
    *   - "rv <  0" = Error at initialization, CarMaker executable will stop
    *   - "rv >= 0" = Everything OK, CarMaker executable will continue
    *
    * Arguments:
    * - Argc/Argv  = Arguments normally provided to ROS executable are not be provided
    *                to CM executable directly, but can be set in Infofile for CMRosIF
    *                with key "Node.Args" in "Data/Config/CMRosIFParameters"
    *
    * - CMNodeName = Default CarMaker Node name
    *                - Can be parameterized in Infofile for CMRosIF
    *                - Final node name might be different (argument remapping, ...)
    *
    * - Inf        = Handle to CarMaker Infofile with parameters for this interface
    *                - Please note that pointer may change, e.g. before TestRun begins
    *
    * ToDo:
    * - Possible to create/initialize node/... before each TestRun start instead of CM startup?
    * - New Param_Get() function to read parameters from Infofile
    */
    int
    CMRosIF_CMNode_Init (int Argc, char **Argv, char *CMNodeName, struct tInfos *Inf)
    {

        int rv;
        bool rvb                = false;
        char sbuf[512]          = "";
        char keybuf[256]        = "";
        ros::NodeHandlePtr node = NULL;
        ros::V_string names;


        LOG("Initialize CarMaker ROS Node");
        LOG("  -> Node Version = %05d", CMNODE_NUMVER);
        LOG("  -> ROS Version  = %05d", ROS_VERSION);
        LOG("  -> CM Version   = %05d", CM_NUMVER);

        /* ROS initialization. Name of Node might be different after remapping! */
        if (ros::isInitialized() == false) {
            /* "Remapping arguments" functionality (launchfiles, ...)? */
            ros::init(Argc, Argv, CMNodeName);
        } else {
            //node.reset(); ToDo!
        }

        if (ros::master::check() == false) {
            LogErrF(EC_Init, "Can't contact ROS Master!\n Start roscore or run launch file e.g. via Extras->CMRosIF\n");
            ros::shutdown();
            return -1;
        }

        /* Node specific */
        CMNode.Cfg.Ros.Node = ros::NodeHandlePtr(boost::make_shared<ros::NodeHandle>());
        node                = CMNode.Cfg.Ros.Node;

        /* Publish specific */
        CMNode.Cfg.QueuePub  = iGetIntOpt(Inf, "Node.QueuePub", 1000); /* ToDo: Influence of queue length relevant? */

        /* Prepare the node to provide simulation time. CarMaker will be /clock server */
        strcpy(sbuf, "/use_sim_time");

        if ((rv = node->hasParam(sbuf)) == true) {
            node->getParam(sbuf, rvb);
            LOG("  -> Has param '%s' with value '%d'", sbuf, rvb);
        }

        /* Additional switch to provide simulation Time */
        strcpy(keybuf, "Node.UseSimTime");

        if ((rv = iGetIntOpt(Inf, keybuf, 1)) > 0) {
            /* Parameter must be set before other nodes start
            * - set parameter outside to be independent from execution order?
            */
            LOG("  -> Provide simulation time!");
            node->setParam("/use_sim_time", true); /* enable parameter if not already done */

            CMNode.Cfg.nCyclesClock  = iGetIntOpt(Inf, "Node.nCyclesClock", 1000);

            strcpy(sbuf, "/clock");
            LOG("    -> Publish '%s' every %dms", sbuf, CMNode.Cfg.nCyclesClock);
            CMNode.Topics.Pub.Clock.Pub  = node->advertise<rosgraph_msgs::Clock>(sbuf, CMNode.Cfg.QueuePub);


            /* ToDo: Necessary/Possible to ensure /clock is zeroed? */
            CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
            CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
        } else {
            LOG("  -> Don't provide simulation time!");
            CMNode.Cfg.nCyclesClock  = 0;
        }

        /* NavSatFix pub */
        strcpy(sbuf, "/sd_current_GPS");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.GPS.Pub           = node->advertise<sensor_msgs::NavSatFix>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.GPS.Job           = CMCRJob_Create("NavSatFix");

        /* LiDAR RSI pub */
        strcpy(sbuf, "/points_raw");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.LidarRSI.Pub      = node->advertise<sensor_msgs::PointCloud2>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.LidarRSI.Job      = CMCRJob_Create("LiDAR");

        /* RADAR RSI pub */
        strcpy(sbuf, "/radar_data");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.RadarRSI.Pub      = node->advertise<sensor_msgs::PointCloud2>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.RadarRSI.Job      = CMCRJob_Create("RADAR");

        /* Vehicle Velocity pub */
        strcpy(sbuf, "/current_velocity");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.Velocity.Pub      = node->advertise<geometry_msgs::TwistStamped>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.Velocity.Job      = CMCRJob_Create("Velocity");
        CMNode.Topics.Pub.Velocity.CycleTime = 1;
        CMNode.Topics.Pub.Velocity.CycleOffset = 0;

        /* Subscribe specific */
        CMNode.Cfg.QueueSub  = iGetIntOpt(Inf, "Node.QueueSub", 1); /* ToDo: Effect of queue length for subscriber? */

        /* Vehicle parameters */
        CMNode.Vhcl.MaxSteerAng = 600 * DEG_to_RAD;                 /* ToDo: This should be dynamic */

        /* SDControl sub */
        strcpy(sbuf, "/sd_control");
        LOG("  -> Subscribe '%s'", sbuf);
        CMNode.Topics.Sub.SDC.Sub = node->subscribe(sbuf, static_cast<uint>(CMNode.Cfg.QueueSub),cmnode_SDControl_CB_TpcIn);
        CMNode.Topics.Sub.SDC.Job = CMCRJob_Create("SDControl");
        CMNode.Topics.Sub.SDC.CycleTime = 1;
        CMNode.Topics.Sub.SDC.CycleOffset = 0;


        /* Services */


        /* Print general information after everything is done */
        LOG("Initialization of ROS Node finished!");
        LOG("  -> Node Name = '%s'", ros::this_node::getName().c_str());
        LOG("  -> Namespace = '%s'", ros::this_node::getNamespace().c_str());


        /* Advertised Topics */
        ros::this_node::getAdvertisedTopics(names);
        LOG("  -> Advertised Topics (%lu)", names.size());

        auto it = names.begin();
        for (; it != names.end(); ++it)
        LOG("    -> %s", (*it).c_str());


        /* Subscribed Topics */
        names.clear();
        ros::this_node::getSubscribedTopics(names);
        LOG("  -> Subscribed Topics (%lu)", names.size());
        it = names.begin();
        for (; it != names.end(); ++it)
        LOG("    -> %s",  (*it).c_str());

        return 1;
    }



    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Add user specific Quantities for data storage
    *   and visualization to DataDictionary
    * - Called once at program start
    * - no realtime conditions
    *
    */
    void
    CMRosIF_CMNode_DeclQuants (void)
    {

        tDDefault *df = DDefaultCreate("CMRosIF.");

        DDefULong   (df, "CycleNoRel",         "ms", &CMNode.CycleNoRel,               DVA_None);
        DDefInt     (df, "Sync.Cycles",        "-",  &CMNode.Sync.nCycles,             DVA_None);
        DDefDouble  (df, "Sync.Time",          "s",  &CMNode.Sync.Duration,            DVA_None);
        DDefInt     (df, "Sync.CyclePrepDone", "-",  &CMNode.Sync.CyclePrepDone,       DVA_None);
        DDefInt     (df, "Sync.CycleJobDone" , "-",  &CMNode.Sync.CycleJobDone,        DVA_None);
        DDefDouble4 (df, "Sync.SynthDelay",     "s", &CMNode.Sync.SynthDelay,          DVA_IO_In);

        DDefUChar   (df, "Cfg.Mode",           "-",  (unsigned char*)&CMNode.Cfg.Mode, DVA_None);
        DDefInt     (df, "Cfg.nCyclesClock",   "ms", &CMNode.Cfg.nCyclesClock,         DVA_None);
        DDefChar    (df, "Cfg.SyncMode",       "-",  (char*)&CMNode.Cfg.SyncMode,      DVA_None);
        DDefDouble4 (df, "Cfg.SyncTimeMax",    "s",  &CMNode.Cfg.SyncTimeMax,          DVA_IO_In);

        DDefInt     (df, "Mdl.CycleNo",        "-",  &CMNode.Model.CycleNo,            DVA_None);
        DDefInt     (df, "Mdl.CycleLastOut",   "ms", &CMNode.Model.CycleLastOut,       DVA_None);
        DDefInt     (df, "Mdl.CycleLastIn",    "ms", &CMNode.Model.CycleLastIn,        DVA_None);
        DDefInt     (df, "Mdl.CycleLastFlush", "ms", &CMNode.Model.CycleLastFlush,     DVA_None);

        DDefaultDelete(df);
    }



    /*!
    * Important:
    * - DO NOT CHANGE FUNCTION NAME !!!
    * - Automatically called by CMRosIF extension
    *
    * Description:
    * - Called when starting a new TestRun
    * - In separate Thread (no realtime conditions)
    * - After standard Infofiles are read in
    * - Return values
    *   - "rv <  0" = Error, TestRun start will be aborted
    *   - "rv >= 0" = Everything OK
    *
    * Arguments:
    * - Inf = CarMaker Infofile for CMRosIF with content after TestRun start
    *         - Please note that the Infofile provided at initialization might have been updated!
    *
    * ToDo:
    * - New Param_Get() function to read parameters from Infofile
    *
    */
    int
    CMRosIF_CMNode_TestRun_Start_atBegin (struct tInfos *Inf)
    {

        /* Node can be disabled via Infofile */
        tCMNode_Mode     *pmode     = &CMNode.Cfg.Mode;
        tCMNode_SyncMode *psyncmode = &CMNode.Cfg.SyncMode;

        if (Inf != NULL) {
            *pmode     =     (tCMNode_Mode)iGetIntOpt(Inf, "Node.Mode",      CMNode_Mode_Disabled);
            *psyncmode = (tCMNode_SyncMode)iGetIntOpt(Inf, "Node.Sync.Mode", CMNode_SyncMode_Disabled);
        }

        if (SimCore.CycleNo == 0 || Inf == NULL || *pmode == CMNode_Mode_Disabled) {
            *pmode = CMNode_Mode_Disabled;
            LOG("CarMaker ROS Node is disabled!");
            return 0;
        }

        char sbuf[512];
        char key[256];
        char *str               = NULL;
        int rv                  = 0;
        bool rvb                = false;

        int cycletime           = 0;
        int *pcycletime         = NULL;
        int cycleoff            = 0;
        tCMCRJob *job           = NULL;
        // auto srv                = &CMNode.Services.Init;

        tInfos *Inf_Vhcl = NULL;
        tInfos *Inf_Env = NULL;
        tErrorMsg *err;
        double pos[] = {0, 0, 0};


        /* Read the Vehicle Infofile */
        const char *FName_Vhcl;
        FName_Vhcl = InfoGetFilename(SimCore.Vhcl.Inf);      /* get the filename of the Infofile */
        Inf_Vhcl = InfoNew();
        iRead2(&err, Inf_Vhcl, FName_Vhcl, "Inf_Vhcl");      /* Create infofile handle */

        CMNode.Wheel.RL.pos                             = iGetFixedTable2(Inf_Vhcl, "Wheel.rl.pos", 3, 1);

        /* Global Navigation */
        CMNode.Sensor.GNav.pos                          = iGetFixedTableOpt2(Inf_Vhcl, "Sensor.GNav.pos", pos, 3, 1);
        CMNode.Sensor.GNav.UpdRate                      = 1000.0/iGetIntOpt(Inf_Vhcl, "Sensor.GNav.UpdRate", 10);       /* Convert from update rate in Hz to cycle time in ms */
        CMNode.Sensor.GNav.nCycleOffset                 = iGetIntOpt(Inf_Vhcl, "Sensor.GNav.nCycleOffset", 0);

        /* LiDAR RSI */
        CMNode.Sensor.LidarRSI.N                        = iGetIntOpt(Inf_Vhcl, "Sensor.LidarRSI.N", 0);
        CMNode.Sensor.LidarRSI.pos                      = iGetFixedTableOpt2(Inf_Vhcl, "Sensor.LidarRSI.0.pos", pos, 3, 1);
        CMNode.Sensor.LidarRSI.UpdRate                  = iGetIntOpt(Inf_Vhcl, "Sensor.LidarRSI.0.CycleTime", 10);
        CMNode.Sensor.LidarRSI.nCycleOffset             = iGetIntOpt(Inf_Vhcl, "Sensor.LidarRSI.0.nCycleOffset", 0);
        CMNode.Sensor.LidarRSI.nCycleOffset            *= iGetIntOpt(Inf_Vhcl, "Sensor.LidarRSI.CycleOffsetIgnore", 0); /* Set offset to 0 if ingnored in CarMaker */

        /* RADAR RSI */
        CMNode.Sensor.RadarRSI.N                        = iGetIntOpt(Inf_Vhcl, "Sensor.RadarRSI.N", 0);
        CMNode.Sensor.RadarRSI.pos                      = iGetFixedTableOpt2(Inf_Vhcl, "Sensor.RadarRSI.0.pos", pos, 3, 1);
        CMNode.Sensor.RadarRSI.UpdRate                  = iGetIntOpt(Inf_Vhcl, "Sensor.RadarRSI.0.CycleTime", 10);
        CMNode.Sensor.RadarRSI.nCycleOffset             = iGetIntOpt(Inf_Vhcl, "Sensor.RadarRSI.0.nCycleOffset", 0);
        CMNode.Sensor.RadarRSI.nCycleOffset            *= iGetIntOpt(Inf_Vhcl, "Sensor.RadarRSI.CycleOffsetIgnore", 0); /* Set offset to 0 if ingnored in CarMaker */
        CMNode.Sensor.RadarRSI.OutputType               = iGetIntOpt(Inf_Vhcl, "Sensor.RadarRSI.0.OutputType", 0);

        /* Read the Environmental Infofile */
        const char *FName_Env;
        FName_Env = InfoGetFilename(SimCore.TestRun.Inf);   /* get the filename of the Infofile */
        Inf_Env = InfoNew();
        iRead2(&err, Inf_Env, FName_Env, "Inf_Env");        /* Create infofile handle */

        CMNode.Sensor.GNav.Active  = iGetIntOpt(Inf_Env, "Env.GNav.Active", 0);

        LOG("CarMaker ROS Node is enabled! Mode=%d, SyncMode=%d", *pmode, *psyncmode);
        LOG("  -> Node Name = %s", ros::this_node::getName().c_str());

        /* Close all Infofile handles */
        InfoDelete(Inf_Vhcl);
        InfoDelete(Inf_Env);

        /* Update synchronization */
        if (*psyncmode != CMNode_SyncMode_Disabled && *psyncmode != CMNode_SyncMode_Tpc) {
            LogErrF(EC_Sim, "CMNode: Invalid synchronization mode '%d'!",*psyncmode);
            *pmode = CMNode_Mode_Disabled;
            return -1;
        }

        CMNode.Cfg.SyncTimeMax = iGetDblOpt(Inf, "Node.Sync.TimeMax", 1.0);


        /* Reset for next cycle */
        CMNode.CycleNoRel           =  0;
        CMNode.Sync.Duration        =  0.0;
        CMNode.Sync.nCycles         = -1;
        CMNode.Sync.CycleJobDone    = -1;
        CMNode.Sync.CyclePrepDone   = -1;
        CMNode.Model.CycleNo        = -1;
        CMNode.Model.CycleLastIn    = -1;
        CMNode.Model.CycleLastOut   = -1;
        CMNode.Model.CycleLastFlush = -1;


        /* Allow an update of the clock only if it was enabled before! */
        if (CMNode.Cfg.nCyclesClock > 0) {
            if ((rv = iGetIntOpt(Inf, "Node.nCyclesClock", 1000)) > 0)
            CMNode.Cfg.nCyclesClock = rv;
        }

        /* Necessary to ensure /clock is zeroed here?
        * ToDo: Create function? */
        if (CMNode.Cfg.nCyclesClock > 0) {
            LOG("  -> Publish /clock every %dms", CMNode.Cfg.nCyclesClock);
            CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
            CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
        }


        /* Prepare external node for next simulation */
        /* if (!srv->Clnt.exists()) {
        // ToDo: possible to get update if external ROS Node name changes?
        LogErrF(EC_Sim, "ROS Service is not ready! Please start external ROS Node providing Service '%s'!",
        srv->Clnt.getService().c_str());
        *pmode = CMNode_Mode_Disabled;
        return -1;
    }
    */

    LOG("  -> Send Service Request");

    /* ToDo: Async?*/
    /* if (!srv->Clnt.call(srv->Msg)) {
    LogErrF(EC_Sim, "ROS Service error!");
    *pmode = CMNode_Mode_Disabled;
    return -1;
}
*/

/* Update cycle time with information of external node */

// #if 1
/* Variant 1:
* - Receiving parameters via ROS Parameter Server
* - Parameter may be set externally e.g. by other node or arguments to command
* - ROS parameters are more flexible than ROS services!
*/
/*     strcpy(sbuf, hellocm::prm_cycletime_name.c_str());
if ((rv = CMNode.Cfg.Ros.Node->hasParam(sbuf)) == true)
CMNode.Cfg.Ros.Node->getParam(sbuf, rv);
*/
// #else
/* Variant 2:
* - Receiving parameters from external Node via Service
* - Services might be too "static"
* - Not recommended!
*/
/*     rv = srv->Msg.response.cycletime;
#endif

pcycletime = &CMNode.Topics.Sub.Ext2CM.CycleTime;

if (*pcycletime != rv) {
LOG("  -> Cycle time of external node changed from %dms to %dms", *pcycletime, rv);
*pcycletime = rv;
}
*/

/* Plausibility check for Cycle Time */
/*     if (CMNode.Cfg.nCyclesClock > 0 && (*pcycletime < CMNode.Cfg.nCyclesClock
|| *pcycletime%CMNode.Cfg.nCyclesClock != 0)) {

LogErrF(EC_Sim, "Ext. ROS Node has invalid cycle time! Expected multiple of %dms but got %dms",
CMNode.Cfg.nCyclesClock, *pcycletime);

*pmode = CMNode_Mode_Disabled;
return -1;
}
*/



/* Prepare Jobs for publish and subscribe
* - Special use case:
*   - Topic in and Topic out use same cycle time with relative shift!
*/

/* SDControl sub job */
job         = CMNode.Topics.Sub.SDC.Job;
cycletime   = CMNode.Topics.Sub.SDC.CycleTime;
cycleoff    = CMNode.Topics.Sub.SDC.CycleOffset;
CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);

/* GPS pub job */
job         = CMNode.Topics.Pub.GPS.Job;
cycletime   = CMNode.Sensor.GNav.UpdRate;
cycleoff    = CMNode.Sensor.GNav.nCycleOffset;
CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);

/* LiDAR RSI pub job */
job         = CMNode.Topics.Pub.LidarRSI.Job;
cycletime   = CMNode.Sensor.LidarRSI.UpdRate;
cycleoff    = CMNode.Sensor.LidarRSI.nCycleOffset;
CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);

/* RADAR RSI pub job */
job         = CMNode.Topics.Pub.RadarRSI.Job;
cycletime   = CMNode.Sensor.RadarRSI.UpdRate;
cycleoff    = CMNode.Sensor.RadarRSI.nCycleOffset;
CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);

/* Vehicle Velocity sub job */
job         = CMNode.Topics.Pub.Velocity.Job;
cycletime   = CMNode.Topics.Pub.Velocity.CycleTime;
cycleoff    = CMNode.Topics.Pub.Velocity.CycleOffset;
CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);

/* Synchronization with external node
* - external node provides cycle time (see service above)
* - other parameterization methods (e.g. ROS parameter, ...) are possible!
* - Expect sync Topic are delayed (communication time, ...)
* - This example shows sync via ROS Timer in external node
*   - Therefore "/clock" topic needs to be published by CarMaker!
*   - Other mechanism, e.g. data triggered on external node side
*     via publishing Topic directly inside subscription callback is also possible!
* - time=0.0 can't be detected by external node, therefore
*   first receive needs to start after expected cycle time
*   of external ROS node
*/


/* Create the synchronization jobs */
if (*psyncmode == CMNode_SyncMode_Tpc) {
    CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Ext);

    LOG("  -> Synchronize on Topic '%s' (cycletime=%d, cycleoff=%d)",
    CMNode.Topics.Sub.SDC.Sub.getTopic().c_str(), cycletime, cycleoff);

} else
CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Default);



LOG("External ROS Node is ready to simulate");

return 1;
}



/*!
* ToDo:
* - Put everything to TestRun_Start_atBegin?
*
* Important:
* - DO NOT CHANGE FUNCTION NAME !!!
* - Automatically called by CMRosIF extension
*
* Description:
* - Repeating call for several CarMaker cycles until return value is 1
* - May be called even previous return value was 1
* - See "User.c:User_TestRun_RampUp()"
*
*/
int
CMRosIF_CMNode_TestRun_RampUp (void)
{

    /* Return immediately if node is disabled */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled) {
        return 1;
    }

    /* Put your code here */
    //if (NotReady) return 0;


    return 1;
}



/*!
* Important:
* - DO NOT CHANGE FUNCTION NAME !!!
* - Automatically called by CMRosIF extension
*
* Description:
* - Called when TestRun ends (no realtime conditions)
* - See "User.c:User_TestRun_End()"
*
*/
int
CMRosIF_CMNode_TestRun_End (void)
{


    /* Put your code here */

    /* Disable after simulation has finished */
    CMNode.Cfg.Mode = CMNode_Mode_Disabled;

    return 1;
}



/*!
* Important:
* - DO NOT CHANGE FUNCTION NAME !!!
* - Automatically called by CMRosIF extension
*
* Description:
* - Called at very beginning of CarMaker cycle
* - Process all topics/services using different modes or synchronization modes
* - See "User.c:User_In()"
*
* ToDo:
* - Additional spin mechanism
*   - e.g. for HIL
*   - e.g. spinning in new thread, copying incoming data here, ...
*
*/
int
CMRosIF_CMNode_In (void)
{

    int rv                   = 0;
    int rx_done              = 0;
    const char *job_name     = NULL;
    tCMCRJob *job            = NULL;
    ros::WallTime tStart     = ros::WallTime::now();
    ros::WallDuration tDelta = ros::WallDuration(0.0);
    CMNode.Sync.nCycles      = 0;
    CMNode.Sync.Duration     = 0.0;

    switch (CMNode.Cfg.Mode) {
        case CMNode_Mode_Disabled:
        /* Comment next line if no messages/services
        * shall be processed in disabled Node state
        */
        ros::spinOnce();
        break;

        case CMNode_Mode_Default:

        if (CMNode.Cfg.SyncMode != CMNode_SyncMode_Tpc) {
            /* Process messages in queue, but do not block */
            ros::spinOnce();

        } else {
            /* Synchronization based on expected Topics
            * - Blocking call (process publish and wait for answer)
            * - Stop simulation if maximum time is exceeded
            */
            do {
                ros::spinOnce();

                /* Only do anything if simulation is running */
                if (SimCore.State != SCState_Simulate) {
                    rx_done = 1;
                    break;
                }

                rx_done = 0;

                /* Check all jobs if preparation is done */
                job      = CMNode.Topics.Sub.SDC.Job;

                if ((rv = CMCRJob_DoPrep(job, CMNode.CycleNoRel, 0, NULL, NULL)) < CMCRJob_RV_OK) {
                    LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s",CMCRJob_GetName(job), CMCRJob_RVStr(rv));
                    rx_done = 0;
                    break;
                }

                /* If job is not done, remember name and prevent loop to finish */
                job_name = (rv != CMCRJob_RV_DoSomething ? NULL : CMCRJob_GetName(job));
                rx_done  = rv == CMCRJob_RV_DoNothing ? 1 : 0;

                if (rx_done == 1)
                break;

                /* Wait a little that data can arrive. WallTime, NOT ROS time!!!*/
                ros::WallDuration(0.0).sleep();
                tDelta = ros::WallTime::now() - tStart;
                CMNode.Sync.nCycles++;

            } while (ros::ok() && rx_done == 0 && tDelta.toSec() < CMNode.Cfg.SyncTimeMax);

            /* Final calculation to get duration including last cycle before receive */
            tDelta = ros::WallTime::now() - tStart;

            CMNode.Sync.Duration = tDelta.toSec();

            if (rx_done != 1 && CMNode.Cfg.SyncTimeMax > 0.0 && tDelta.toSec() >= CMNode.Cfg.SyncTimeMax)
            LogErrF(EC_Sim, "CMNode: Synchronization error! tDelta=%.3f, Last invalid Job='%s'\n", tDelta.toSec(), job_name);
        }

        break;

        case CMNode_Mode_Threaded:
        /* ToDo
        * - Spinning in parallel thread started before
        * - Lock variables!
        * - e.g. for HIL
        */
        break;

        default:
        /* Invalid!!! */;
    }

    return 1;
}



/*!
* Important:
* - DO NOT CHANGE FUNCTION NAME !!!
* - Automatically called by CMRosIF extension
*
* Description:
* - Called after driving maneuver calculation
* - before CMRosIF_CMNode_VehicleControl_Calc()
* - See "User.c:User_DrivManCalc()"
*/
int
CMRosIF_CMNode_DrivMan_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate) {
        return 0;
    }

    /* Put your code here */

    return 1;
}



/*!
* Important:
* - DO NOT CHANGE FUNCTION NAME !!!
* - Automatically called by CMRosIF extension
*
* Description:
* - Called after CMRosIF_CMNode_DrivManCalc
* - before CMRosIF_CMNode_VehicleControl_Calc()
* - See "User.c:User_VehicleControl_Calc()"
*/
int
CMRosIF_CMNode_VehicleControl_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate) {
        return 0;
    }

    int rv;
    auto sub = &CMNode.Topics.Sub.SDC;
    double steer_ang, trq_demand;   /* Intermediate values from SDControl */
    double gas, brake;              /* VehicleControl pedal values */

    if ((rv = CMCRJob_DoJob(sub->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
    && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sub->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
        /* Translate torque demand to pedal positions */
        trq_demand = CMNode.Topics.Sub.SDC.Msg.torque/100.0;

        if (trq_demand >= 0) {
            gas = trq_demand;
            brake = 0.0;
        } else {
            gas = 0.0;
            brake = -trq_demand;
        }

        /* Calculate the steering angle from the current steering input */
        steer_ang = CMNode.Topics.Sub.SDC.Msg.steer/100.0*CMNode.Vhcl.MaxSteerAng;

        VehicleControl.Gas              = gas;
        VehicleControl.Brake            = brake;
        VehicleControl.Steering.Ang     = steer_ang;

        /* Remember cycle for debugging */
        CMNode.Sync.CycleJobDone    = CMNode.CycleNoRel;
        CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;

    }

    /* Put your code here */
    return 1;
}



/*!
* Important:
* - DO NOT CHANGE FUNCTION NAME !!!
* - Automatically called by CMRosIF extension
*
* Description:
* - Called after vehicle model has been calculated
* - See "User.c:User_Calc()"
*/
int
CMRosIF_CMNode_Calc (double dt)
{
    int rv;

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate) {
        return 0;
    }

    /* Publish GPS data from CarMaker */
    if (CMNode.Sensor.GNav.Active) {
        if ((rv = CMCRJob_DoJob(CMNode.Topics.Pub.GPS.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.GPS.Job), CMCRJob_RVStr(rv));
        } else {
            CMNode.Topics.Pub.GPS.Msg.header.frame_id = "Fr0";
            CMNode.Topics.Pub.GPS.Msg.header.stamp = ros::Time(SimCore.Time);

            CMNode.Topics.Pub.GPS.Msg.status.status = 0;    /* Augmentation fix not available in CarMaker */
            CMNode.Topics.Pub.GPS.Msg.status.service = 1;   /* CarMaker simulates GPS only */

            /* Exact receiver position, could be replaced by Pseudorange if errors are to be simulated */
            CMNode.Topics.Pub.GPS.Msg.latitude = GNavSensor.Receiver.UserPosLlhTsa[0];
            CMNode.Topics.Pub.GPS.Msg.longitude = GNavSensor.Receiver.UserPosLlhTsa[1];
            CMNode.Topics.Pub.GPS.Msg.altitude = GNavSensor.Receiver.UserPosLlhTsa[2];

            /* Use ideal covariance matrix with the exact receiver position */
            int ideal_cov[] = {1,0,0,0,1,0,0,0,1};
            for (int i = 0; i < 9; i++) {
                CMNode.Topics.Pub.GPS.Msg.position_covariance[i] = ideal_cov[i];
            }
            CMNode.Topics.Pub.GPS.Msg.position_covariance_type = 2; /* Assume this is a covariance matrix of known diagonal */
        }
    }

    /* Publish LiDAR RSI data from CarMaker */
    if (CMNode.Sensor.LidarRSI.N > 0) {
        if ((rv = CMCRJob_DoJob(CMNode.Topics.Pub.LidarRSI.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.LidarRSI.Job), CMCRJob_RVStr(rv));
        } else {

            sensor_msgs::PointField field;

            /* Frame header */
            CMNode.Topics.Pub.LidarRSI.Msg.header.frame_id = "Fr0";
            CMNode.Topics.Pub.LidarRSI.Msg.header.stamp = ros::Time(LidarRSI[0].ScanTime);

            /* Unordered cloud with the number of scanned points of the LiDAR */
            CMNode.Topics.Pub.LidarRSI.Msg.height = 1;
            CMNode.Topics.Pub.LidarRSI.Msg.width = LidarRSI[0].nScanPoints;

            /* Field: BeamID */
            field.name = "beam_id";
            field.offset = 0;
            field.datatype = 5;              /* int32 */
            field.count = 1;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: EchoID */
            field.name = "echo_id";
            field.offset = 4;
            field.datatype = 5;              /* int32 */
            field.count = 1;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: Time of flight (ns) */
            field.name = "time_of";
            field.offset = 8;
            field.datatype = 8;              /* double */
            field.count = 1;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: Length of flight (m) */
            field.name = "length_of";
            field.offset = 16;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: Origin.x of reflection */
            field.name = "x";
            field.offset = 24;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: Origin.y of reflection */
            field.name = "y";
            field.offset = 32;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: Origin.z of reflection */
            field.name = "z";
            field.offset = 40;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: Intensity (nW) */
            field.name = "intensity";
            field.offset = 48;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: Pulse width (ns) */
            field.name = "pulse_width";
            field.offset = 56;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            /* Field: Number of reflections */
            field.name = "reflections";
            field.offset = 64;
            field.datatype = 5;              /* int32 */
            field.count = 1;
            CMNode.Topics.Pub.LidarRSI.Msg.fields.push_back(field);

            int point_step = 68;
            CMNode.Topics.Pub.LidarRSI.Msg.is_bigendian = 1;
            CMNode.Topics.Pub.LidarRSI.Msg.point_step = point_step;
            CMNode.Topics.Pub.LidarRSI.Msg.row_step = point_step * LidarRSI[0].nScanPoints;
            CMNode.Topics.Pub.LidarRSI.Msg.is_dense = 1;

            /* Create the output data binary blob */
            int data_idx;
            uint8_t data[CMNode.Topics.Pub.LidarRSI.Msg.row_step];
            for (int ii = 0; ii < LidarRSI[0].nScanPoints; ii++) {
                data_idx = ii*point_step;
                uint8_t* b = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].BeamID);
                uint8_t* e = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].EchoID);
                uint8_t* t = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].TimeOF);
                uint8_t* l = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].LengthOF);
                uint8_t* x = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].Origin[0]);
                uint8_t* y = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].Origin[1]);
                uint8_t* z = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].Origin[2]);
                uint8_t* i = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].Intensity);
                uint8_t* p = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].PulseWidth);
                uint8_t* r = reinterpret_cast<uint8_t*>(&LidarRSI[0].ScanPoint[ii].nRefl);
                std::copy(b, b+4, data+data_idx+0);
                std::copy(e, e+4, data+data_idx+4);
                std::copy(t, t+8, data+data_idx+8);
                std::copy(l, l+8, data+data_idx+16);
                std::copy(x, x+8, data+data_idx+24);
                std::copy(y, y+8, data+data_idx+32);
                std::copy(z, z+8, data+data_idx+40);
                std::copy(i, i+8, data+data_idx+48);
                std::copy(p, p+8, data+data_idx+56);
                std::copy(r, r+4, data+data_idx+64);
            }

            /* Add the binary data to the message */
            for (int ii = 0; ii < CMNode.Topics.Pub.LidarRSI.Msg.row_step; ii++) {
               CMNode.Topics.Pub.LidarRSI.Msg.data.push_back(data[ii]);
            }
        }
    }

    /* Publish RADAR RSI data from CarMaker */
    if (CMNode.Sensor.RadarRSI.N > 0) {
        if ((rv = CMCRJob_DoJob(CMNode.Topics.Pub.RadarRSI.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.RadarRSI.Job), CMCRJob_RVStr(rv));
        } else {

            sensor_msgs::PointField field;

            /* Frame header */
            CMNode.Topics.Pub.RadarRSI.Msg.header.frame_id = "FrRadar";
            CMNode.Topics.Pub.RadarRSI.Msg.header.stamp = ros::Time(RadarRSI[0].TimeFired);

            /* Unordered cloud with the number of scanned points of the LiDAR */
            CMNode.Topics.Pub.RadarRSI.Msg.height = 1;
            CMNode.Topics.Pub.RadarRSI.Msg.width = RadarRSI[0].nDetections;

            /* Field: x */
            field.name = "x";
            field.offset = 0;
            field.datatype = 8;              /* double */
            field.count = 1;
            CMNode.Topics.Pub.RadarRSI.Msg.fields.push_back(field);

            /* Field: y */
            field.name = "y";
            field.offset = 8;
            CMNode.Topics.Pub.RadarRSI.Msg.fields.push_back(field);

            /* Field: z */
            field.name = "z";
            field.offset = 16;
            CMNode.Topics.Pub.RadarRSI.Msg.fields.push_back(field);

            /* Field: velocity (m/s) */
            field.name = "velocity";
            field.offset = 24;
            CMNode.Topics.Pub.RadarRSI.Msg.fields.push_back(field);

            /* Field: power (dBm) */
            field.name = "power";
            field.offset = 32;
            CMNode.Topics.Pub.RadarRSI.Msg.fields.push_back(field);

            int point_step = 40;
            CMNode.Topics.Pub.RadarRSI.Msg.is_bigendian = 1;
            CMNode.Topics.Pub.RadarRSI.Msg.point_step = point_step;
            CMNode.Topics.Pub.RadarRSI.Msg.row_step = point_step * RadarRSI[0].nDetections;
            CMNode.Topics.Pub.RadarRSI.Msg.is_dense = 1;

            /* Create the output data binary blob */
            int data_idx;
            double x_f, y_f, z_f;
            double distance, azimuth, elevation;
            uint8_t data[CMNode.Topics.Pub.RadarRSI.Msg.row_step];
            for (int ii = 0; ii < RadarRSI[0].nDetections; ii++) {
                data_idx = ii*point_step;

                switch(CMNode.Sensor.RadarRSI.OutputType) {
                    case 1:
                        x_f = RadarRSI[0].DetPoints[ii].Coordinates[0];
                        y_f = RadarRSI[0].DetPoints[ii].Coordinates[1];
                        z_f = RadarRSI[0].DetPoints[ii].Coordinates[2];
                        break;
                    case 2:
                        distance    = RadarRSI[0].DetPoints[ii].Coordinates[0];
                        azimuth     = RadarRSI[0].DetPoints[ii].Coordinates[1];
                        elevation   = RadarRSI[0].DetPoints[ii].Coordinates[2];     // note that this is elevation, NOT inclination
                        x_f         = distance*cos(elevation)*cos(azimuth);
                        y_f         = distance*cos(elevation)*sin(azimuth);
                        z_f         = distance*sin(elevation);                      // sin(elevation) == cos(inclination)
                        break;
                    case 3:
                        LogErrF(EC_Sim, "CMNode: VRx RADAR RSI receivers not supported.");
                }
                uint8_t* x = reinterpret_cast<uint8_t*>(&x_f);
                uint8_t* y = reinterpret_cast<uint8_t*>(&y_f);
                uint8_t* z = reinterpret_cast<uint8_t*>(&z_f);
                uint8_t* v = reinterpret_cast<uint8_t*>(&RadarRSI[0].DetPoints[ii].Velocity);
                uint8_t* p = reinterpret_cast<uint8_t*>(&RadarRSI[0].DetPoints[ii].Power);
                std::copy(x, x+8, data+data_idx+0);
                std::copy(y, y+8, data+data_idx+8);
                std::copy(z, z+8, data+data_idx+16);
                std::copy(v, v+8, data+data_idx+24);
                std::copy(p, p+8, data+data_idx+32);
            }

            /* Add the binary data to the message */
            for (int ii = 0; ii < CMNode.Topics.Pub.RadarRSI.Msg.row_step; ii++) {
               CMNode.Topics.Pub.RadarRSI.Msg.data.push_back(data[ii]);
            }
        }
    }

    /* Publish vehicle velocity data from CarMaker */
    if ((rv = CMCRJob_DoJob(CMNode.Topics.Pub.Velocity.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
        LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Velocity.Job), CMCRJob_RVStr(rv));
    } else {
        CMNode.Topics.Pub.Velocity.Msg.header.frame_id = "Fr1";
        CMNode.Topics.Pub.Velocity.Msg.header.stamp = ros::Time(SimCore.Time);

        /* Publish linear velocity only in the X direction */
        CMNode.Topics.Pub.Velocity.Msg.twist.linear.x = Vehicle.v;
        CMNode.Topics.Pub.Velocity.Msg.twist.linear.y = 0;
        CMNode.Topics.Pub.Velocity.Msg.twist.linear.z = 0;
        CMNode.Topics.Pub.Velocity.Msg.twist.angular.x = 0;
        CMNode.Topics.Pub.Velocity.Msg.twist.angular.y = 0;
        CMNode.Topics.Pub.Velocity.Msg.twist.angular.z = 0;
    }

    /* Remember cycle for debugging */
    CMNode.Sync.CycleJobDone    = CMNode.CycleNoRel;
    CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;

    /* Put your code here
    * - Update model parameters here?
    * - Do some calculation...
    */

    /* Update model with values from external node only in specific cycle?
    * - This data handling is optionl, but necessary for deterministic behaviour
    * - if synchronization is active, incoming data remains in msg buffer until correct cycle
    */
    /*
    int rv;
    auto sync = &CMNode.Topics.Sub.Ext2CM;

    if ((rv = CMCRJob_DoJob(sync->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
    && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
    */
        /* Something to do in sync cycle? */
        //CMCRJob_Info(in->Job, CMNode.CycleNoRel, "CMNode: Do Something for Sync: ");

        /* Update model parameters here? *
        CMNode.Model.CycleNo = CMNode.Topics.Sub.Ext2CM.Msg.cycleno;


        * Remember cycle for debugging *
        CMNode.Sync.CycleJobDone    = CMNode.CycleNoRel;
        CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;
    }
    */

    /* Do some calculation... */

    return 1;
}



/*!
* Important:
* - DO NOT CHANGE FUNCTION NAME !!!
* - Automatically called by CMRosIF extension
*
* Description:
* - Called close to end of CarMaker cycle
* - See "User.c:User_Out()"
*/
int
CMRosIF_CMNode_Out (void)
{
    ros::WallTime wtime = ros::WallTime::now();

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate) {
        return 0;
    }

    int rv;

    /* Publish SatNav messages */
    if (CMNode.Sensor.GNav.Active) {
        auto out_gps = &CMNode.Topics.Pub.GPS;

        if ((rv = CMCRJob_DoJob(out_gps->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_gps->Job), CMCRJob_RVStr(rv));
        } else if (rv == CMCRJob_RV_DoSomething) {

            /* Publish message to output */
            out_gps->Pub.publish(out_gps->Msg);

            /* Remember cycle for debugging */
            CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
        }
    }

    /* Publish LiDAR PointCooud2 messages */
    auto out_lidar = &CMNode.Topics.Pub.LidarRSI;

    if ((rv = CMCRJob_DoJob(out_lidar->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_lidar->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {

        /* Publish message to output */
        out_lidar->Pub.publish(out_lidar->Msg);

        /* Remember cycle for debugging */
        CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }

    /* Publish RADAR PointCooud2 messages */
    auto out_radar = &CMNode.Topics.Pub.RadarRSI;

    if ((rv = CMCRJob_DoJob(out_radar->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_radar->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {

        /* Publish message to output */
        out_radar->Pub.publish(out_radar->Msg);

        /* Remember cycle for debugging */
        CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }

    /* Publish vehicle velocity messages */
    auto out_vel = &CMNode.Topics.Pub.Velocity;

    if ((rv = CMCRJob_DoJob(out_vel->Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
        LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_vel->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {

        /* Publish message to output */
        out_vel->Pub.publish(out_vel->Msg);

        /* Remember cycle for debugging */
        CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }


    // auto out = &CMNode.Topics.Pub.CM2Ext;
    //
    // /* Communicate to External ROS Node in this cycle?
    // * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
    // */
    // if ((rv = CMCRJob_DoJob(out->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
    // && rv != CMCRJob_RV_DoSomething) {
    //     LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out->Job), CMCRJob_RVStr(rv));
    // } else if (rv == CMCRJob_RV_DoSomething) {
    //
    //     out->Msg.cycleno      = CMNode.CycleNoRel;
    //     out->Msg.time         = ros::Time(SimCore.Time);
    //     out->Msg.synthdelay   = CMNode.Sync.SynthDelay;
    //
    //     /* Header stamp and frame needs to be set manually! */
    //
    //     /* provide system time close to data is sent */
    //     wtime = ros::WallTime::now();
    //     out->Msg.header.stamp.sec  = wtime.sec;
    //     out->Msg.header.stamp.nsec = wtime.nsec;
    //
    //     out->Pub.publish(out->Msg);
    //
    //     /* Remember cycle for debugging */
    //     CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    // }


    /* Publish "/clock" topic after all other other topics are published
    * - Is the order of arrival in other node identical? */
    if (CMNode.Cfg.nCyclesClock > 0 && CMNode.CycleNoRel%CMNode.Cfg.nCyclesClock == 0) {
        CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(SimCore.Time);
        CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    }

    /* ToDo: When increase? */
    CMNode.CycleNoRel++;

    return 1;
}



/*!
* Important:
* - DO NOT CHANGE FUNCTION NAME !!!
* - Automatically called by CMRosIF extension
*
* Description:
* - Called one Time when CarMaker ends
* - See "User.c:User_End()"
*/
int
CMRosIF_CMNode_End (void)
{

    LOG("%s: End", __func__);

    if (ros::isInitialized()) {

        /* Needs to be called before program exists, otherwise
        * "boost" error due to shared library and default deconstructor */
        CMNode.Cfg.Ros.Node->shutdown();

        /* ToDo:
        * - Blocking call? Wait until shutdown has finished?
        * - Optional? */
        ros::shutdown();
    }

    return 1;
}



/*!
* Important:
* - NOT automatically called by CMRosIF extension
*
* Description:
* - Example of user generated function
* - Can be accessed in other sources, e.g. User.c
* - Use "CMRosIF_GetSymbol()" to get symbol (see "lib/CMRosIF.h")
*
*/
int
CMRosIF_CMNode_MyFunc (char *LogMsg)
{

    LOG("%s: %s",  __func__, LogMsg);
    return 1;
}

#ifdef __cplusplus
}
#endif
