/*!
******************************************************************************
**  CarMaker - Version 10.0
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*
* Description:
* - CarMaker ROS node adapted to work with the Project ASLAN autonomous vehicle stack
* - Structure inherited from basic CarMaker ROS node example in the IPG Client Area FAQ
* - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
* - Basic communication without parameterizable synchronization
*/

#include <stdio.h>
#include <string.h>

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
#include "Car/Car.h"
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
#include "rsds-client-camera.h"         /* RSDS Camera RSI client */

/* ROS */
#include "sensor_msgs/PointCloud2.h"                    /* ROS PointCloud2 for sensor inputs */
#include "sensor_msgs/Image.h"                          /* ROS Image for camera sensor inputs */
#include "sensor_msgs/CameraInfo.h"                     /* ROS CameraInfo for camera metadata */
#include "sensor_msgs/NavSatFix.h"                      /* ROS Navigation Satellite fix */
#include "geometry_msgs/TwistStamped.h"                 /* ROS Twist command */
#include "tf2/LinearMath/Quaternion.h"                  /* Ros TF2 quaternion */
#include "tf2_ros/transform_broadcaster.h"              /* Publish TF2 transforms */

/* ASLAN */
#include "sd_vehicle_interface/sd_vehicle_interface.h"  /* ASLAN SD Vehicle interface */

/*! String and numerical version of this Node
*  - String:    e.g. <Major>.<Minor>.<Patch>
*  - Numerical: e.g. <nDigitsMajor><2DigitsMinor><2DigitsPatch>
*/
#define CMNODE_VERSION "0.10.0"
#define CMNODE_NUMVER  1000


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


/* Global struct for this Node */
static struct {
    unsigned long       CycleNoRel;     /*!< CarMaker relative cycle number, e.g. since start of TestRun */

    struct {
        int             CycleNo;        /*!< Cycle number of external ROS Node (only for information) */

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
            tRosIF_TpcPub<sensor_msgs::Image> CameraStereoL;
            tRosIF_TpcPub<sensor_msgs::Image> CameraStereoR;
            tRosIF_TpcPub<sensor_msgs::CameraInfo> CameraInfoL;
            tRosIF_TpcPub<sensor_msgs::CameraInfo> CameraInfoR;
            tRosIF_TpcPub<sensor_msgs::Image> CameraRGB;
            tRosIF_TpcPub<sensor_msgs::CameraInfo> CameraInfoRGB;
            tRosIF_TpcPub<sensor_msgs::Image> CameraMono;
            tRosIF_TpcPub<sensor_msgs::CameraInfo> CameraMonoInfo;
            tRosIF_TpcPub<geometry_msgs::TwistStamped> Velocity;

            /*!< CarMaker can be working as ROS Time Server providing simulation time
            *   starting at 0 for each TestRun */
            tRosIF_TpcPub<rosgraph_msgs::Clock> Clock;
        } Pub; /*!< Topics to be published */
    } Topics; /*!< ROS Topics used by this Node */

    struct {
    } Services; /*!< ROS Services used by this Node (client and server)*/

    struct {
        int                 QueuePub;           /*!< Queue size for Publishers */
        int                 QueueSub;           /*!< Queue size for Subscribers */
        int                 nCyclesClock;       /*!< Number of cycles publishing /clock topic.
        CycleTime should be multiple of this value */
        tCMNode_Mode        Mode;

        tRosIF_Cfg          Ros;
    } Cfg; /*!< General configuration for this Node */

    struct {
        int                 MaxSteerAng;        /*!< Maximum steering angle of the vehicle */
        geometry_msgs::TransformStamped TF;     /*!< ROS Reference frame */
    } Vhcl; /*!< Vehicle parameters */

    struct {
        struct {
            double*          pos;               /*!< Mounting position on vehicle frame */
        } RL;
    } Wheel; /*!< Wheel parameters */

    struct {
        struct {
            int             Active;             /*!< Presence of active GNav sensors */
            double*         pos;                /*!< Mounting position on vehicle frame */
            double*         rot;                /*!< Mounting rotation on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;
        } GNav;

        struct {
            int             Active;                 /*!< Presence of active LidarRSI sensors */
            double*         pos;                    /*!< Mounting position on vehicle frame */
            double*         rot;                    /*!< Mounting rotation on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;
            char*           BeamFName;              /*!< Filename for beam configuration */
            int             nBeams;                 /*!< Total number of scan beams */
            double*         Beams;                  /*!< Table of scan beams information */
            geometry_msgs::TransformStamped TF;     /*!< ROS Reference frame */
        } LidarRSI;

        struct {
            int             Active;                 /*!< Presence of active RadarRSI sensors */
            double*         pos;                    /*!< Mounting position on vehicle frame */
            double*         rot;                    /*!< Mounting rotation on vehicle frame */
            int             UpdRate;
            int             nCycleOffset;
            int             OutputType;             /*!< 0:Cartesian 1:Spherical 2:VRx */
            geometry_msgs::TransformStamped TF;     /*!< ROS Reference frame */
        } RadarRSI;

        struct {
            int             Active;                 /*!< Presence of active CameraRSI sensors */
            double*         pos;                    /*!< Mounting position on vehicle frame */
            double*         rot;                    /*!< Mounting rotation on vehicle frame */
            int             nCycleOffset;
            int             UpdRate;
            char*           OutputFormat;           /*!< Format of the output image data */
            int             Channel;                /*!< RSDS channel where the camera data occurs */
            char*           Name;                   /*!< User-defined name for the sensor */
            geometry_msgs::TransformStamped TF;     /*!< ROS Reference frame */
        } CameraStereoL;

        struct {
            int             Active;                 /*!< Presence of active CameraRSI sensors */
            double*         pos;                    /*!< Mounting position on vehicle frame */
            double*         rot;                    /*!< Mounting rotation on vehicle frame */
            int             nCycleOffset;
            int             UpdRate;
            char*           OutputFormat;           /*!< Format of the output image data */
            int             Channel;                /*!< RSDS channel where the camera data occurs */
            char*           Name;                   /*!< User-defined name for the sensor */
            geometry_msgs::TransformStamped TF;     /*!< ROS Reference frame */
        } CameraStereoR;

        struct {
            int             Active;                 /*!< Presence of active CameraRSI sensors */
            double*         pos;                    /*!< Mounting position on vehicle frame */
            double*         rot;                    /*!< Mounting rotation on vehicle frame */
            int             nCycleOffset;
            int             UpdRate;
            char*           OutputFormat;           /*!< Format of the output image data */
            int             Channel;                /*!< RSDS channel where the camera data occurs */
            char*           Name;                   /*!< User-defined name for the sensor */
            geometry_msgs::TransformStamped TF;     /*!< ROS Reference frame */
        } CameraMono;
    } Sensor; /*!< Sensor parameters */

    struct {
        tf2_ros::TransformBroadcaster *TF_br;       /*!< ROS transform broadcaster */
        geometry_msgs::TransformStamped TF;         /*!< ROS Reference frame */
    } Global; /*!< Global simulation properties */

} CMNode;

/*!
* Description:
* - Callback for ROS Topics published by external ROS Nodes
*/

/* Subscriber Callback for SDControl from ASLAN */
static void
cmnode_SDControl_CB_TpcIn(const aslan_msgs::SDControl::ConstPtr &msg) {

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
        strcpy(sbuf, "/target_list_cartesian");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.RadarRSI.Pub      = node->advertise<sensor_msgs::PointCloud2>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.RadarRSI.Job      = CMCRJob_Create("RADAR");

        /* Stereo Camera RSI Left pub */
        strcpy(sbuf, "/zed/zed_node/left/image_rect_color");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CameraStereoL.Pub = node->advertise<sensor_msgs::Image>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.CameraStereoL.Job = CMCRJob_Create("CAMERA_L");

        /* Stereo Camera RSI Left Info pub */
        strcpy(sbuf, "/zed/zed_node/left/camera_info");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CameraInfoL.Pub   = node->advertise<sensor_msgs::CameraInfo>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.CameraInfoL.Job   = CMCRJob_Create("CAMERA_INFO_L");

        /* Stereo Camera RSI Right pub */
        strcpy(sbuf, "/zed/zed_node/right/image_rect_color");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CameraStereoR.Pub = node->advertise<sensor_msgs::Image>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.CameraStereoR.Job = CMCRJob_Create("CAMERA_R");

        /* Stereo Camera RSI Right Info pub */
        strcpy(sbuf, "/zed/zed_node/right/camera_info");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CameraInfoR.Pub   = node->advertise<sensor_msgs::CameraInfo>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.CameraInfoR.Job   = CMCRJob_Create("CAMERA_INFO_R");

        /* Stereo Camera RSI RGB pub */
        strcpy(sbuf, "/zed/zed_node/rgb/image_rect_color");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CameraRGB.Pub     = node->advertise<sensor_msgs::Image>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.CameraRGB.Job     = CMCRJob_Create("CAMERA_RGB");

        /* Stereo Camera RSI RGB Info pub */
        strcpy(sbuf, "/zed/zed_node/rgb/camera_info");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CameraInfoRGB.Pub = node->advertise<sensor_msgs::CameraInfo>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.CameraInfoRGB.Job = CMCRJob_Create("CAMERA_INFO_RGB");

        /* Mono Camera RSI pub */
        strcpy(sbuf, "/pylon_camera/image_rect");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CameraMono.Pub    = node->advertise<sensor_msgs::Image>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.CameraMono.Job    = CMCRJob_Create("CAMERA_M");

        /* Mono Camera RSI Info pub */
        strcpy(sbuf, "/pylon_camera/camera_info");
        LOG("  -> Publish '%s'", sbuf);
        CMNode.Topics.Pub.CameraMonoInfo.Pub = node->advertise<sensor_msgs::CameraInfo>(sbuf, static_cast<uint>(CMNode.Cfg.QueuePub));
        CMNode.Topics.Pub.CameraMonoInfo.Job = CMCRJob_Create("CAMERA_INFO_M");

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

        /* Coordinate transform broadcaster */
        static tf2_ros::TransformBroadcaster br;
        CMNode.Global.TF_br = &br;

        /* sensor_msgs::PointCloud2 LidarRSI Message Metadata */
        CMNode.Topics.Pub.LidarRSI.Msg.header.frame_id = "Fr_LidarRSI";
        CMNode.Topics.Pub.LidarRSI.Msg.fields.resize(10);

        /* Field: BeamID */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[0].name = "beam_id";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[0].offset = 0;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[0].datatype = 5;              /* int32 */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[0].count = 1;

        /* Field: EchoID */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[1].name = "echo_id";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[1].offset = 4;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[1].datatype = 5;              /* int32 */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[1].count = 1;

        /* Field: Time of flight (ns) */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[2].name = "time_of";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[2].offset = 8;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[2].datatype = 8;              /* double */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[2].count = 1;

        /* Field: Length of flight (m) */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[3].name = "length_of";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[3].offset = 16;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[3].datatype = 8;              /* double */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[3].count = 1;

        /* Field: Origin.x of reflection */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[4].name = "x";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[4].offset = 24;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[4].datatype = 7;              /* float */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[4].count = 1;

        /* Field: Origin.y of reflection */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[5].name = "y";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[5].offset = 28;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[5].datatype = 7;              /* float */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[5].count = 1;

        /* Field: Origin.z of reflection */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[6].name = "z";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[6].offset = 32;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[6].datatype = 7;              /* float */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[6].count = 1;

        /* Field: Intensity (nW) */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[7].name = "intensity";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[7].offset = 36;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[7].datatype = 8;              /* double */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[7].count = 1;

        /* Field: Pulse width (ns) */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[8].name = "pulse_width";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[8].offset = 44;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[8].datatype = 8;              /* double */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[8].count = 1;

        /* Field: Number of reflections */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[9].name = "reflections";
        CMNode.Topics.Pub.LidarRSI.Msg.fields[9].offset = 52;
        CMNode.Topics.Pub.LidarRSI.Msg.fields[9].datatype = 5;              /* int32 */
        CMNode.Topics.Pub.LidarRSI.Msg.fields[9].count = 1;

        CMNode.Topics.Pub.LidarRSI.Msg.height = 1;
        CMNode.Topics.Pub.LidarRSI.Msg.is_bigendian = false;
        CMNode.Topics.Pub.LidarRSI.Msg.is_dense = true;


        /* sensor_msgs::PointCloud2 RadarRSI Message Metadata */
        CMNode.Topics.Pub.RadarRSI.Msg.header.frame_id = "Fr_RadarRSI";
        CMNode.Topics.Pub.RadarRSI.Msg.fields.resize(5);

        /* Field: x */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[0].name = "x";
        CMNode.Topics.Pub.RadarRSI.Msg.fields[0].offset = 0;
        CMNode.Topics.Pub.RadarRSI.Msg.fields[0].datatype = 7;              /* float */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[0].count = 1;

        /* Field: y */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[1].name = "y";
        CMNode.Topics.Pub.RadarRSI.Msg.fields[1].offset = 4;
        CMNode.Topics.Pub.RadarRSI.Msg.fields[1].datatype = 7;              /* float */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[1].count = 1;

        /* Field: z */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[2].name = "z";
        CMNode.Topics.Pub.RadarRSI.Msg.fields[2].offset = 8;
        CMNode.Topics.Pub.RadarRSI.Msg.fields[2].datatype = 7;              /* float */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[2].count = 1;

        /* Field: velocity (m/s) */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[3].name = "velocity";
        CMNode.Topics.Pub.RadarRSI.Msg.fields[3].offset = 12;
        CMNode.Topics.Pub.RadarRSI.Msg.fields[3].datatype = 8;              /* double */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[3].count = 1;

        /* Field: power (dBm) */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[4].name = "power";
        CMNode.Topics.Pub.RadarRSI.Msg.fields[4].offset = 20;
        CMNode.Topics.Pub.RadarRSI.Msg.fields[4].datatype = 8;              /* double */
        CMNode.Topics.Pub.RadarRSI.Msg.fields[4].count = 1;

        CMNode.Topics.Pub.RadarRSI.Msg.height = 1;
        CMNode.Topics.Pub.RadarRSI.Msg.is_bigendian = false;
        CMNode.Topics.Pub.RadarRSI.Msg.is_dense = true;

        /* Services */

        /* Print general information after everything is done */
        LOG("Initialisation of ROS Node finished!");
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

        DDefUChar   (df, "Cfg.Mode",           "-",  (unsigned char*)&CMNode.Cfg.Mode, DVA_None);
        DDefInt     (df, "Cfg.nCyclesClock",   "ms", &CMNode.Cfg.nCyclesClock,         DVA_None);

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
    */
    int
    CMRosIF_CMNode_TestRun_Start_atBegin (struct tInfos *Inf)
    {

        /* Node can be disabled via Infofile */
        tCMNode_Mode     *pmode     = &CMNode.Cfg.Mode;

        if (Inf != NULL) {
            *pmode     =     (tCMNode_Mode)iGetIntOpt(Inf, "Node.Mode",      CMNode_Mode_Disabled);
            
            CMNode.Sensor.CameraStereoL.Name    = iGetStrOpt(Inf, "Camera.Stereo.Left.Name", "");
            CMNode.Sensor.CameraStereoR.Name    = iGetStrOpt(Inf, "Camera.Stereo.Right.Name", "");
            CMNode.Sensor.CameraMono.Name       = iGetStrOpt(Inf, "Camera.Mono.Name", "");
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
        int idxC, idxS, idxP, Channel, ref;

        int cycletime           = 0;
        int cycleoff            = 0;
        tCMCRJob *job           = NULL;

        tInfos *Inf_Vhcl = NULL;
        tInfos *Inf_Lidar = NULL;
        tErrorMsg *err;
        int rStat;                              // Infofile read status
        int nRows = 0;                          // Number of rows when reading tables
        double def2c[] = {0, 0};                // Default 2-col table data
        double def3c[] = {0, 0, 0};             // Default 3-col table data
        double def6c[] = {0, 0, 0, 0, 0, 0};    // Default 6-col table data
        tf2::Quaternion q;

        /* Read the Vehicle Infofile */
        const char *FName_Vhcl;
        FName_Vhcl = InfoGetFilename(SimCore.Vhcl.Inf);      /* get the filename of the Infofile */
        Inf_Vhcl = InfoNew();
        iRead2(&err, Inf_Vhcl, FName_Vhcl, "Inf_Vhcl");      /* Create infofile handle */

        CMNode.Wheel.RL.pos                             = iGetFixedTable2(Inf_Vhcl, "Wheel.rl.pos", 3, 1);

        /* Total number of mounted sensors */
        int N = iGetIntOpt(Inf_Vhcl, "Sensor.N", 0);

        /* Global Navigation */
        idxP = -1;
        for (idxS = 0; idxS < N; idxS++) {
            sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
            ref = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.Type", ref);
            str = iGetStrOpt(Inf_Vhcl, sbuf, "");
            if (!strcmp(str, "GNav")) {
                /* If the GNav sensor is found, get its index and exit loop */
                idxP = ref;
                break;
            }
        }
        if (idxP != -1) {
            sprintf(sbuf, "Sensor.%d.Active", idxS);
            CMNode.Sensor.GNav.Active                   = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.%d.pos", idxS);
            CMNode.Sensor.GNav.pos                      = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.%d.rot", idxS);
            CMNode.Sensor.GNav.rot                      = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.Param.%d.UpdRate", idxP);
            CMNode.Sensor.GNav.UpdRate                  = iGetIntOpt(Inf_Vhcl, sbuf, 10);
            sprintf(sbuf, "Sensor.Param.%d.CycleOffset", idxP);
            CMNode.Sensor.GNav.nCycleOffset             = iGetIntOpt(Inf_Vhcl, sbuf, 0);
        } else {
            CMNode.Sensor.GNav.Active                   = 0;
        }

        /* LiDAR RSI */
        idxP = -1;
        for (idxS = 0; idxS < N; idxS++) {
            sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
            ref = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.Type", ref);
            str = iGetStrOpt(Inf_Vhcl, sbuf, "");
            if (!strcmp(str, "LidarRSI")) {
                /* If the LidarRSI sensor is found, get its index and exit loop */
                idxP = ref;
                sprintf(sbuf, "Sensor.%d.Ref.Cluster", idxS);
                idxC = iGetIntOpt(Inf_Vhcl, sbuf, 0);
                break;
            }
        }
        if (idxP != -1) {
            sprintf(sbuf, "Sensor.%d.Active", idxS);
            CMNode.Sensor.LidarRSI.Active                   = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.%d.pos", idxS);
            CMNode.Sensor.LidarRSI.pos                      = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.%d.rot", idxS);
            CMNode.Sensor.LidarRSI.rot                      = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "SensorCluster.%d.CycleTime", idxC);
            CMNode.Sensor.LidarRSI.UpdRate                  = iGetIntOpt(Inf_Vhcl, sbuf, 10);
            sprintf(sbuf, "SensorCluster.%d.CycleOffset", idxC);
            CMNode.Sensor.LidarRSI.nCycleOffset             = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.BeamsFName", idxP);
            CMNode.Sensor.LidarRSI.BeamFName                = iGetStrOpt(Inf_Vhcl, sbuf, "LidarRSI_Default");

            /* Read the LiDAR RSI Map Infofile */
            const char *FName_Lidar;
            std::string DName_Lidar, EName_Lidar;                       /* These will hold the potential pathnames to the Lidar map file in the project Data and in IPG Examples */
            FName_Lidar = CMNode.Sensor.LidarRSI.BeamFName;             /* get the filename of the Infofile */

            DName_Lidar.append("Data/Sensor/");
            DName_Lidar.append(FName_Lidar);

            EName_Lidar.append(SimCore.System.ProdDir);
            EName_Lidar.append("/");
            EName_Lidar.append(DName_Lidar);

            Inf_Lidar = InfoNew();
            rStat = iRead2(&err, Inf_Lidar, DName_Lidar.c_str(), "Lidar_Map");      /* Attempt to read file in project Data */

            if (rStat) {
                /* First try failed. Attempt to read file in IPG Examples */
                rStat = iRead2(&err, Inf_Lidar, EName_Lidar.c_str(), "Lidar_Map");

                if (rStat) {
                    /* Both locations failed. Could not locate the LiDAR map file! */
                    LogErrF(EC_Sim, "Could not locate LiDAR beam data file with name %s", FName_Lidar);
                } else {
                    LOG("Reading LiDAR beam data from file: %s", EName_Lidar.c_str());
                }
            } else {
                LOG("Reading LiDAR beam data from file: %s", DName_Lidar.c_str());
            }

            CMNode.Sensor.LidarRSI.Beams                    = iGetTableOpt2(Inf_Lidar, "Beams", def6c, 6, &CMNode.Sensor.LidarRSI.nBeams);

            /* Build the LIDAR RSI frame transformation */
            CMNode.Sensor.LidarRSI.TF.header.frame_id       = "Fr1";
            CMNode.Sensor.LidarRSI.TF.child_frame_id        = "Fr_LidarRSI";
            q.setRPY(CMNode.Sensor.LidarRSI.rot[0] * DEG_to_RAD, CMNode.Sensor.LidarRSI.rot[1] * DEG_to_RAD, CMNode.Sensor.LidarRSI.rot[2] * DEG_to_RAD);
            CMNode.Sensor.LidarRSI.TF.transform.rotation.x  = q.x();
            CMNode.Sensor.LidarRSI.TF.transform.rotation.y  = q.y();
            CMNode.Sensor.LidarRSI.TF.transform.rotation.z  = q.z();
            CMNode.Sensor.LidarRSI.TF.transform.rotation.w  = q.w();
            CMNode.Sensor.LidarRSI.TF.transform.translation.x = CMNode.Sensor.LidarRSI.pos[0];
            CMNode.Sensor.LidarRSI.TF.transform.translation.y = CMNode.Sensor.LidarRSI.pos[1];
            CMNode.Sensor.LidarRSI.TF.transform.translation.z = CMNode.Sensor.LidarRSI.pos[2];
        } else {
            CMNode.Sensor.LidarRSI.Active                   = 0;
        }
        
        /* RADAR RSI */
        idxP = -1;
        for (idxS = 0; idxS < N; idxS++) {
            sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
            ref = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.Type", ref);
            str = iGetStrOpt(Inf_Vhcl, sbuf, "");
            if (!strcmp(str, "RadarRSI")) {
                /* If the RadarRSI sensor is found, get its index and exit loop */
                idxP = ref;
                sprintf(sbuf, "Sensor.%d.Ref.Cluster", idxS);
                idxC = iGetIntOpt(Inf_Vhcl, sbuf, 0);
                break;
            }
        }
        if (idxP != -1) {
            sprintf(sbuf, "Sensor.%d.Active", idxS);
            CMNode.Sensor.RadarRSI.Active                   = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.%d.pos", idxS);
            CMNode.Sensor.RadarRSI.pos                      = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.%d.rot", idxS);
            CMNode.Sensor.RadarRSI.rot                      = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "SensorCluster.%d.CycleTime", idxC);
            CMNode.Sensor.RadarRSI.UpdRate                  = iGetIntOpt(Inf_Vhcl, sbuf, 10);
            sprintf(sbuf, "SensorCluster.%d.CycleOffset", idxC);
            CMNode.Sensor.RadarRSI.nCycleOffset             = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.OutputType", idxP);
            CMNode.Sensor.RadarRSI.OutputType               = iGetIntOpt(Inf_Vhcl, sbuf, 0);

            /* Build the RADAR RSI frame transformation */
            CMNode.Sensor.RadarRSI.TF.header.frame_id       = "Fr1";
            CMNode.Sensor.RadarRSI.TF.child_frame_id        = "Fr_RadarRSI";
            q.setRPY(CMNode.Sensor.RadarRSI.rot[0] * DEG_to_RAD, CMNode.Sensor.RadarRSI.rot[1] * DEG_to_RAD, CMNode.Sensor.RadarRSI.rot[2] * DEG_to_RAD);
            CMNode.Sensor.RadarRSI.TF.transform.rotation.x  = q.x();
            CMNode.Sensor.RadarRSI.TF.transform.rotation.y  = q.y();
            CMNode.Sensor.RadarRSI.TF.transform.rotation.z  = q.z();
            CMNode.Sensor.RadarRSI.TF.transform.rotation.w  = q.w();
            CMNode.Sensor.RadarRSI.TF.transform.translation.x = CMNode.Sensor.RadarRSI.pos[0];
            CMNode.Sensor.RadarRSI.TF.transform.translation.y = CMNode.Sensor.RadarRSI.pos[1];
            CMNode.Sensor.RadarRSI.TF.transform.translation.z = CMNode.Sensor.RadarRSI.pos[2];
        } else {
            CMNode.Sensor.RadarRSI.Active                   = 0;
        }

        /* CameraStereoL */
        idxP = -1;
        Channel = 0;
        for (idxS = 0; idxS < N; idxS++) {
            sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
            ref = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.Type", ref);
            str = iGetStrOpt(Inf_Vhcl, sbuf, "");
            if (!strcmp(str, "CameraRSI")) {
                /* If the CameraRSI sensor is found, check the name */
                sprintf(sbuf, "Sensor.%d.name", idxS);
                str = iGetStrOpt(Inf_Vhcl, sbuf, "");
                if (!strcmp(str, CMNode.Sensor.CameraStereoL.Name)) {
                    /* If the matching name is found, get its index and exit loop */
                    idxP = ref;
                    sprintf(sbuf, "Sensor.%d.Ref.Cluster", idxS);
                    idxC = iGetIntOpt(Inf_Vhcl, sbuf, 0);
                    break;
                } else {
                    /* A different CameraRSI was found, increment the channel number if active */
                    sprintf(sbuf, "Sensor.%d.Active", idxS);
                    if(iGetIntOpt(Inf_Vhcl, sbuf, 0)) {
                        Channel++;
                    }
                    
                }
            }
        }
        if (idxP != -1) {
            sprintf(sbuf, "Sensor.%d.Active", idxS);
            CMNode.Sensor.CameraStereoL.Active                      = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.%d.pos", idxS);
            CMNode.Sensor.CameraStereoL.pos                         = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.%d.rot", idxS);
            CMNode.Sensor.CameraStereoL.rot                         = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "SensorCluster.%d.CycleTime", idxC);
            CMNode.Sensor.CameraStereoL.UpdRate                     = iGetIntOpt(Inf_Vhcl, sbuf, 10);
            sprintf(sbuf, "SensorCluster.%d.CycleOffset", idxC);
            CMNode.Sensor.CameraStereoL.nCycleOffset                = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.OutputFormat", idxP);
            CMNode.Sensor.CameraStereoL.OutputFormat                = iGetStrOpt(Inf_Vhcl, sbuf, "");
            CMNode.Sensor.CameraStereoL.Channel                     = Channel;
        
            /* Build the CameraStereoL frame transformation */
            CMNode.Sensor.CameraStereoL.TF.header.frame_id          = "Fr1";
            CMNode.Sensor.CameraStereoL.TF.child_frame_id           = "Fr_CameraStereoL";
            q.setRPY(
                (CMNode.Sensor.CameraStereoL.rot[0] + 90) * DEG_to_RAD, 
                (CMNode.Sensor.CameraStereoL.rot[1] + 180) * DEG_to_RAD, 
                (CMNode.Sensor.CameraStereoL.rot[2] + 90) * DEG_to_RAD);
            CMNode.Sensor.CameraStereoL.TF.transform.rotation.x     = q.x();
            CMNode.Sensor.CameraStereoL.TF.transform.rotation.y     = q.y();
            CMNode.Sensor.CameraStereoL.TF.transform.rotation.z     = q.z();
            CMNode.Sensor.CameraStereoL.TF.transform.rotation.w     = q.w();
            CMNode.Sensor.CameraStereoL.TF.transform.translation.x  = CMNode.Sensor.CameraStereoL.pos[0];
            CMNode.Sensor.CameraStereoL.TF.transform.translation.y  = CMNode.Sensor.CameraStereoL.pos[1];
            CMNode.Sensor.CameraStereoL.TF.transform.translation.z  = CMNode.Sensor.CameraStereoL.pos[2];
        } else {
            CMNode.Sensor.CameraStereoL.Active                      = 0;
        }

        /* CameraStereoR */
        idxP = -1;
        Channel = 0;
        for (idxS = 0; idxS < N; idxS++) {
            sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
            ref = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.Type", ref);
            str = iGetStrOpt(Inf_Vhcl, sbuf, "");
            if (!strcmp(str, "CameraRSI")) {
                /* If the CameraRSI sensor is found, check the name */
                sprintf(sbuf, "Sensor.%d.name", idxS);
                str = iGetStrOpt(Inf_Vhcl, sbuf, "");
                if (!strcmp(str, CMNode.Sensor.CameraStereoR.Name)) {
                    /* If the matching name is found, get its index and exit loop */
                    idxP = ref;
                    sprintf(sbuf, "Sensor.%d.Ref.Cluster", idxS);
                    idxC = iGetIntOpt(Inf_Vhcl, sbuf, 0);
                    break;
                } else {
                    /* A different CameraRSI was found, increment the channel number if active */
                    sprintf(sbuf, "Sensor.%d.Active", idxS);
                    if(iGetIntOpt(Inf_Vhcl, sbuf, 0)) {
                        Channel++;
                    }
                }
            }
        }
        if (idxP != -1) {
            sprintf(sbuf, "Sensor.%d.Active", idxS);
            CMNode.Sensor.CameraStereoR.Active                      = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.%d.pos", idxS);
            CMNode.Sensor.CameraStereoR.pos                         = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.%d.rot", idxS);
            CMNode.Sensor.CameraStereoR.rot                         = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "SensorCluster.%d.CycleTime", idxC);
            CMNode.Sensor.CameraStereoR.UpdRate                     = iGetIntOpt(Inf_Vhcl, sbuf, 10);
            sprintf(sbuf, "SensorCluster.%d.CycleOffset", idxC);
            CMNode.Sensor.CameraStereoR.nCycleOffset                = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.OutputFormat", idxP);
            CMNode.Sensor.CameraStereoR.OutputFormat                = iGetStrOpt(Inf_Vhcl, sbuf, "");
            CMNode.Sensor.CameraStereoR.Channel                     = Channel;

            /* Build the CameraStereoR frame transformation */
            CMNode.Sensor.CameraStereoR.TF.header.frame_id          = "Fr1";
            CMNode.Sensor.CameraStereoR.TF.child_frame_id           = "Fr_CameraStereoR";
            q.setRPY(
                (CMNode.Sensor.CameraStereoR.rot[0] + 90) * DEG_to_RAD, 
                (CMNode.Sensor.CameraStereoR.rot[1] + 180) * DEG_to_RAD, 
                (CMNode.Sensor.CameraStereoR.rot[2] + 90) * DEG_to_RAD); 
            CMNode.Sensor.CameraStereoR.TF.transform.rotation.x     = q.x();
            CMNode.Sensor.CameraStereoR.TF.transform.rotation.y     = q.y();
            CMNode.Sensor.CameraStereoR.TF.transform.rotation.z     = q.z();
            CMNode.Sensor.CameraStereoR.TF.transform.rotation.w     = q.w();
            CMNode.Sensor.CameraStereoR.TF.transform.translation.x  = CMNode.Sensor.CameraStereoR.pos[0];
            CMNode.Sensor.CameraStereoR.TF.transform.translation.y  = CMNode.Sensor.CameraStereoR.pos[1];
            CMNode.Sensor.CameraStereoR.TF.transform.translation.z  = CMNode.Sensor.CameraStereoR.pos[2];
        } else {
            CMNode.Sensor.CameraStereoR.Active                      = 0;
        }

        /* CameraMono */
        idxP = -1;
        Channel = 0;
        for (idxS = 0; idxS < N; idxS++) {
            sprintf(sbuf, "Sensor.%d.Ref.Param", idxS);
            ref = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.Type", ref);
            str = iGetStrOpt(Inf_Vhcl, sbuf, "");
            if (!strcmp(str, "CameraRSI")) {
                /* If the CameraRSI sensor is found, check the name */
                sprintf(sbuf, "Sensor.%d.name", idxS);
                str = iGetStrOpt(Inf_Vhcl, sbuf, "");
                if (!strcmp(str, CMNode.Sensor.CameraMono.Name)) {
                    /* If the matching name is found, get its index and exit loop */
                    idxP = ref;
                    sprintf(sbuf, "Sensor.%d.Ref.Cluster", idxS);
                    idxC = iGetIntOpt(Inf_Vhcl, sbuf, 0);
                    break;
                } else {
                    /* A different CameraRSI was found, increment the channel number if active */
                    sprintf(sbuf, "Sensor.%d.Active", idxS);
                    if(iGetIntOpt(Inf_Vhcl, sbuf, 0)) {
                        Channel++;
                    }
                }
            }
        }
        if (idxP != -1) {
            sprintf(sbuf, "Sensor.%d.Active", idxS);
            CMNode.Sensor.CameraMono.Active                         = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.%d.pos", idxS);
            CMNode.Sensor.CameraMono.pos                            = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "Sensor.%d.rot", idxS);
            CMNode.Sensor.CameraMono.rot                            = iGetFixedTableOpt2(Inf_Vhcl, sbuf, def3c, 3, 1);
            sprintf(sbuf, "SensorCluster.%d.CycleTime", idxC);
            CMNode.Sensor.CameraMono.UpdRate                        = iGetIntOpt(Inf_Vhcl, sbuf, 10);
            sprintf(sbuf, "SensorCluster.%d.CycleOffset", idxC);
            CMNode.Sensor.CameraMono.nCycleOffset                   = iGetIntOpt(Inf_Vhcl, sbuf, 0);
            sprintf(sbuf, "Sensor.Param.%d.OutputFormat", idxP);
            CMNode.Sensor.CameraMono.OutputFormat                   = iGetStrOpt(Inf_Vhcl, sbuf, "");
            CMNode.Sensor.CameraMono.Channel                        = Channel;

            /* Build the CameraMono frame transformation */
            CMNode.Sensor.CameraMono.TF.header.frame_id             = "Fr1";
            CMNode.Sensor.CameraMono.TF.child_frame_id              = "Fr_CameraMono";
            q.setRPY(
                (CMNode.Sensor.CameraMono.rot[0] + 90) * DEG_to_RAD, 
                (CMNode.Sensor.CameraMono.rot[1] + 180) * DEG_to_RAD, 
                (CMNode.Sensor.CameraMono.rot[2] + 90) * DEG_to_RAD);            
            CMNode.Sensor.CameraMono.TF.transform.rotation.x        = q.x();
            CMNode.Sensor.CameraMono.TF.transform.rotation.y        = q.y();
            CMNode.Sensor.CameraMono.TF.transform.rotation.z        = q.z();
            CMNode.Sensor.CameraMono.TF.transform.rotation.w        = q.w();
            CMNode.Sensor.CameraMono.TF.transform.translation.x     = CMNode.Sensor.CameraMono.pos[0];
            CMNode.Sensor.CameraMono.TF.transform.translation.y     = CMNode.Sensor.CameraMono.pos[1];
            CMNode.Sensor.CameraMono.TF.transform.translation.z     = CMNode.Sensor.CameraMono.pos[2];
        } else {
            CMNode.Sensor.CameraMono.Active                         = 0;
        }

        LOG("CarMaker ROS Node is enabled! Mode=%d", *pmode);
        LOG("  -> Node Name = %s", ros::this_node::getName().c_str());

        /* Close all Infofile handles */
        InfoDelete(Inf_Vhcl);
        InfoDelete(Inf_Lidar);

        /* Reset for next cycle */
        CMNode.CycleNoRel           =  0;
        CMNode.Model.CycleNo        = -1;
        CMNode.Model.CycleLastIn    = -1;
        CMNode.Model.CycleLastOut   = -1;
        CMNode.Model.CycleLastFlush = -1;


        /* Allow an update of the clock only if it was enabled before! */
        if (CMNode.Cfg.nCyclesClock > 0) {
            if ((rv = iGetIntOpt(Inf, "Node.nCyclesClock", 1000)) > 0)
            CMNode.Cfg.nCyclesClock = rv;
        }

        /* Necessary to ensure /clock is zeroed here? */
        if (CMNode.Cfg.nCyclesClock > 0) {
            LOG("  -> Publish /clock every %dms", CMNode.Cfg.nCyclesClock);
            CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
            CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
        }

        /* Prepare Jobs for publish and subscribe
        * - Special use case:
        *   - Topic in and Topic out use same cycle time with relative shift!
        *   - CarMaker starts publishing at 1ms instead of 0ms, so a 1ms offset
        *     has been artificially added here
        */

        /* SDControl sub job */
        job         = CMNode.Topics.Sub.SDC.Job;
        cycletime   = CMNode.Topics.Sub.SDC.CycleTime;
        cycleoff    = CMNode.Topics.Sub.SDC.CycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);

        /* GPS pub job */
        job         = CMNode.Topics.Pub.GPS.Job;
        cycletime   = CMNode.Sensor.GNav.UpdRate;
        cycleoff    = CMNode.Sensor.GNav.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* LiDAR RSI pub job */
        job         = CMNode.Topics.Pub.LidarRSI.Job;
        cycletime   = CMNode.Sensor.LidarRSI.UpdRate;
        cycleoff    = CMNode.Sensor.LidarRSI.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* RADAR RSI pub job */
        job         = CMNode.Topics.Pub.RadarRSI.Job;
        cycletime   = CMNode.Sensor.RadarRSI.UpdRate;
        cycleoff    = CMNode.Sensor.RadarRSI.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Stereo Camera RSI Left pub job */
        job         = CMNode.Topics.Pub.CameraStereoL.Job;
        cycletime   = CMNode.Sensor.CameraStereoL.UpdRate;
        cycleoff    = CMNode.Sensor.CameraStereoL.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Stereo Camera RSI Left Info pub job */
        job         = CMNode.Topics.Pub.CameraInfoL.Job;
        cycletime   = CMNode.Sensor.CameraStereoL.UpdRate;
        cycleoff    = CMNode.Sensor.CameraStereoL.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Stereo Camera RSI Right pub job */
        job         = CMNode.Topics.Pub.CameraStereoR.Job;
        cycletime   = CMNode.Sensor.CameraStereoR.UpdRate;
        cycleoff    = CMNode.Sensor.CameraStereoR.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Stereo Camera RSI Right Info pub job */
        job         = CMNode.Topics.Pub.CameraInfoR.Job;
        cycletime   = CMNode.Sensor.CameraStereoR.UpdRate;
        cycleoff    = CMNode.Sensor.CameraStereoR.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Stereo Camera RSI RGB pub job */
        job         = CMNode.Topics.Pub.CameraRGB.Job;
        cycletime   = CMNode.Sensor.CameraStereoL.UpdRate;
        cycleoff    = CMNode.Sensor.CameraStereoL.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Stereo Camera RSI RGB Info pub job */
        job         = CMNode.Topics.Pub.CameraInfoRGB.Job;
        cycletime   = CMNode.Sensor.CameraStereoL.UpdRate;
        cycleoff    = CMNode.Sensor.CameraStereoL.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Mono Camera RSI pub job */
        job         = CMNode.Topics.Pub.CameraMono.Job;
        cycletime   = CMNode.Sensor.CameraMono.UpdRate;
        cycleoff    = CMNode.Sensor.CameraMono.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Mono Camera RSI Info pub job */
        job         = CMNode.Topics.Pub.CameraMonoInfo.Job;
        cycletime   = CMNode.Sensor.CameraMono.UpdRate;
        cycleoff    = CMNode.Sensor.CameraMono.nCycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);

        /* Vehicle Velocity pub job */
        job         = CMNode.Topics.Pub.Velocity.Job;
        cycletime   = CMNode.Topics.Pub.Velocity.CycleTime;
        cycleoff    = CMNode.Topics.Pub.Velocity.CycleOffset+1;
        CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Ext);


        LOG("External ROS Node is ready to simulate");

        return 1;
    }



    /*!
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
    * - Process all topics/services
    * - See "User.c:User_In()"
    *
    */
    int
    CMRosIF_CMNode_In (void)
    {
        /* Is the CMNode enabled */
        switch (CMNode.Cfg.Mode) {
            case CMNode_Mode_Disabled:
                /* No messages/services shall be processed in disabled state */
                break;

            case CMNode_Mode_Default:
                ros::spinOnce();
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
            CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;
        }
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
        uint8_t *ptr;
        uint8_t tmp;
        int point_step, row_step;
        int bgr_idx;
        double maxd;

        /* 3D Coordinate system points */
        double x, y, z;
        double distance, azimuth, elevation;

        /* LiDAR parameter references for convenience */
        double *Lbeams  = CMNode.Sensor.LidarRSI.Beams;
        int N           = CMNode.Sensor.LidarRSI.nBeams;

        /* Only do anything if simulation is running */
        if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate) {
            return 0;
        }

        /* Publish GPS data from CarMaker */
        if (CMNode.Sensor.GNav.Active) {
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.GPS.Job, CMNode.CycleNoRel, 1, nullptr, nullptr)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.GPS.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {
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
        if (CMNode.Sensor.LidarRSI.Active) {
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.LidarRSI.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.LidarRSI.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Frame header timestamp */
                CMNode.Topics.Pub.LidarRSI.Msg.header.stamp = ros::Time(LidarRSI[0].ScanTime);

                /* Unordered cloud with the number of scanned points of the LiDAR */
                CMNode.Topics.Pub.LidarRSI.Msg.width = LidarRSI[0].nScanPoints;

                // LiDAR messages contain 56 bytes
                point_step = 56;
                row_step = point_step * LidarRSI[0].nScanPoints;
                CMNode.Topics.Pub.LidarRSI.Msg.point_step = point_step;
                CMNode.Topics.Pub.LidarRSI.Msg.row_step = row_step;

                /* Create the output data binary blob */
                CMNode.Topics.Pub.LidarRSI.Msg.data.resize(row_step);
                ptr = CMNode.Topics.Pub.LidarRSI.Msg.data.data();

                for (int ii = 0; ii < LidarRSI[0].nScanPoints; ii++) {
                    /* Extract the beam spherical and convert to Cartesian coordinates */
                    distance    = LidarRSI[0].ScanPoint[ii].LengthOF/2;                             // Flight length is double the distance
                    azimuth     = DEG_to_RAD * Lbeams[4*N + LidarRSI[0].ScanPoint[ii].BeamID];      // BeamID is the table row, and Azimuth is column 5
                    elevation   = DEG_to_RAD * Lbeams[5*N + LidarRSI[0].ScanPoint[ii].BeamID];      // BeamID is the table row, and Elevation is column 6
                    x           = distance*cos(elevation)*cos(azimuth);
                    y           = distance*cos(elevation)*sin(azimuth);
                    z           = distance*sin(elevation);                                          // sin(elevation) == cos(inclination)

                    *(reinterpret_cast<uint32_t*>(ptr +  0)) = LidarRSI[0].ScanPoint[ii].BeamID;
                    *(reinterpret_cast<uint32_t*>(ptr +  4)) = LidarRSI[0].ScanPoint[ii].EchoID;
                    *(reinterpret_cast<double*>(ptr +  8)) = LidarRSI[0].ScanPoint[ii].TimeOF;
                    *(reinterpret_cast<double*>(ptr + 16)) = LidarRSI[0].ScanPoint[ii].LengthOF;
                    *(reinterpret_cast<float*>(ptr + 24)) = (float) x;
                    *(reinterpret_cast<float*>(ptr + 28)) = (float) y;
                    *(reinterpret_cast<float*>(ptr + 32)) = (float) z;
                    *(reinterpret_cast<double*>(ptr + 36)) = LidarRSI[0].ScanPoint[ii].Intensity;
                    *(reinterpret_cast<double*>(ptr + 44)) = LidarRSI[0].ScanPoint[ii].PulseWidth;
                    *(reinterpret_cast<uint32_t*>(ptr + 52)) = LidarRSI[0].ScanPoint[ii].nRefl;
                    ptr += point_step;
                }
            }
        }

        /* Publish RADAR RSI data from CarMaker */
        if (CMNode.Sensor.RadarRSI.Active) {
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.RadarRSI.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.RadarRSI.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Frame header timestamp*/
                CMNode.Topics.Pub.RadarRSI.Msg.header.stamp = ros::Time(RadarRSI[0].TimeFired);

                /* Unordered cloud with the number of scanned points of the RADAR */
                CMNode.Topics.Pub.RadarRSI.Msg.width = RadarRSI[0].nDetections;

                // RADAR messages contain 28 bytes
                point_step = 28;
                row_step = point_step * RadarRSI[0].nDetections;
                CMNode.Topics.Pub.RadarRSI.Msg.point_step = point_step;
                CMNode.Topics.Pub.RadarRSI.Msg.row_step = row_step;

                /* Create the output data binary blob */
                CMNode.Topics.Pub.RadarRSI.Msg.data.resize(row_step);
                ptr = CMNode.Topics.Pub.RadarRSI.Msg.data.data();

                for (int ii = 0; ii < RadarRSI[0].nDetections; ii++) {
                    switch(CMNode.Sensor.RadarRSI.OutputType) {
                        case 0:
                            x           = RadarRSI[0].DetPoints[ii].Coordinates[0];
                            y           = RadarRSI[0].DetPoints[ii].Coordinates[1];
                            z           = RadarRSI[0].DetPoints[ii].Coordinates[2];
                            break;
                        case 1:
                            distance    = RadarRSI[0].DetPoints[ii].Coordinates[0];
                            azimuth     = RadarRSI[0].DetPoints[ii].Coordinates[1];
                            elevation   = RadarRSI[0].DetPoints[ii].Coordinates[2];     // note that this is elevation, NOT inclination
                            x           = distance*cos(elevation)*cos(azimuth);
                            y           = distance*cos(elevation)*sin(azimuth);
                            z           = distance*sin(elevation);                      // sin(elevation) == cos(inclination)
                            break;
                        case 2:
                            LogErrF(EC_Sim, "CMNode: VRx RADAR RSI receivers not supported.");
                    }
                    *(reinterpret_cast<float*>(ptr +   0)) = (float) x;
                    *(reinterpret_cast<float*>(ptr +   4)) = (float) y;
                    *(reinterpret_cast<float*>(ptr +   8)) = (float) z;
                    *(reinterpret_cast<double*>(ptr + 12)) = RadarRSI[0].DetPoints[ii].Velocity;
                    *(reinterpret_cast<double*>(ptr + 20)) = RadarRSI[0].DetPoints[ii].Power;

                    ptr += point_step;
                }
            }
        }

        /* Publish CameraStereoL/CameraRGB RSI data from CarMaker */
        if (CMNode.Sensor.CameraStereoL.Active) {
            /* CameraStereoL Image */
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.CameraStereoL.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.CameraStereoL.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Get the sensor channel number in the RSDS data */
                int cidx = CMNode.Sensor.CameraStereoL.Channel;

                CMNode.Topics.Pub.CameraStereoL.Msg = CData[cidx];

                /* Frame header */
                CMNode.Topics.Pub.CameraStereoL.Msg.header.frame_id     = "Fr_CameraStereoL";
                CMNode.Topics.Pub.CameraStereoL.Msg.header.stamp        = ros::Time(SimCore.Time);
            }

            /* CameraStereoRGB Image */
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.CameraRGB.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.CameraRGB.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {
                /* Get the sensor channel number in the RSDS data */
                int cidx = CMNode.Sensor.CameraStereoL.Channel;

                CMNode.Topics.Pub.CameraRGB.Msg = CData[cidx];

                /* Frame header */
                CMNode.Topics.Pub.CameraRGB.Msg.header.frame_id         = "Fr_CameraStereoL";
                CMNode.Topics.Pub.CameraRGB.Msg.header.stamp            = ros::Time(SimCore.Time);
            }

            /* CameraStereoL Info */
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.CameraInfoL.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.CameraInfoL.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Frame header */
                CMNode.Topics.Pub.CameraInfoL.Msg.header.frame_id       = "Fr_CameraStereoL";
                CMNode.Topics.Pub.CameraInfoL.Msg.header.stamp          = ros::Time(SimCore.Time);

                /* Get the sensor channel number in the RSDS data */
                int cidx = CMNode.Sensor.CameraStereoL.Channel;

                /* Extract the image size */
                CMNode.Topics.Pub.CameraInfoL.Msg.height                = CData[cidx].height;
                CMNode.Topics.Pub.CameraInfoL.Msg.width                 = CData[cidx].width;
                maxd = (double)std::max(CData[cidx].height, CData[cidx].width);

                /* Set the image distortion model */
                CMNode.Topics.Pub.CameraInfoL.Msg.distortion_model      = "plump_bob";
                CMNode.Topics.Pub.CameraInfoL.Msg.D                     = {1, 0, 0, 0, 0};
                CMNode.Topics.Pub.CameraInfoL.Msg.K                     = { maxd*7/8,   0,          (double)CData[cidx].width/2, 
                                                                            0,          maxd*7/8,   (double)CData[cidx].height/2, 
                                                                            0,          0,          1
                                                                          };
                CMNode.Topics.Pub.CameraInfoL.Msg.R                     = { 1, 0, 0, 
                                                                            0, 1, 0, 
                                                                            0, 0, 1
                                                                          };
                CMNode.Topics.Pub.CameraInfoL.Msg.P                     = { maxd*7/8,   0,          (double)(CData[cidx].width)/2,      0, 
                                                                            0,          maxd*7/8,   (double)(CData[cidx].height)/2,     0, 
                                                                            0,          0,          1,                                  0
                                                                          };

                /* Set the subsampling values */
                CMNode.Topics.Pub.CameraInfoL.Msg.binning_x             = 1;
                CMNode.Topics.Pub.CameraInfoL.Msg.binning_y             = 1;

                /* Set the region of interest */
                CMNode.Topics.Pub.CameraInfoL.Msg.roi.x_offset          = 0;
                CMNode.Topics.Pub.CameraInfoL.Msg.roi.y_offset          = 0;
                CMNode.Topics.Pub.CameraInfoL.Msg.roi.height            = 0;
                CMNode.Topics.Pub.CameraInfoL.Msg.roi.width             = 0;
                CMNode.Topics.Pub.CameraInfoL.Msg.roi.do_rectify        = false;
            }

            /* CameraStereoRGB Info */
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.CameraInfoRGB.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.CameraInfoRGB.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Frame header */
                CMNode.Topics.Pub.CameraInfoRGB.Msg.header.frame_id     = "Fr_CameraStereoL";
                CMNode.Topics.Pub.CameraInfoRGB.Msg.header.stamp        = ros::Time(SimCore.Time);

                /* Get the sensor channel number in the RSDS data */
                int cidx = CMNode.Sensor.CameraStereoL.Channel;

                /* Extract the image size */
                CMNode.Topics.Pub.CameraInfoRGB.Msg.height                = CData[cidx].height;
                CMNode.Topics.Pub.CameraInfoRGB.Msg.width                 = CData[cidx].width;
                maxd = (double)std::max(CData[cidx].height, CData[cidx].width);

                /* Set the image distortion model */
                CMNode.Topics.Pub.CameraInfoRGB.Msg.distortion_model      = "plump_bob";
                CMNode.Topics.Pub.CameraInfoRGB.Msg.D                     = {1, 0, 0, 0, 0};
                CMNode.Topics.Pub.CameraInfoRGB.Msg.K                     = { maxd*7/8,   0,        (double)CData[cidx].width/2, 
                                                                            0,          maxd*7/8,   (double)CData[cidx].height/2, 
                                                                            0,          0,          1
                                                                          };
                CMNode.Topics.Pub.CameraInfoRGB.Msg.R                     = { 1, 0, 0, 
                                                                            0, 1, 0, 
                                                                            0, 0, 1
                                                                          };
                CMNode.Topics.Pub.CameraInfoRGB.Msg.P                     = { maxd*7/8,   0,        (double)(CData[cidx].width)/2,      0, 
                                                                            0,          maxd*7/8,   (double)(CData[cidx].height)/2,     0, 
                                                                            0,          0,          1,                                  0
                                                                          };

                /* Set the subsampling values */
                CMNode.Topics.Pub.CameraInfoRGB.Msg.binning_x           = 1;
                CMNode.Topics.Pub.CameraInfoRGB.Msg.binning_y           = 1;

                /* Set the region of interest */
                CMNode.Topics.Pub.CameraInfoRGB.Msg.roi.x_offset        = 0;
                CMNode.Topics.Pub.CameraInfoRGB.Msg.roi.y_offset        = 0;
                CMNode.Topics.Pub.CameraInfoRGB.Msg.roi.height          = 0;
                CMNode.Topics.Pub.CameraInfoRGB.Msg.roi.width           = 0;
                CMNode.Topics.Pub.CameraInfoRGB.Msg.roi.do_rectify      = false;
            }
        }

        /* Publish CameraStereoR RSI data from CarMaker */
        if (CMNode.Sensor.CameraStereoR.Active) {
            /* Image Data */
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.CameraStereoR.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.CameraStereoR.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Get the sensor channel number in the RSDS data */
                int cidx = CMNode.Sensor.CameraStereoR.Channel;

                CMNode.Topics.Pub.CameraStereoR.Msg = CData[cidx];
                
                /* Frame header */
                CMNode.Topics.Pub.CameraStereoR.Msg.header.frame_id     = "Fr_CameraStereoR";
                CMNode.Topics.Pub.CameraStereoR.Msg.header.stamp        = ros::Time(SimCore.Time);
            }

            /* Camera Info */
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.CameraInfoR.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.CameraInfoR.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Frame header */
                CMNode.Topics.Pub.CameraInfoR.Msg.header.frame_id       = "Fr_CameraStereoR";
                CMNode.Topics.Pub.CameraInfoR.Msg.header.stamp          = ros::Time(SimCore.Time);

                /* Get the sensor channel number in the RSDS data */
                int cidx = CMNode.Sensor.CameraStereoR.Channel;

                /* Extract the image size */
                CMNode.Topics.Pub.CameraInfoR.Msg.height                = CData[cidx].height;
                CMNode.Topics.Pub.CameraInfoR.Msg.width                 = CData[cidx].width;
                maxd = (double)std::max(CData[cidx].height, CData[cidx].width);

                /* Set the image distortion model */
                CMNode.Topics.Pub.CameraInfoR.Msg.distortion_model      = "plump_bob";
                CMNode.Topics.Pub.CameraInfoR.Msg.D                     = {1, 0, 0, 0, 0};
                CMNode.Topics.Pub.CameraInfoR.Msg.K                     = { maxd*7/8,   0,          (double)CData[cidx].width/2, 
                                                                            0,          maxd*7/8,   (double)CData[cidx].height/2, 
                                                                            0,          0,          1
                                                                          };
                CMNode.Topics.Pub.CameraInfoR.Msg.R                     = { 1, 0, 0, 
                                                                            0, 1, 0, 
                                                                            0, 0, 1
                                                                          };
                CMNode.Topics.Pub.CameraInfoR.Msg.P                     = { maxd*7/8,   0,          (double)(CData[cidx].width)/2,      0, 
                                                                            0,          maxd*7/8,   (double)(CData[cidx].height)/2,     0, 
                                                                            0,          0,          1,                                  0
                                                                          };

                /* Set the subsampling values */
                CMNode.Topics.Pub.CameraInfoR.Msg.binning_x             = 1;
                CMNode.Topics.Pub.CameraInfoR.Msg.binning_y             = 1;

                /* Set the region of interest */
                CMNode.Topics.Pub.CameraInfoR.Msg.roi.x_offset          = 0;
                CMNode.Topics.Pub.CameraInfoR.Msg.roi.y_offset          = 0;
                CMNode.Topics.Pub.CameraInfoR.Msg.roi.height            = 0;
                CMNode.Topics.Pub.CameraInfoR.Msg.roi.width             = 0;
                CMNode.Topics.Pub.CameraInfoR.Msg.roi.do_rectify        = false;
            }
        }

        /* Publish CameraMono RSI data from CarMaker */
        if (CMNode.Sensor.CameraMono.Active) {
            /* Image Data */
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.CameraMono.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.CameraMono.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Get the sensor channel number in the RSDS data */
                int cidx = CMNode.Sensor.CameraMono.Channel;

                CData[cidx].encoding = "bgr8";
                CMNode.Topics.Pub.CameraMono.Msg = CData[cidx];

                int imsize = CData[cidx].width*CData[cidx].height;
                ptr = CMNode.Topics.Pub.CameraMono.Msg.data.data();
                for (int ii = 0; ii < imsize; ii++) {
                    /* Convert from RGB to BGR data */
                    tmp         = *ptr;
                    *(ptr)      = *(ptr+2);
                    *(ptr+2)    = tmp;
                    ptr += 3;
                }
                
                /* Frame header */
                CMNode.Topics.Pub.CameraMono.Msg.header.frame_id        = "Fr_CameraMono";
                CMNode.Topics.Pub.CameraMono.Msg.header.stamp           = ros::Time(SimCore.Time);
            }

            /* Camera Info */
            if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.CameraMonoInfo.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
                LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.CameraMonoInfo.Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Frame header */
                CMNode.Topics.Pub.CameraMonoInfo.Msg.header.frame_id    = "Fr_CameraMono";
                CMNode.Topics.Pub.CameraMonoInfo.Msg.header.stamp       = ros::Time(SimCore.Time);

                /* Get the sensor channel number in the RSDS data */
                int cidx = CMNode.Sensor.CameraMono.Channel;

                /* Extract the image size */
                CMNode.Topics.Pub.CameraMonoInfo.Msg.height             = CData[cidx].height;
                CMNode.Topics.Pub.CameraMonoInfo.Msg.width              = CData[cidx].width;
                maxd = (double)std::max(CData[cidx].height, CData[cidx].width);

                /* Set the image distortion model */
                CMNode.Topics.Pub.CameraMonoInfo.Msg.distortion_model   = "plump_bob";
                CMNode.Topics.Pub.CameraMonoInfo.Msg.D                  = {1, 0, 0, 0, 0};
                CMNode.Topics.Pub.CameraMonoInfo.Msg.K                  = { maxd*7/8,   0,          (double)CData[cidx].width/2, 
                                                                            0,          maxd*7/8,   (double)CData[cidx].height/2, 
                                                                            0,          0,          1
                                                                          };
                CMNode.Topics.Pub.CameraMonoInfo.Msg.R                  = { 1, 0, 0, 
                                                                            0, 1, 0, 
                                                                            0, 0, 1
                                                                          };
                CMNode.Topics.Pub.CameraMonoInfo.Msg.P                  = { maxd*7/8,   0,          (double)(CData[cidx].width)/2,      0, 
                                                                            0,          maxd*7/8,   (double)(CData[cidx].height)/2,     0, 
                                                                            0,          0,          1,                                  0
                                                                          };

                /* Set the subsampling values */
                CMNode.Topics.Pub.CameraMonoInfo.Msg.binning_x          = 1;
                CMNode.Topics.Pub.CameraMonoInfo.Msg.binning_y          = 1;

                /* Set the region of interest */
                CMNode.Topics.Pub.CameraMonoInfo.Msg.roi.x_offset       = 0;
                CMNode.Topics.Pub.CameraMonoInfo.Msg.roi.y_offset       = 0;
                CMNode.Topics.Pub.CameraMonoInfo.Msg.roi.height         = 0;
                CMNode.Topics.Pub.CameraMonoInfo.Msg.roi.width          = 0;
                CMNode.Topics.Pub.CameraMonoInfo.Msg.roi.do_rectify     = false;
            }
        }

        /* Publish vehicle velocity data from CarMaker */
        if ((rv = CMCRJob_DoPrep(CMNode.Topics.Pub.Velocity.Job, CMNode.CycleNoRel, 1, NULL, NULL)) < CMCRJob_RV_OK) {
            LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s", CMCRJob_GetName(CMNode.Topics.Pub.Velocity.Job), CMCRJob_RVStr(rv));
        } else if (rv == CMCRJob_RV_DoSomething) {
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
        tf2::Quaternion q;

        /* Only do anything if simulation is running */
        if (CMNode.Cfg.Mode == CMNode_Mode_Disabled || SimCore.State != SCState_Simulate) {
            return 0;
        }

        int rv;

        /* Publish SatNav messages */
        if (CMNode.Sensor.GNav.Active) {
            auto out_gps = &CMNode.Topics.Pub.GPS;

            if ((rv = CMCRJob_DoJob(out_gps->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_gps->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_gps->Pub.publish(out_gps->Msg);

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }
        }

        /* Publish LiDAR PointCloud2 messages */
        if (CMNode.Sensor.LidarRSI.Active) {
            auto out_lidar = &CMNode.Topics.Pub.LidarRSI;

            if ((rv = CMCRJob_DoJob(out_lidar->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_lidar->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_lidar->Pub.publish(out_lidar->Msg);
                CMNode.Topics.Pub.LidarRSI.Msg.data.clear();

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }
            /* Update the coordinate transformation between Fr1 and Fr_LidarRSI */
            CMNode.Sensor.LidarRSI.TF.header.stamp = ros::Time(SimCore.Time);
            CMNode.Global.TF_br->sendTransform(CMNode.Sensor.LidarRSI.TF);
        }

        /* Publish RADAR PointCloud2 messages */
        if (CMNode.Sensor.RadarRSI.Active) {
            auto out_radar = &CMNode.Topics.Pub.RadarRSI;

            if ((rv = CMCRJob_DoJob(out_radar->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_radar->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_radar->Pub.publish(out_radar->Msg);
                CMNode.Topics.Pub.RadarRSI.Msg.data.clear();

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }
            /* Update the coordinate transformation between Fr1 and Fr_RadarRSI */
            CMNode.Sensor.RadarRSI.TF.header.stamp = ros::Time(SimCore.Time);
            CMNode.Global.TF_br->sendTransform(CMNode.Sensor.RadarRSI.TF);
        }

        /* Publish CameraStereoL/CameraRGB messages */
        if (CMNode.Sensor.CameraStereoL.Active) {
            /* CameraStereoL Image */
            auto out_scaml = &CMNode.Topics.Pub.CameraStereoL;

            if ((rv = CMCRJob_DoJob(out_scaml->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_scaml->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_scaml->Pub.publish(out_scaml->Msg);
                //CMNode.Topics.Pub.CameraStereoL.Msg.data.clear();

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }

            /* CameraStereoL Info */
            auto out_scamli = &CMNode.Topics.Pub.CameraInfoL;

            if ((rv = CMCRJob_DoJob(out_scamli->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_scamli->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_scamli->Pub.publish(out_scamli->Msg);

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }

            /* CameraStereoRGB Image */
            auto out_rgbcam = &CMNode.Topics.Pub.CameraRGB;

            if ((rv = CMCRJob_DoJob(out_rgbcam->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_rgbcam->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_rgbcam->Pub.publish(out_rgbcam->Msg);
                //CMNode.Topics.Pub.CameraRGB.Msg.data.clear();

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }

            /* CameraStereoRGB Info */
            auto out_rgbcami = &CMNode.Topics.Pub.CameraInfoRGB;

            if ((rv = CMCRJob_DoJob(out_rgbcami->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_rgbcami->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_rgbcami->Pub.publish(out_rgbcami->Msg);

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }

            /* Update the coordinate transformation between Fr1 and Fr_CameraStereoL */
            CMNode.Sensor.CameraStereoL.TF.header.stamp = ros::Time(SimCore.Time);
            CMNode.Global.TF_br->sendTransform(CMNode.Sensor.CameraStereoL.TF);
        }

        /* Publish CameraStereoR messages */
        if (CMNode.Sensor.CameraStereoR.Active) {
            /* CameraStereoR Image */
            auto out_scamr = &CMNode.Topics.Pub.CameraStereoR;

            if ((rv = CMCRJob_DoJob(out_scamr->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_scamr->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_scamr->Pub.publish(out_scamr->Msg);
                //CMNode.Topics.Pub.CameraStereoR.Msg.data.clear();

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }

            /* CameraStereoR Info */
            auto out_scamri = &CMNode.Topics.Pub.CameraInfoR;

            if ((rv = CMCRJob_DoJob(out_scamri->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_scamri->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_scamri->Pub.publish(out_scamri->Msg);

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }

            /* Update the coordinate transformation between Fr1 and Fr_CameraStereoR */
            CMNode.Sensor.CameraStereoR.TF.header.stamp = ros::Time(SimCore.Time);
            CMNode.Global.TF_br->sendTransform(CMNode.Sensor.CameraStereoR.TF);
        }

        /* Publish CameraMono messages */
        if (CMNode.Sensor.CameraMono.Active) {
            /* CameraMono Image */
            auto out_mcam = &CMNode.Topics.Pub.CameraMono;

            if ((rv = CMCRJob_DoJob(out_mcam->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_mcam->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_mcam->Pub.publish(out_mcam->Msg);
                //CMNode.Topics.Pub.CameraMono.Msg.data.clear();

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }

            /* CameraMono Info */
            auto out_mcami = &CMNode.Topics.Pub.CameraMonoInfo;

            if ((rv = CMCRJob_DoJob(out_mcami->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
                LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_mcami->Job), CMCRJob_RVStr(rv));
            } else if (rv == CMCRJob_RV_DoSomething) {

                /* Publish message to output */
                out_mcami->Pub.publish(out_mcami->Msg);

                /* Remember cycle for debugging */
                CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
            }

            /* Update the coordinate transformation between Fr1 and Fr_CameraMono */
            CMNode.Sensor.CameraMono.TF.header.stamp = ros::Time(SimCore.Time);
            CMNode.Global.TF_br->sendTransform(CMNode.Sensor.CameraMono.TF);
        }

        /* Publish vehicle velocity messages */
        auto out_vel = &CMNode.Topics.Pub.Velocity;

        if ((rv = CMCRJob_DoJob(out_vel->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing && rv != CMCRJob_RV_DoSomething) {
            LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out_vel->Job), CMCRJob_RVStr(rv));
        } else if (rv == CMCRJob_RV_DoSomething) {

            /* Publish message to output */
            out_vel->Pub.publish(out_vel->Msg);

            /* Remember cycle for debugging */
            CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
        }

        /* Update the coordinate transformation between Fr0 and Fr1 */
        CMNode.Vhcl.TF.header.stamp = ros::Time(SimCore.Time);
        CMNode.Vhcl.TF.header.frame_id = "Fr0";
        CMNode.Vhcl.TF.child_frame_id = "Fr1";
        CMNode.Vhcl.TF.transform.translation.x = Car.Fr1.t_0[0];
        CMNode.Vhcl.TF.transform.translation.y = Car.Fr1.t_0[1];
        CMNode.Vhcl.TF.transform.translation.z = Car.Fr1.t_0[2];
        q.setRPY(Car.Fr1.r_zyx[0], Car.Fr1.r_zyx[1], Car.Fr1.r_zyx[2]);
        CMNode.Vhcl.TF.transform.rotation.x = q.x();
        CMNode.Vhcl.TF.transform.rotation.y = q.y();
        CMNode.Vhcl.TF.transform.rotation.z = q.z();
        CMNode.Vhcl.TF.transform.rotation.w = q.w();
        // CMNode.Global.TF_br->sendTransform(CMNode.Vhcl.TF);

        /* Publish "/clock" topic after all other other topics are published
        * - Is the order of arrival in other node identical? */
        if (CMNode.Cfg.nCyclesClock >= 0 && CMNode.CycleNoRel%CMNode.Cfg.nCyclesClock == 0) {
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
