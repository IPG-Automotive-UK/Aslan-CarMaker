/*!
 ******************************************************************************
 **  CarMaker ROS Utilities
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Description:
 * - General templates, structs,... usable in ROS environment
 * - Independent from specific node/messages!
 * - Most functionality can be used for CarMaker dependent ROS Node (CMNode) and independent external ROS Nodes
 * - Some functionality is only designed for CMNode!
 *
 * Important:
 * - Prototype/Proof of concept!
 * - Unsupported ROS Example with CarMaker
 *
 * ToDo:
 * - Namespace and type renaming
 *
 */

#ifndef CMROSUTILS_H_
#define CMROSUTILS_H_


#ifdef __cplusplus

/*****************************
 *   Commonly used headers   *
 *****************************/

#  include "ros/ros.h"
#  include "rosgraph_msgs/Clock.h" /*!< Necessary for use_sim_time */


struct tCMCRJob; /*!< Only for CarMaker ROS Node!!! Functions located in library for CarMaker ROS Interface */



/********************************
 *   Templates for ROS Topics   *
 ********************************/

/*!
 * Description:
 * - This template can be used to manage common tasks for a Topic to be published
 *
 */
template <typename RosIF_TpcMsgPubT>
struct tRosIF_TpcPub
{
    ros::Publisher    Pub;     /*!< Publisher handle */
    RosIF_TpcMsgPubT  Msg;     /*!< Message buffer for Publisher */
    struct tCMCRJob   *Job;    /*!< Only for CMNode! Cyclic job related to this topic */
    int               CycleTime;
    int               CycleOffset;
};



/*!
 * Description:
 * - This template can be used to manage common tasks for a Topic to be subscribed
 *
 */
template <typename RosIF_TpcMsgSubT>
struct tRosIF_TpcSub
{
    ros::Subscriber   Sub;    /*!< Subscriber handle */
    RosIF_TpcMsgSubT  Msg;    /*!< Message buffer for Subscription */
    struct tCMCRJob   *Job;   /*!< Only for CMNode! Cyclic job related to this topic */
    int               CycleTime;
    int               CycleOffset;
};



/*********************************
 *   Template for ROS Services   *
 *********************************/

/*!
 * Description:
 * - This template can be used to manage common tasks for a ROS Service
 * - A node can be Service Server OR Client! */
template <typename RosIF_SrvMsgT>
struct tRosIF_Srv
{
    ros::ServiceServer Srv;   /*!< Service handle for Service Server */
    ros::ServiceClient Clnt;  /*!< Service handle for Service Client */
    RosIF_SrvMsgT      Msg;   /*!< Message buffer for Service*/
};



/***************************************
 *   Templates for ROS Configuration   *
 ***************************************/

typedef struct tRosIF_Cfg {
    ros::NodeHandlePtr Node;  /*!< Node Handle for current process */
} tRosIF_Cfg;


#else
/* ToDo:
   - C-Interface relevant?
   - Only ROS2?
*/
#endif


#endif /* CMROSUTILS_H_ */
