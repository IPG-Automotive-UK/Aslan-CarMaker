/*
******************************************************************************
**  CarMaker - Version 10.0
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Raw Signal Data Stream example client for IPGMovie 3.4 and later versions.
*/

#include <stdint.h>
#include "sensor_msgs/Image.h"                          /* ROS Image for camera sensor inputs */


#ifndef __RSDS_CLIENT_H__
#define __RSDS_CLIENT_H__

#define MAX_CAM 6

#ifdef __cplusplus
extern "C" {
#endif

extern sensor_msgs::Image CData[];

void RSDS_Init  (void);
void RSDS_Start (void);
void RSDS_Exit  (void);
int RSDS_GetData(sensor_msgs::Image *dt);

#ifdef __cplusplus
}
#endif

#endif /* __VDS_CLIENT_H__ */
