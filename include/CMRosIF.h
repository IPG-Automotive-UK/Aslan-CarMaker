/*
 ******************************************************************************
 **  CarMaker - Version 6.0.3
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Important:
 * - Prototype/Proof of Concept
 * - Unsupported ROS Example with CarMaker
 *
 */

#ifndef CMROSIF_H_
#define CMROSIF_H_

#ifdef __cplusplus
extern "C" {
#endif

struct tInfos;

/*!
 * Description:
 * - Call this function e.g. in User.c before functions below are called
 * - Loading shared library with CarMaker ROS Node
 * - Mapping of functions from shared library to functions below
 * - General parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
 * - Returns >0 on success, 0 if interface is disabled
 * - Return value corresponds to CarMaker ROS Node library init if interface is enabled
 *
 */
int CMRosIF_Init                     (void);



/*!
 * Description:
 * - Call these functions in corresponding "User_*" in User.c
 * - Not all functions have to be set in User.c
 * - CMRosIF_Init() must be called first!
 * - e.g. put the define "-DWITH_CMROSIF" to Makefile and following to source code
 *    #if defined(WITH_CMROSIF)
 *       rv = CMRosIF_TestRun_Start_atBegin();
 *    #endif
 * - These functions call the CMRosIF_CMNode_* functions in CarMaker ROS Node library
 *
 */
int CMRosIF_DeclQuants            (void);
int CMRosIF_TestRun_Start_atBegin (void);
int CMRosIF_TestRun_RampUp        (void);
int CMRosIF_TestRun_End           (void);
int CMRosIF_In                    (void);
int CMRosIF_DrivMan_Calc          (double dt);
int CMRosIF_VehicleControl_Calc   (double dt);
int CMRosIF_Calc                  (double dt);
int CMRosIF_Out                   (void);
int CMRosIF_End                   (void);



/*!
 * Description:
 * - Get non-default functions defined by user from library with CarMaker ROS Node
 * - Wrapper around CM_GetSymbol (see CarMaker EnvUtils.h for more information)
 * - CMRosIF_Init() must be called before
 * - returns NULL if symbol was not found
 *
 * Arguments:
 * - Name = Symbol name in shared library
 *
 */
void* CMRosIF_GetSymbol           (const char *Name);



#ifdef __cplusplus
}
#endif

#endif /* CMROSIF_H_ */
