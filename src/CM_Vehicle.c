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
** Functions Overview
** ==================
**
**	Vhcl_ModelCheck_BeforePre()
**	Vhcl_ModelCheck_AfterPre()
**	Vhcl_Param_SetDummy()
**	Vhcl_Init()
**	Vhcl_Register()
**	Vhcl_DeclQuants()
**	Vhcl_New()
**	Vhcl_StaticCond_Calc()
**	Vhcl_Calc()
**	Vhcl_Calc_Part()			(CM4SL only)
**	Vhcl_Snapshot_Take()
**	Vhcl_Snapshot_Export2Inf()
**	Vhcl_Delete()
**	Vhcl_Cleanup()
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32)
#  include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <CarMaker.h>


# include <Car/Vehicle_Car.h>

# include <Vehicle/VehicleControlApps.h>

#include "User.h"


/*
** Vhcl_ModelCheck_BeforePre()
**
** before preprocessing
*/

int
Vhcl_ModelCheck_BeforePre (struct tInfos *Inf)
{
    ModelCheck_BeforePre_Begin();
    if (Vehicle.Model.Source == VehicleSource_BuiltIn &&
	Car_ModelCheck_BeforePre(Inf) < 0)
	goto ErrorReturn;
    if (!BrakeDisabled && Brake_ModelCheck_BeforePre(Inf) < 0)
	goto ErrorReturn;
    if (!PowerTrainDisabled && PowerTrain_ModelCheck_BeforePre(Inf) < 0)
	goto ErrorReturn;
    if (Trailer_ModelCheck_BeforePre(Inf) < 0)
	goto ErrorReturn;
    ModelCheck_End();
    return 0;

    ErrorReturn:
	ModelCheck_End();
	return -1;
}



/*
** Vhcl_ModelCheck_AfterPre()
**
** after preprocessing
*/

int
Vhcl_ModelCheck_AfterPre (struct tInfos *Inf)
{
    ModelCheck_AfterPre_Begin();
    if (Vehicle.Model.Source == VehicleSource_BuiltIn &&
	Car_ModelCheck_AfterPre(Inf) < 0)
	goto ErrorReturn;
    if (SimCore.Trailer.nTrailers != 0 &&
	Trailer_ModelCheck_AfterPre(Inf) < 0)
	goto ErrorReturn;
    ModelCheck_End();
    return 0;

    ErrorReturn:
	ModelCheck_End();
	return -1;
}



/*
** Vhcl_Param_SetDummy()
**
**
*/

void
Vhcl_Param_SetDummy (struct tInfos *Inf)
{
    Car_Param_SetDummy(Inf);
}



/*
** Vhcl_Init()
**
** Initialize all vehicle module components
**
** called:
** - once at program start
**
*/

int
Vhcl_Init (void)
{
    VehicleStruct_Init();

    Car_Init();
    Brake_Init();
    PowerTrain_Init();
    Trailer_Init();
    TrBrake_Init();

    return 0;
}



/*
** Vhcl_Register()
**
** Register vehicle sub models
**
** called:
** - once at program start
*/

int
Vhcl_Register (void)
{
    SimCore_SetVhclClass(VhclClass_Car_Id);

    Steering_Register_Dummy();		/* Dummy model, don't remove!!! */
    Tire_Register_Dummy();		/* Dummy model, don't remove!!! */
    Aero_Register_Dummy();		/* Dummy model, don't remove!!! */

    Susp_KnC_Register();
    Susp_FrcEl_Register();


    Steering_Register_GenAngle();
    Steering_Register_GenTorque();
    Steering_Register_Pfeffer();

    Env_Register_Generic();

    Aero_Register_Coeff6x1();

    Tire_Register_RTTire();
    Tire_Register_MagicFormula_52();
    Tire_Register_MagicFormula_61();
    Tire_Register_TASS_MF();
    Tire_Register_FTire();
    Tire_Register_TameTire();
    Tire_Register_IPGTire();


    Brake_Register();

    PowerTrain_Register();

    VC_Register_AccelCtrl   ();
    VC_Register_GenLatCtrl  ();
    VC_Register_GenLongCtrl ();

    VhclOperator_Register_IPGOperator();

    TrfFollow_Register ();
    TrfLaneChange_Register ();

    return 0;
}



/*
** Vhcl_DeclQuants()
**
** Add quantities to CarMaker's data dictionary
**
** called:
** - once at program start
*/

int
Vhcl_DeclQuants (void)
{
    VehicleStruct_DeclQuants();

    if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	Car_DeclQuants();
	if (SimCore.Trailer.nTrailers != 0) {
	    Trailer_DeclQuants();
	    TrBrake_DeclQuants();
	}
    } else if (Vehicle.Model.Source == VehicleSource_ModelMgr) {
	VhclModel_DeclQuants();
    }

    Brake_DeclQuants();
    PowerTrain_DeclQuants();


    Model_ExportConfig(SimCore.Config.Inf);
    return 0;
}




/*
** Vhcl_New()
**
** Prepare the vehicle model for a new TestRun.
**
** called:
** - each TestRun start
*/

int
Vhcl_New (void)
{
    int rv = 0;

    if (!SimCore.Vhcl.Modified) {
	Vhcl_Delete (1);
	if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	    if (Car_SoftNew (SimCore.Vhcl.Inf) < 0)
		rv |= 1<<1;
	}
	if (SimCore.Trailer.nTrailers != 0) {
	    if (Trailer_SoftNew () < 0)
		rv |= 1<<5;
	}

	Vehicle_CreateInstrMsg(SimCore.Vhcl.Inf);

	return rv > 0 ? -rv : 0;
    }

    /*** free existing vehicle before starting a new one */
    Vhcl_Delete (0);

    Vehicle.Model.Source = Vehicle.Model.NextSource;
    SteeringDisabled     = Vehicle_SubsystemDisabled(SimCore.Vhcl.Inf, "Steering");
    BrakeDisabled        = Vehicle_SubsystemDisabled(SimCore.Vhcl.Inf, "Brake");
    PowerTrainDisabled   = Vehicle_SubsystemDisabled(SimCore.Vhcl.Inf, "PowerTrain");

    Vehicle_InitVhclCfg(SimCore.Vhcl.Class);
    Vehicle_RegisterMdlFrames();

    if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	if (Car_New(SimCore.Vhcl.Inf) < 0)
	    rv |= 1<<1;
    } else if (Vehicle.Model.Source == VehicleSource_ModelMgr) {
	if (VhclModel_New(SimCore.Vhcl.Inf) != 0)
	    rv |= 1<<1;
    } else {
	LogErrF(EC_Init, "Vehicle dataset: Unsupported vehicle kind");
	rv |= 1<<1;
    }

    if (!BrakeDisabled) {
	if (Brake_New(SimCore.Vhcl.Inf) < 0)
	    rv |= 1<<3;
    }

    if (!PowerTrainDisabled) {
	if (PowerTrain_New(SimCore.Vhcl.Inf) < 0)
	    rv |= 1<<4;
    } else {
	if (Vehicle_GetVhclCfg_PT(SimCore.Vhcl.Inf) < 0)
	    rv |= 1<<4;
	if (PowerTrain_Reduced_New(SimCore.Vhcl.Inf) < 0)
	    rv |= 1<<4;
    }

    if (SimCore.Trailer.nTrailers != 0) {
	if (Trailer_New() < 0)
	    rv |= 1<<5;

	if (!TrBrakeDisabled) {
	    if (TrBrake_New() < 0)
		rv |= 1<<6;
	}
    }


    Vehicle_CreateInstrMsg(SimCore.Vhcl.Inf);

    return rv > 0 ? -rv : 0;
}



/*
** Vhcl_StaticCond_Calc()
**
** Get the vehicle into static conditions
**
** called:
** - each TestRun start
*/

int
Vhcl_StaticCond_Calc (void)
{
    if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	if (Car_StaticCond_Calc() < 0)
	    return -1;
    }
    if (SimCore.Trailer.nTrailers != 0) {
	if (Trailer_StaticCond_Calc() < 0)
	    return -3;

	if (CarAndTrailer_StaticCond_Calc() < 0)
	    return -4;
    }

    return 0;
}



/*
** Vhcl_Calc()
**
** called:
** - each simulation cycle
*/

int
Vhcl_Calc (double dt)
{
    int rv = 0;

    if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	if (Car_Calc(dt) < 0)
	    rv = -1;
    } else if (Vehicle.Model.Source == VehicleSource_ModelMgr) {
	if (VhclModel_Calc(dt) < 0)
	    rv = -1;
    }
    SimCore_TCPU_TakeTS(&SimCore.TS.Vehicle);

    if (SimCore.Trailer.nTrailers != 0) {
	if (Trailer_Calc(dt) < 0)
	    rv = -2;

	SimCore_TCPU_TakeTS(&SimCore.TS.Trailer);
    } else {
	SimCore.TS.Trailer = SimCore.TS.Vehicle;
    }



    if (!BrakeDisabled) {
	if (Brake_Calc(dt) < 0)
	    rv = -3;
    } else {
	Brake_CalcPost(dt);
    }
    if (SimCore.Trailer.nTrailers != 0) {
	if (TrBrake_Calc(dt) < 0)
	    rv = -5;
    }
    if (User_Brake_Calc(dt) < 0)
	rv = -6;

    SimCore_TCPU_TakeTS(&SimCore.TS.Brake);


    if (!PowerTrainDisabled) {
	if (PowerTrain_Calc(dt) < 0)
	    rv = -4;
    } else {
	PowerTrain_CalcPost(dt);
    }
    SimCore_TCPU_TakeTS(&SimCore.TS.PowerTrain);

    PowerFlow_Calc();


    return rv;
}




/*
** Vhcl_Snapshot_Take()
**
** Takes a snapshot of the actual vehicle state.
*/

int
Vhcl_Snapshot_Take (void)
{
    if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	return Car_Snapshot_Take();
    } else {
	return 0;
    }
}



/*
** Vhcl_Snapshot_Export2Inf()
**
** Exports all important informations to the model snapshot info handle.
*/

int
Vhcl_Snapshot_Export2Inf (void)
{
    if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	return Car_Snapshot_Export2Inf();
    } else {
	return 0;
    }
}



/*
** Vhcl_TestRun_End_Delete()
**
** called:
** - once, to release resources of parts of the vehicle model
** - at the end of a test run
** - called from App_TestRun_End()
*/

int
Vhcl_TestRun_End_Delete (void)
{
    return 0;
}



/*
** Vhcl_Delete()
**
** called:
** - once, to release vehicle model resources
** - at the beginning before Vhcl_New() really starts
** - called from App_TestRun_End()
*/

int
Vhcl_Delete (int soft)
{
    /* Soft delete: Only parts from vehicle, which are connected with other modules
       outside the vehicle module (e.g: Road-Handle) */
    if (soft) {
	if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	    Car_SoftDelete ();
	}
	Trailer_SoftDelete ();
	return 0;
    }

    VehicleStruct_Delete ();

    if (Vehicle.Model.Source == VehicleSource_BuiltIn) {
	Car_Delete();
    } else if (Vehicle.Model.Source == VehicleSource_ModelMgr) {
	VhclModel_Delete();
    }

    Trailer_Delete();
    if (!TrBrakeDisabled)
	TrBrake_Delete();

    if (!BrakeDisabled)
	Brake_Delete();

    if (!PowerTrainDisabled)
	PowerTrain_Delete();

    return 0;
}



/*
** Vhcl_Cleanup()
**
** free all vehicle module components
**
** called:
** - once at end of program
*/

void
Vhcl_Cleanup (void)
{
    /* !!! Don't change the order of the following function calls !!! */

    Trailer_Cleanup();
    if (!BrakeDisabled)
	Brake_Cleanup();
    if (!TrBrakeDisabled)
	TrBrake_Cleanup();
    if (!PowerTrainDisabled)
	PowerTrain_Cleanup();
    Car_Cleanup();

}

