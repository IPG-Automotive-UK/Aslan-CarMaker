/*
 ******************************************************************************
 **  CarMaker - Version 7.1.2
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 */

#ifndef CMROSIF_UTILS_H_
#define CMROSIF_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************
 *
 * CarMaker Cyclic Job (for CarMaker ROS Interface)
 *
 * Description:
 * - Create objects usable for jobs to be done cyclic with fixed step size
 * - Different Job modes are available
 * - Possible use cases
 *   - Run a function each 5ms starting at beginning of simulation
 *   - Copy data in a defined cycle but wait for some data of external program
 *   - Synchronization of multiple tasks
 *
 ******************************************************************************/


struct tCMCRJob;



typedef enum tCMCRJob_Mode {
    CMCRJob_Mode_Disabled = 0,
    CMCRJob_Mode_Default  = 1,  /* Default job mode, only "DoJob*()" is necessary */
    CMCRJob_Mode_Ext      = 2   /* Extended job mode, "DoPrep*()" and "DoJob*()" is necessary */
} tCMCRJob_Mode;



typedef enum tCMCRJob_RV {
    /* General return values */
    CMCRJob_RV_ErrTime     = -3, /*!< E.g. Time for Job is in past          */
    CMCRJob_RV_ErrInvalid  = -2, /*!< E.g. Job is disabled or in wrong mode */
    CMCRJob_RV_Error       = -1, /*!< General return value on error         */
    CMCRJob_RV_OK          =  0, /*!< Non-critical                          */
    CMCRJob_RV_Success     =  1, /*!< General return value on success       */

    /* Special return values for DoJob() and DoPrep() */
    CMCRJob_RV_DoNothing   =  2, /*!< Currently nothing to do               */
    CMCRJob_RV_DoSomething =  3, /*!< User action is necessary              */
} tCMCRJob_RV;



/*!
 * Description:
 * - Optional function pointer
 * - Can be used to create callbacks for DoJob() and DoPrep()
 */
typedef void (*tCMCRJob_DoFunc) (struct tCMCRJob *Job, void *Data);



/*!
 * Description:
 * - Create a job object to realize cyclic jobs
 * - Allocates memory for a new job
 * - Created jobs have to be deleted properly
 * - Return Values
 *   - NULL pointer on error
 *   - Valid pointer to Job object on success
 *
 * Arguments:
 * - Name = Name of the job to be created
 *          - Used for logging and can be used by user to identify the current job object
 */
struct tCMCRJob* CMCRJob_Create (const char *Name);



/*!
 * Description:
 * - Free the memory after the job is not necessary anymore
 */
void CMCRJob_Delete (struct tCMCRJob *Job);



/*!
 * Description:
 * - Initialize a previously created job
 * - Re-Initialize an already finished job (e.g. every time a new simulation starts)
 * - TimeStart=5, TimeStep_ms=10 will cause a job to be executed at time={5, 15, 25, ...}ms
 *
 * Arguments:
 * - Job          = Pointer to a valid job
 * - TimeStart_ms = First time the job needs to be executed in milliseconds
 * - TimeStep_ms  = Step size for a cyclic job to be executed in milliseconds
 *                  - Must be greater than zero!
 * - Mode         = The job can act in different modes
 *                  - 0 = Disabled
 *                  - 1 = Default mode
 *                        - Use this mode if a simple cyclic job has to be done
 *                          - e.g. calculate an algorithm each 10ms
 *                        - Just call DoJob() to check if the real job needs to be done
 *                        - Calls of DoPrep*() are not allowed!
 *                  - 2 = Extended mode
 *                        - Use this mode if a job expects an additional preparation before
 *                          - e.g. calculate an algorithm each 10ms only when other data has been provided
 *                          - the preparation might finish at a time before the real job, so it might be
 *                            necessary to use a buffer to provide the data when the real job starts
 *                        - A preparation needs to be finished before the job is allowed to start!
 *                        - Call DoPrep() to check if the preparation needs to be done
 *                          . A Preparation needs to be finished automatically or manually via DoPrep_SetDone()
 *                        - Afterwards call *DoJob() to check if the real job needs to be done
 */
tCMCRJob_RV CMCRJob_Init (struct tCMCRJob *Job, unsigned long TimeStart_ms, unsigned long TimeStep_ms, tCMCRJob_Mode Mode);



/*!
 * Description:
 * - Check if at current time a job needs to be done
 * - A callback function can be used to be called automatically when job is necessary
 * - Return Values
 *   - "0" = Currently no job is necessary
 *   - "1" = Job is or was necessary dependent on argument "AutoDone"
 *
 * Arguments:
 * - Job      = Pointer to a valid job
 * - Time_ms  = Current time in milliseconds
 * - AutoDone = Flag to update job state when preparation is necessary
 *              - 0 = Do not update the job state automatically, user has to call *DoPrep_SetDone() manually
 *              - 1 = Update the job state automatically
 * - Func     = User function that is called automatically when preparation is necessary
 *              - NULL = No user function
 * - FuncData = Data pointer that can be provided to user function
 *              - NULL = No data
 */
tCMCRJob_RV CMCRJob_DoJob (struct tCMCRJob *Job, unsigned long Time_ms, char AutoDone, tCMCRJob_DoFunc Func, void *FuncData);

/*!
 * Description:
 * - Almost identical to CMCRJob_DoJob
 * - Throw CarMaker Log Error that normally stops the simulation
 * - Return Values
 *   - "-1" = Error occured
 *   -  "0" = Currently no job is necessary
 *   -  "1" = Job is or was necessary dependent on argument "AutoDone"
 */
int CMCRJob_DoJob_LogErr (struct tCMCRJob *Job, unsigned long Time_ms, char AutoDone, tCMCRJob_DoFunc Func, void *FuncData, const char* Prefix);

/*!
 * Description:
 * - Call this function to confirm that the job has finished
 * - Calculates the new cycle for next job
 *
 * Arguments:
 * - Job      = Pointer to a valid job
 * - Time_ms  = Current time in milliseconds
 */
tCMCRJob_RV CMCRJob_DoJob_SetDone (struct tCMCRJob *Job, unsigned long Time_ms);



/*!
 * Description:
 * - Use the *DoPrep() function only in extended mode!
 * - Check if at current time a preparation needs to be done
 * - A callback function can be used to be called automatically when preparation is necessary
 *
 * Arguments:
 * - Job      = Pointer to a valid job
 * - Time_ms  = Current time in milliseconds
 * - AutoDone = Flag to update job state when preparation is necessary
 *              - 0 = Do not update the job state automatically, user has to call *DoPrep_SetDone() manually
 *              - 1 = Update the job state automatically
 * - Func     = User function that is called automatically when preparation is necessary
 *              - NULL = No user function
 * - FuncData = Data pointer that can be provided to user function
 *              - NULL = No data
 *
 */
tCMCRJob_RV CMCRJob_DoPrep (struct tCMCRJob *Job, unsigned long Time_ms, char  AutoDone, tCMCRJob_DoFunc Func, void *FuncData);


/*!
 * Description:
 * - Call this function to confirm that the preparation has finished
 *
 * Arguments:
 * - Job      = Pointer to a valid job
 * - Time_ms  = Current time in milliseconds
 *
 */
tCMCRJob_RV CMCRJob_DoPrep_SetDone (struct tCMCRJob *Job, unsigned long Time_ms);



/* Misc */
const char*     CMCRJob_RVStr              (int ReturnValue);
int             CMCRJob_DoJob_GetLastDone  (struct tCMCRJob *Job);
int             CMCRJob_DoPrep_GetLastDone (struct tCMCRJob *Job);
void            CMCRJob_Info               (struct tCMCRJob *Job, unsigned long CycleNo, const char *Prefix);
const char*     CMCRJob_GetName            (struct tCMCRJob *Job);



#ifdef __cplusplus
}
#endif

#endif /* CMROSIF_UTILS_H_ */
