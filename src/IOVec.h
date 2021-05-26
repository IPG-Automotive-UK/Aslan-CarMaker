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
*/

#ifndef _IO_H__
#define _IO_H__

#ifdef __cplusplus
extern "C" {
#endif

struct tInfos;

/*** Input Vector, signals from hardware, ... */
typedef struct {
    double	T;
    float	DeltaT;			/* DeltaT of the last time step */
} tIOVec;

extern tIOVec IO;


/*** I/O configuration */

/* extern int IO_None; DON'T - Variable is predefined by CarMaker! */
extern int IO_CAN_IF;
#if !defined(LABCAR)
extern int IO_FlexRay;
#endif


/*** I/O calibration */

typedef struct tCal {
    float	Min;
    float	Max;
    float	LimitLow;
    float	LimitHigh;
    float	Factor;
    float	Offset;
    int		Rezip;
} tCal;


void	iGetCal	(struct tInfos *Inf, const char *key, tCal *cal, int optional);
float	CalIn   (tCal *cal, int   Value);
float	CalInF  (tCal *cal, float Value);
int	CalOut	(tCal *cal, float Value);
float	CalOutF (tCal *cal, float Value);
int 	LimitInt (float fValue, int Min, int Max);


int	IO_Init_First (void);
int	IO_Init (void);
int	IO_Init_Finalize (void);

int	IO_Param_Get (struct tInfos *inf);
void	IO_BeginCycle (void);
void	IO_In  (unsigned CycleNo);
void	IO_Out (unsigned CycleNo);

void	IO_Cleanup (void);


#ifdef __cplusplus
}
#endif

#endif	/* #ifndef _IO_H__ */

