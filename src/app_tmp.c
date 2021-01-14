/*
******************************************************************************
**  CarMaker
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32)
#  include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_DSRT)
#  include <DsApplicationInterface.h>
#endif

#include <infoc.h>
#include <CarMaker.h>
#include <ipgdriver.h>
#include <road.h>

extern const char *SetConnectedIO (const char *io);

static const char *CompileLibs[] = {
    /* /opt/ipg/carmaker/linux64-9.1/lib/libcar.a */
    /* /opt/ipg/carmaker/linux64-9.1/lib/libcarmaker.a */
    /* /opt/ipg/carmaker/linux64-9.1/lib/libipgdriver.a */
    /* /opt/ipg/carmaker/linux64-9.1/lib/libipgroad.a */
    /* /opt/ipg/carmaker/linux64-9.1/lib/libipgtire.a */
    "libcar.a	CarMaker-Car linux64 9.1 2020-10-21",
    "libcarmaker.a	CarMaker linux64 9.1 2020-10-21",
    "libipgdriver.a	IPGDriver linux64 9.1 2020-10-20",
    "libipgroad.a	IPGRoad linux64 9.1 2020-10-20",
    "libipgtire.a	IPGTire linux64 9.0.2 2020-06-29",
    NULL
};


static const char *CompileFlags[] = {
    "-m64 -fPIC -O3 -DNDEBUG -DLINUX -DLINUX64 -D_GNU_SOURCE",
    "-D_FILE_OFFSET_BITS=64 -DCM_NUMVER=90100",
    "-DWITH_CMROSIF",
    "-I/opt/ipg/carmaker/linux64-9.1/include",
    "-I../include -Wall -Wimplicit -Wmissing-prototypes",
    "-rdynamic",
    NULL
};


tAppStartInfo   AppStartInfo = {
    "Car_Generic with CMRosIF",          /* App_Version         */
    "54",          /* App_BuildVersion    */
    "bogotm",     /* App_CompileUser     */
    "desktop-t8gfiu5",         /* App_CompileSystem   */
    "2021-01-14 10:28:35",  /* App_CompileTime */

    CompileFlags,                /* App_CompileFlags  */
    CompileLibs,                 /* App_Libs          */

    "9.1",          /* SetVersion        */

    NULL,           /* TestRunName       */
    NULL,           /* TestRunFName      */
    NULL,           /* TestRunVariation  */
    NULL,           /* LogFName          */

    0,              /* SaveMode          */
    0,              /* OnErrSaveHist     */

    0,              /* Verbose           */
    0,              /* Comments          */
    0,              /* ModelCheck        */
    0,              /* Snapshot          */
    0,              /* DriverAdaption    */
    0,              /* Log2Screen        */
    0,              /* ShowDataDict      */
    0,              /* DontHandleSignals */
    {0, 0, NULL},   /* Suffixes          */
    {0, 0, NULL}    /* Keys              */
};



void
App_InfoPrint (void)
{
    int i;
    Log ("App.Version\t%s #%s (%s)\n",
            AppStartInfo.App_Version,
            AppStartInfo.App_BuildVersion,
            SimCoreInfo.Version);
    Log ("App.Compiled\t%s@%s %s\n",
            AppStartInfo.App_CompileUser,
            AppStartInfo.App_CompileSystem,
            AppStartInfo.App_CompileTime);

    i = 0;
    Log ("App.CompileFlags:\n");
    while (AppStartInfo.App_CompileFlags != NULL
        && AppStartInfo.App_CompileFlags[i] != NULL) {
        Log ("			%s\n", AppStartInfo.App_CompileFlags[i++]);
    }

    i = 0;
    Log ("App.Libs:\n");
    while (AppStartInfo.App_Libs != NULL && AppStartInfo.App_Libs[i] != NULL)
        Log ("			%s\n", AppStartInfo.App_Libs[i++]);

    /* Security */
    i = 0;
    Log ("App.Suffixes:\n");
    while (AppStartInfo.Suffix.List != NULL && AppStartInfo.Suffix.List[i] != NULL)
        Log ("			%s\n", AppStartInfo.Suffix.List[i++]);

    i = 0;
    Log ("App.Keys:\n");
    while (AppStartInfo.Key.List != NULL && AppStartInfo.Key.List[i] != NULL)
        Log ("			%s\n", AppStartInfo.Key.List[i++]);


    /*** Linked libraries */
    Log ("App.Version.Driver =\t%s\n",  IPGDrv_LibVersion);
    Log ("App.Version.Road =\t%s\n",    RoadLibVersion);
}




int
App_ExportConfig (void)
{
    int        i, n;
    char       *txt[42], sbuf[512];
    char const *item;
    tInfos *inf = SimCore.Config.Inf;

    InfoSetStr (inf, "Application.Version",       AppStartInfo.App_Version);
    InfoSetStr (inf, "Application.BuildVersion",  AppStartInfo.App_BuildVersion);
    InfoSetStr (inf, "Application.CompileTime",   AppStartInfo.App_CompileTime);
    InfoSetStr (inf, "Application.CompileUser",   AppStartInfo.App_CompileUser);
    InfoSetStr (inf, "Application.CompileSystem", AppStartInfo.App_CompileSystem);
    if (AppStartInfo.App_CompileFlags != NULL)
        InfoSetTxt (inf, "Application.CompileFlags",
                    (char**)AppStartInfo.App_CompileFlags);
    InfoAddLineBehind (inf, NULL, "");
    if (AppStartInfo.App_Libs != NULL)
        InfoSetTxt (inf, "Application.Libs",
                    (char**)AppStartInfo.App_Libs);
    InfoAddLineBehind (inf, NULL, "");

    /*** Linked libraries */
    InfoSetStr (inf, "Application.Version.Driver",  IPGDrv_LibVersion);
    InfoSetStr (inf, "Application.Version.Road",    RoadLibVersion);
    InfoAddLineBehind (inf, NULL, "");

    /*** I/O configuration */
    IO_ListNames(sbuf, -1);

    n = 0;
    txt[n] = NULL;
    while (1) {
	item = strtok((n==0 ? sbuf : NULL), " \t");
	if (item == NULL || n >= 42-1)
	    break;

	txt[n++] = strdup(item);
	txt[n] =   NULL;
    }

    InfoSetTxt (inf, "IO.Configs", txt);
    InfoAddLineBehind (inf, NULL, "");

    for (i=0; i < n; i++)
	free (txt[i]);

    return 0;
}


#if defined(_DS1006)
void
IPGRT_Board_Init (void)
{
    init();
}
#endif

