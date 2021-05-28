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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#ifdef WIN32
#  include <winsock2.h>
#else
#  include <unistd.h>
#  include <sys/socket.h>
#  include <sys/types.h>
#  include <net/if.h>
#  include <netinet/in.h>
#  include <arpa/inet.h>
#  include <netdb.h>

#  include <pthread.h>
#  include <semaphore.h>
#endif

#include <Log.h>
#include <DataDict.h>
#include <SimCore.h>
#include <InfoUtils.h>

#include "rsds-client-camera.h"


static struct {
    char *MovieHost;  /* pc on which IPGMovie runs          */
    int   MoviePort;  /* TCP/IP port for RSDS               */
    int   sock;       /* TCP/IP Socket                      */
    char  sbuf[64];   /* Buffer for transmitted information */
    int   RecvFlags;  /* Receive Flags                      */
    int   Verbose;    /* Logging Output                     */
} RSDScfg;

struct {
    int         nImages;
    int         nBytes;
} RSDSIF;

struct RSData CData[MAX_CAM];

#ifdef WIN32
  HANDLE rsdsc_thread;
  HANDLE rsdsc_sem;
#else
  pthread_t rsdsc_thread;
  sem_t  rsdsc_sem;
#endif

static volatile int rsdsc_terminate;
static fd_set rsdsc_readfds;


static void *RSDS_Thread_Func (void *arg);
void PrintEmbeddedData (const char* data, unsigned int dataLen);



static int
RSDS_ReadChunk (char *buf, int count)
{
    int nread = 0, res;
    while (nread < count) {
	if (rsdsc_terminate)
	    return -1;

	FD_SET(RSDScfg.sock, &rsdsc_readfds);

	const int timeout_ms = 100;
	struct timeval tv;
	tv.tv_sec  =  timeout_ms / 1000;
	tv.tv_usec = (timeout_ms % 1000) * 1000;

	res = select(RSDScfg.sock+1, &rsdsc_readfds, NULL, NULL, &tv);
	if (res == 0) {
	    continue;
	} else if (res < 0) {
	    //Log("RSDS: socket read error: select() failed\n");
	    return -1;
	}

	res = recv(RSDScfg.sock, buf+nread, count-nread, RSDScfg.RecvFlags);
	if (res <= 0) {
	    Log("RSDS: socket read error: recv() failed\n");
	    return -1;
	}
	nread += res;
    }
    return 0;
}



/*
** RSDS_RecvHdr
**
** Scan TCP/IP Socket and writes to buffer
*/
static int
RSDS_RecvHdr (void)
{
    const int HdrSize = 64;
    char *hdr = RSDScfg.sbuf;
    int nSkipped = 0, len = 0, i;

    while (1) {
	if (RSDS_ReadChunk(hdr+len, HdrSize-len) != 0)
	    return -1;
	len = HdrSize;

	if (hdr[0]=='*' && hdr[1]>='A' && hdr[1]<='Z') {
	    /* remove whitespace at end of line */
	    while (len>0 && hdr[len-1]<=' ')
		len--;
	    hdr[len] = '\0';
	    if (RSDScfg.Verbose && nSkipped>0)
	        Log("RSDS: HDR resync, %d bytes skipped\n", nSkipped);
	    return 0;
	}

	for (i=1; i<len && hdr[i]!='*'; i++) ;
	len -= i;
	nSkipped += i;
	memmove(hdr, hdr+i, len);
    }
}



/*
** RSDS_Connect
**
** Connect over TCP/IP socket
*/
static int
RSDS_Connect (void)
{
    struct sockaddr_in	DestAddr;
    struct hostent  	*he;

    if ((he=gethostbyname(RSDScfg.MovieHost)) == NULL) {
        Log("RSDS: unknown host: %s\n", RSDScfg.MovieHost);
        return -2;
    }
    DestAddr.sin_family      = AF_INET;
    DestAddr.sin_port        = htons((unsigned short)RSDScfg.MoviePort);
    DestAddr.sin_addr.s_addr = *(unsigned *)he->h_addr;
    RSDScfg.sock = socket(AF_INET, SOCK_STREAM, 0);
    if (connect(RSDScfg.sock, (struct sockaddr *) &DestAddr, sizeof(DestAddr)) < 0) {
        Log ("RSDS: can't connect '%s:%d'\n", RSDScfg.MovieHost, RSDScfg.MoviePort);
        return -2;
    }
    if (RSDS_RecvHdr() < 0)
        return -3;

    Log("RSDS: connected: %s\n", RSDScfg.sbuf+1);

    return 0;
}



/*
** RSDS_GetData
**
** data and image processing
*/
int
RSDS_GetData (struct RSData *dt)
{
    /* Variables for Image Processing */
    char    ImgType[64];
    int     ImgWidth, ImgHeight, Channel;
    float   SimTime;
    int     ImgLen;

    if (sscanf(RSDScfg.sbuf, "*RSDS %d %s %f %dx%d %d", &Channel,
    ImgType, &SimTime, &ImgWidth, &ImgHeight, &ImgLen) == 6) {
        
        if (ImgLen > 0) {
            if (RSDScfg.Verbose)
                Log ("%6.3f %d: %8s %dx%d %d\n", SimTime, Channel, ImgType, ImgWidth, ImgHeight, ImgLen);
            
            if (strcmp(ImgType, "rgb") == 0) {
                char *img = (char *)malloc(ImgLen);
                if (RSDS_ReadChunk(img, ImgLen) != 0) {
                    free(img);
                    return -1;
                }
            
                strcpy(dt[Channel].ImgType, ImgType);
                dt[Channel].ImgWidth        = ImgWidth;
                dt[Channel].ImgHeight       = ImgHeight;
                dt[Channel].Channel         = Channel;
                dt[Channel].SimTime         = SimTime;
                dt[Channel].ImgLen          = ImgLen;
                dt[Channel].img             = img;

                if (RSDScfg.Verbose) {
                    /* Print general image information */
                    Log ("> %s\n", RSDScfg.sbuf);
                    Log ("\n");
                }
            free(img);
            }
        }
        RSDSIF.nImages++;
        RSDSIF.nBytes += ImgLen;
    } else {
        Log ("RSDS: not handled: %s\n", RSDScfg.sbuf);
    }

    return 0;
}


/*
** RSDS_Init
**
** Initialize Data Struct
** Create RSDS Thread
** Define DataDict Variables
*/
void
RSDS_Init (void)
{
    int i;

    RSDScfg.MovieHost = "localhost";
    RSDScfg.MoviePort = 2210;
    RSDScfg.Verbose   = 0;
    RSDScfg.RecvFlags = 0;

    RSDSIF.nImages  = 0;
    RSDSIF.nBytes   = 0;

    #ifdef WIN32
        /* Initialize Semaphore for Thread Synchronisation */
        rsdsc_sem = CreateSemaphore(NULL, 0, 1, NULL);

        /* Create Thread for Data Aquisition and Processing */
        rsdsc_thread = CreateThread(NULL, 100 * 1024,
    			       (LPTHREAD_START_ROUTINE)RSDS_Thread_Func, NULL, 0, NULL);
        i = rsdsc_thread==NULL ? -1 : 0;
    #else
        /* Initialize Semaphore for Thread Synchronisation */
        sem_init(&rsdsc_sem, 0, 0);

        /* Create Thread for Data Aquisition and Processing */
        pthread_attr_t attr;
        pthread_attr_init (&attr);
        pthread_attr_setstacksize(&attr, 100 * 1024);
        rsdsc_thread = -1;
        i = pthread_create(&rsdsc_thread, &attr, RSDS_Thread_Func, NULL);
        pthread_attr_destroy(&attr);
    #endif

    if (i != 0) {
        LogErrF(EC_General, "RSDS: unable to create background thread (returns %d, %s)\n",
                i, strerror(errno));
    }

    /* Define UAQ's to be used as Interface to CarMaker */
    DDefInt  (NULL, "RSDS.nImages",  "",  &RSDSIF.nImages,  DVA_None);
    DDefInt  (NULL, "RSDS.nBytes",   "",  &RSDSIF.nBytes,   DVA_None);
}


/*
** RSDS_Exit
**
** Signal termination request to background thread,
** wait for it to complete, then clean up.
*/
void
RSDS_Exit (void)
{
    rsdsc_terminate = 1;

    #ifdef WIN32
        ReleaseSemaphore(rsdsc_sem, 1, NULL);
        if (rsdsc_thread != NULL)
    	WaitForSingleObject(rsdsc_thread, INFINITE);

        CloseHandle(&rsdsc_sem);
    #else
    sem_post(&rsdsc_sem);
    if (rsdsc_thread != -1)
	pthread_join(rsdsc_thread, NULL);

    sem_destroy(&rsdsc_sem);
#endif
}


/*
** RSDS_Start
**
** Restart RSDS Thread if not already started
*/
void
RSDS_Start(void)
{
    #ifdef WIN32
        ReleaseSemaphore(rsdsc_sem, 1, NULL);
    #else
        int i;
        sem_getvalue (&rsdsc_sem, &i);
        if (i == 0)
            sem_post (&rsdsc_sem);
    #endif

    RSDScfg.MovieHost = iGetStrOpt(SimCore.TestRig.SimParam.Inf, "RSDS.MovieHost", "localhost");
    RSDScfg.MoviePort = iGetDblOpt(SimCore.TestRig.SimParam.Inf, "RSDS.MoviePort", 2210);
    RSDScfg.Verbose   = iGetDblOpt(SimCore.TestRig.SimParam.Inf, "RSDS.Verbose", 0);
}



/*
** Read_RSDS
**
** RSDS Thread
*/
static void *
RSDS_Thread_Func (void *arg)
{
    int i;
    //struct RSData *dmy;

    /* Wait (blocking) for semaphore */
    #ifdef WIN32
        while (!rsdsc_terminate && WaitForSingleObject(rsdsc_sem, INFINITE) != WAIT_FAILED) {
    #else
        while (!rsdsc_terminate && sem_wait(&rsdsc_sem) == 0) {
    #endif
	if (rsdsc_terminate)
	    break;

	/* Connect to RSDS Server */
	if ((i = RSDS_Connect()) != 0) {
	    LogWarnF(EC_General, "RSDS: unable to connect (returns %d, %s)\n",
		    i, strerror(errno));
	    continue;
	}

	/* Read from TCP/IP-Port and process the image */
	while (1) {
	    if (RSDS_RecvHdr() != 0)
		break;
	    if (RSDS_GetData(CData) != 0)
		break;
	}

    #ifdef WIN32
    	closesocket(RSDScfg.sock);
    #else
    	close(RSDScfg.sock);
    #endif
    }

    return NULL;
}
