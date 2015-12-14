//=============================================================================
// Copyright © 2014 NaturalPoint, Inc. All Rights Reserved.
// 
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall NaturalPoint, Inc. or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//=============================================================================


/*

SampleClient.cpp

This program connects to a NatNet server, receives a data stream, and writes that data stream
to an ascii file.  The purpose is to illustrate using the NatNetClient class.

Usage [optional]:

	SampleClient [ServerIP] [LocalIP] [OutputFilename]

	[ServerIP]			IP address of the server (e.g. 192.168.0.107) ( defaults to local machine)
	[OutputFilename]	Name of points file (pts) to write out.  defaults to Client-output.pts

*/

#include <stdio.h>
#include <tchar.h>
#include <conio.h>
#include <winsock2.h>
#include <math.h>
#include <string>
#include <SerialClass.h>
#include <sstream>

#include "NatNetTypes.h"
#include "NatNetClient.h"

#pragma warning( disable : 4996 )
#define FRAMENUM 5

void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs);
void _WriteFrame(FILE* fp, sFrameOfMocapData* data);
void _WriteFooter(FILE* fp);
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData);		// receives data from the server
void __cdecl MessageHandler(int msgType, char* msg);		            // receives NatNet error mesages
void resetClient();
int CreateClient(int iConnectionType);
bool inverseKinematics(float x, float y);
void sendAngles(char *angles, Serial *SP);

unsigned int MyServersDataPort = 3130;
unsigned int MyServersCommandPort = 3131;
//int iConnectionType = ConnectionType_Multicast;
int iConnectionType = ConnectionType_Unicast;

NatNetClient* theClient;
FILE* fp;

char szMyIPAddress[128] = "";
char szServerIPAddress[128] = "";

int analogSamplesPerMocapFrame = 0;
int sampleCount = 0;
bool objectInFrame = false;
bool doneCalc = false;
bool isDebug = true;
MarkerData markerDataArr[FRAMENUM];

// Rig constants 
const float l1 = 0.28;
const float l5 = 0.62;
const float l2 = 0.6;
const float l3 = l2;
const float l4 = l1;

const float pi = 3.142;

char motors[7] = "";

typedef enum{
	INITIALIZE,				//initializing connections
	WAITING,				//waiting for object to enter frame
	DATA_CAPTURE,			//receiving 5 frames
	CALCULATE_ENDPOINT,		//calculating landing position of object
	RESET					//wait for object to leave frame
} captureState_t;

captureState_t state = INITIALIZE;

int _tmain(int argc, _TCHAR* argv[])
{
	Serial* SP = new Serial("\\\\.\\COM3");    // adjust as needed

	if (SP->IsConnected())
		printf("We're connected");

	int iResult;     
    // parse command line args
    if(argc>1)
    {
        strcpy(szServerIPAddress, argv[1]);	// specified on command line
        printf("Connecting to server at %s...\n", szServerIPAddress);
    }
    else
    {
        strcpy(szServerIPAddress, "");		// not specified - assume server is local machine
        printf("Connecting to server at LocalMachine\n");
    }
    if(argc>2)
    {
        strcpy(szMyIPAddress, argv[2]);	    // specified on command line
        printf("Connecting from %s...\n", szMyIPAddress);
    }
    else
    {
        strcpy(szMyIPAddress, "");          // not specified - assume server is local machine
        printf("Connecting from LocalMachine...\n");
    }

    // Create NatNet Client
    iResult = CreateClient(iConnectionType);
    if(iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");
	void* response;
	int nBytes;
	iResult = theClient->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	// Retrieve Data Descriptions from server
	printf("\n\n[SampleClient] Requesting Data Descriptions...");
	sDataDescriptions* pDataDefs = NULL;
	int nBodies = theClient->GetDataDescriptions(&pDataDefs);
	if(!pDataDefs)
	{
		printf("[SampleClient] Unable to retrieve Data Descriptions.");
	}
	else
	{
        printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("MarkerSet Name : %s\n", pMS->szName);
                for(int i=0; i < pMS->nMarkers; i++)
                    printf("%s\n", pMS->szMarkerNames[i]);

            }
            else
            {
                printf("Unknown data type.");
                // Unknown
            }
        }      
	}

	
	// Create data file for writing received stream into
	char szFile[MAX_PATH];
	char szFolder[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, szFolder);
	if(argc > 3)
		sprintf(szFile, "%s\\%s", szFolder, argv[3]);
	else
		sprintf(szFile, "%s\\Client-output.pts",szFolder);
	fp = fopen(szFile, "w");
	if(!fp)
	{
		printf("error opening output file %s.  Exiting.", szFile);
		exit(1);
	}
	if(pDataDefs)
		_WriteHeader(fp, pDataDefs);

	// Ready to receive marker stream!
	printf("\nClient is connected to server and listening for data...\n");
	bool bExit = false;
	while (1) {
	//*************************************
	// state transition - run region      *
	//*************************************
		if (state == INITIALIZE) {
			state = WAITING;
		}
		else if (state == WAITING) {
			if (objectInFrame) {
				if (isDebug) {
					printf("STATE: Object has entered frame\n");
				}
				sampleCount = 0;
				doneCalc = false;
				state = DATA_CAPTURE;
			}
		}
		else if (state == DATA_CAPTURE) {
			if (sampleCount >= FRAMENUM) {
				if (isDebug) {
					printf("STATE: Sample complete\n");
				}
				state = CALCULATE_ENDPOINT;
				if (isDebug) {
					printf("Reading from array\n");
					for (int i = 0; i < sampleCount; i++) {
						printf("Other Marker %d : %3.5f   %3.5f   %3.5f\n",
							i,
							markerDataArr[i][0],
							markerDataArr[i][1],
							markerDataArr[i][2]);
					}
				}
			}
		}
		else if (state == CALCULATE_ENDPOINT) {
			if (!objectInFrame) {
				if (isDebug) {
					printf("STATE: Object has left frame\n");
				}
				state = WAITING;
			}
		}
	//*****************
	//* state actions *
	//*****************
		double y_d, v_x, v_y, v_z, t_1, t_2, x_f, z_f;
		const double g = 9.81;
		switch (state) {
		case INITIALIZE:
			sampleCount = 0;
			objectInFrame = false;
			break;
		case WAITING:
			break;
		case DATA_CAPTURE:
			break;
		case CALCULATE_ENDPOINT:
			if (!doneCalc) {
				y_d = 0.076-markerDataArr[0][1];
				v_x = (markerDataArr[FRAMENUM - 1][0] - markerDataArr[0][0]) / ((FRAMENUM - 1)*0.01);
				v_y = (markerDataArr[FRAMENUM - 1][1] - markerDataArr[0][1]) / ((FRAMENUM - 1)*0.01);
				v_z = (markerDataArr[FRAMENUM - 1][2] - markerDataArr[0][2]) / ((FRAMENUM - 1)*0.01);
				t_1 = (v_y + sqrt((v_y*v_y)-2*g*y_d))/g;
				t_2 = (v_y - sqrt((v_y*v_y) - 2 * g*y_d)) / g;
				x_f = v_x * t_1 + markerDataArr[0][0];
				z_f = v_z * t_1 + markerDataArr[0][2];
				if (isDebug) {
					printf("t_1 = %.5f\n t_2 = %.5f\n x_f = %.5f\n z_f = %.5f\n", t_1, t_2, x_f, z_f);
				}
				//Calculate IK
				if (inverseKinematics(x_f, -z_f)){
					//Send to arduino
					sendAngles(motors, SP);
					sendAngles("095095", SP);
				}
			}
			doneCalc = true;
			break;
		default:
			break;
		}
	}

	// Done - clean up.
	theClient->Uninitialize();
	_WriteFooter(fp);
	fclose(fp);

	return ErrorCode_OK;
}

// Establish a NatNet Client connection
int CreateClient(int iConnectionType)
{
    // release previous server
    if(theClient)
    {
        theClient->Uninitialize();
        delete theClient;
    }

    // create NatNet client
    theClient = new NatNetClient(iConnectionType);



    // set the callback handlers
    theClient->SetVerbosityLevel(Verbosity_Warning);
    theClient->SetMessageCallback(MessageHandler);
    theClient->SetDataCallback( DataHandler, theClient );	// this function will receive data from the server
    // [optional] use old multicast group
    //theClient->SetMulticastAddress("224.0.0.1");

    // print version info
    unsigned char ver[4];
    theClient->NatNetVersion(ver);
    printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

    // Init Client and connect to NatNet server
    // to use NatNet default port assignments
    int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress);
    // to use a different port for commands and/or data:
    //int retCode = theClient->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // get # of analog samples per mocap frame of data
        void* pResult;
        int ret = 0;
        int nBytes = 0;
        ret = theClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            analogSamplesPerMocapFrame = *((int*)pResult);
            printf("Analog Samples Per Mocap Frame : %d", analogSamplesPerMocapFrame);
        }

        // print server info
        sServerDescription ServerDescription;
        memset(&ServerDescription, 0, sizeof(ServerDescription));
        theClient->GetServerDescription(&ServerDescription);
        if(!ServerDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("[SampleClient] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
            ServerDescription.HostAppVersion[1],ServerDescription.HostAppVersion[2],ServerDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
            ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", szMyIPAddress);
        printf("Server IP:%s\n", szServerIPAddress);
        printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
    }

    return ErrorCode_OK;

}

// DataHandler receives data from the server
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	NatNetClient* pClient = (NatNetClient*) pUserData;

	if(fp)
		_WriteFrame(fp,data);
	
    int i=0;
	
	//if number of markers > 0, object has entered frame
	objectInFrame = (data->nOtherMarkers > 0) ? true : false;

	//only increment sample count when object is in frame to prevent overflow
	if (state == DATA_CAPTURE && sampleCount < FRAMENUM) {
		// Other Markers
		printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
		for (i = 0; i < data->nOtherMarkers; i++)
		{
			printf("Other Marker %d : %3.5f   %3.5f   %3.5f\n",
			i,
			data->OtherMarkers[i][0],
			data->OtherMarkers[i][1],
			data->OtherMarkers[i][2]);
		}
		memcpy(&markerDataArr[sampleCount], data->OtherMarkers[0], sizeof(data->OtherMarkers[0]));
		sampleCount++;
	}

    /*printf("FrameID : %d\n", data->iFrame);
    printf("Timestamp :  %3.2lf\n", data->fTimestamp);
    printf("Latency :  %3.2lf\n", data->fLatency);*/
    
    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording)
        printf("RECORDING\n");
    if(bTrackedModelsChanged)
        printf("Models Changed.\n");
	
        
    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;
	bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
	// decode to friendly string
	char szTimecode[128] = "";
	pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);
	//printf("Timecode : %s\n", szTimecode);
}

// MessageHandler receives NatNet error/debug messages
void __cdecl MessageHandler(int msgType, char* msg)
{
	printf("\n%s\n", msg);
}

/* File writing routines */
void _WriteHeader(FILE* fp, sDataDescriptions* pBodyDefs)
{
	int i=0;

    if(!pBodyDefs->arrDataDescriptions[0].type == Descriptor_MarkerSet)
        return;
        
	sMarkerSetDescription* pMS = pBodyDefs->arrDataDescriptions[0].Data.MarkerSetDescription;

	fprintf(fp, "<MarkerSet>\n\n");
	fprintf(fp, "<Name>\n%s\n</Name>\n\n", pMS->szName);

	fprintf(fp, "<Markers>\n");
	for(i=0; i < pMS->nMarkers; i++)
	{
		fprintf(fp, "%s\n", pMS->szMarkerNames[i]);
	}
	fprintf(fp, "</Markers>\n\n");

	fprintf(fp, "<Data>\n");
	fprintf(fp, "Frame#\t");
	for(i=0; i < pMS->nMarkers; i++)
	{
		fprintf(fp, "M%dX\tM%dY\tM%dZ\t", i, i, i);
	}
	fprintf(fp,"\n");

}

void _WriteFrame(FILE* fp, sFrameOfMocapData* data)
{
	fprintf(fp, "%d", data->iFrame);
	for(int i =0; i < data->MocapData->nMarkers; i++)
	{
		fprintf(fp, "\t%.5f\t%.5f\t%.5f", data->MocapData->Markers[i][0], data->MocapData->Markers[i][1], data->MocapData->Markers[i][2]);
	}
	fprintf(fp, "\n");
}

void _WriteFooter(FILE* fp)
{
	fprintf(fp, "</Data>\n\n");
	fprintf(fp, "</MarkerSet>\n");
}

void resetClient()
{
	int iSuccess;

	printf("\n\nre-setting Client\n\n.");

	iSuccess = theClient->Uninitialize();
	if(iSuccess != 0)
		printf("error un-initting Client\n");

	iSuccess = theClient->Initialize(szMyIPAddress, szServerIPAddress);
	if(iSuccess != 0)
		printf("error re-initting Client\n");


}

bool inverseKinematics(float x, float y)
{
	float E1, F1, G1, E4, F4, G4, t11, t12, t41, t42, angle1_1, angle1_2, angle4_1, angle4_2;
	int motor1, motor2;
	std::string motor1string, motor2string, motorString;
	char buffer2[4];

	E1 = -2 * l1*x;
	F1 = -2 * l1*y;
	G1 = pow(l1, 2) - pow(l2, 2) + pow(x, 2) + pow(y, 2);
	E4 = 2 * l4*(-x + l5);
	F4 = 2 * l4*(-y);
	G4 = pow(l5, 2) + pow(l4, 2) - pow(l3, 2) + pow(x, 2) + pow(y, 2) - 2 * l5*x;
	t11 = -F1 + sqrt(pow(E1, 2) + pow(F1, 2) - pow(G1, 2));
	t11 = t11 / (G1 - E1);
	t12 = -F1 - sqrt(pow(E1, 2) + pow(F1, 2) - pow(G1, 2));
	t12 = t12 / (G1 - E1);
	t41 = -F4 + sqrt(pow(E4, 2) + pow(F4, 2) - pow(G4, 2));
	t41 = t41 / (G4 - E4);
	t42 = -F4 - sqrt(pow(E4, 2) + pow(F4, 2) - pow(G4, 2));
	t42 = t42 / (G4 - E4);

	angle1_1 = 2 * atan2(t11, 1);
	angle1_1 = 180 * angle1_1 / pi;

	angle1_2 = 2 * atan2(t12, 1);
	angle1_2 = 180 * angle1_2 / pi;

	angle4_1 = 2 * atan2(t41, 1);
	angle4_1 = 180 * angle4_1 / pi;

	angle4_2 = 2 * atan2(t42, 1);
	angle4_2 = 180 * angle4_2 / pi;


	if (angle1_1 >= 30 && angle1_1 <= 160)
		motor1 = int(angle1_1);
	else if (angle1_2 >= 30 && angle1_2 <= 160)
		motor1 = int(angle1_2);
	else
		motor1 = 0;

	if (angle4_1 >= 30 && angle4_1 <= 160)
		motor2 = int(angle4_1);
	else if (angle4_2 >= 30 && angle4_2 <= 160)
		motor2 = int(angle4_2);
	else
		motor2 = 0;
	
	motor1string = std::to_string(motor1);
	motor2string = std::to_string(motor2);

	int temp_length = motor1string.length();

	for (int i = 0; i < 3 - temp_length; i++) {
		motor1string = "0" + motor1string;
	}

	temp_length = motor2string.length();

	for (int i = 0; i < 3 - temp_length; i++) {
		motor2string = "0" + motor2string;
	}

	motorString = motor1string + motor2string;
	strcpy(motors, motorString.c_str());
	
	/*itoa(motor1, motors, 10);
	itoa(motor2, buffer2, 10);
	strncat(motors, buffer2, 4);*/
	if (isDebug) {
		printf("angle for Servo1 = %d \n angle for Servo 2 = %d \n", motor1, motor2);
		printf("Angles: %f.2 %f.2 %f.2 %f.2\n", angle1_1, angle1_2, angle4_1, angle4_2);
	}
	
	if (motor1 == 0 || motor2 == 0) 
		return false;
	return true;
}

void sendAngles(char *angles, Serial *SP)
{
	//char incomingData[7] = "";   // don't forget to pre-allocate memory
	int dataLength = 7;
	//int readResult = 0;
	int writeResult = 0;

	writeResult = SP->WriteData(angles, dataLength);
	if (true) {
		printf("String angles: %s\n", angles);
	}

	//writeResult = SP->WriteInt(outgoingData, dataLength);
	//printf("Bytes written: %i\n", writeResult);
	Sleep(2000);
	//readResult = SP->ReadData(incomingData, dataLength);
	//printf("Bytes read: (-1 means no data available) %i\n", readResult);

	//std::string test(incomingData);

	//printf("%s\n", incomingData);
	//test = "";

}