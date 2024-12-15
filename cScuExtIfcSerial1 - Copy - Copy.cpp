#include "cScuExtIfcSerial.h"

// Declares the directory structure
struct dirent **pNamelist = NULL;


// 2294 as of now written here, pls check and revert it
char *mcDevice = "/dev/ttyS0";

	
// Pointer to Data frame, assigns dynamically
char *cDataFrame = NULL;

char *cDataBuffer = NULL;

char *cReadBuffer;    		//2294 allocated 17 bytes but check msg 10 seg is taking only 1 byte

int32 nHL7Msg10SegLenth = 0;

char cSentDataMsg10Seg[50] = "";

char cSerFileName[100] = "";

// Constructor
cScuExtIfcSerial :: cScuExtIfcSerial(stSerialPortConfig *tstSerialPortParam)
{

	tstSerialPortParams =  tstSerialPortParam;
	/*// Pointer to message queue of External Interface

	mpExtIfcSerialMsgQ = pMsgQ;


	// Initializes the serial port settings

	mtenumBaudRate = tstSerialPortParam.mtenumBaudRate;
	mtenumStpBit = tstSerialPortParam.mtenumStpBit;
	mtenumParity = tstSerialPortParam.mtenumParity;
	mStartByte = tstSerialPortParam.mStartByte;
	mEndByte = tstSerialPortParam.mEndByte;
	mtenumAckNack = tstSerialPortParam.mtenumAckNack;
	mnAckTimeout = tstSerialPortParam.mnAckTimeout;
	mnBlockDelay = tstSerialPortParam.mnBlockDelay;*/

	mnFileDesp = INITIAL_VAL;			// 2294 Check global variables can initialize in the  constructor
	nFrameLength = INITIAL_VAL;

}

// mCreateThread
void cScuExtIfcSerial::mCreateSerialCommThread()
{

	if (pthread_create(&mSerialThreadID, NULL, &mStartSerialThread, NULL) != SCU_SUCCESS)
	{
		cScuEvLogIfc::mLogEventErr(PID_EXTIFC, time(0), EV_EXTIFC_SER_THREAD_CREATE_FAIL, (int32)errno);      // check murali thread create error number
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial comm thread fail :%d", (int32)errno));

		return;
	}
	else
	{
		cScuEvLogIfc::mLogEventErr(PID_EXTIFC, time(0), EV_EXTIFC_SERIAL_THREAD_CREATED); 
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial comm thread creation success"));
	}

	if(pthread_detach(mSerialThreadID) != SCU_SUCCESS)
	{
		cScuEvLogIfc::mLogEventErr(PID_EXTIFC, time(0), EV_EXTIFC_SER_THREAD_DETACH_FAIL, (int32)errno);
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial comm thread detach fail :%d", (int32)errno));
	
	}
	else
	{
		cScuEvLogIfc::mLogEventErr(PID_EXTIFC, time(0), EV_EXTIFC_SERIAL_THREAD_DETACHED);
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial comm thread detach success"));
	}

	return;	
}

void* cScuExtIfcSerial::mStartSerialThread(void *arg)
{
	int32 nRetval = INITIAL_VAL;
	int32 nFileCount = INITIAL_VAL;
	int32 nIndex = INITIAL_VAL;

	
	nRetval = mInitializeSerialPort();

	if(nRetval == SCU_FAIL)
	{
		cScuEvLogIfc::mLogEventErr(PID_EXTIFC, time(0), EV_EXTIFC_SERIALPORT_INTIALIZATION_FAIL);

		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial port Initialization Fail"));

		return;   // 2294 check it return should be there or not
	}
	else
	{
		cScuEvLogIfc::mLogEventErr(PID_EXTIFC, time(0), EV_EXTIFC_SERIALPORT_INTIALIZATION_SUCCESS);

		while(1)    // 2294 check for stopping the loop on entering into error mode
		{

			nFileCount = CheckFileAvailability();

			if(nFileCount > 2)
			{
				for(nIndex = INITIAL_VAL; nIndex<nFileCount; nIndex++)
				{

					if((strcmp(pNamelist[nIndex]->d_name, ".") == INITIAL_VAL) || (strcmp(pNamelist[nIndex]->d_name, "..") == INITIAL_VAL))
					{
						// does nothing
					}
					else
					{  										//  2294 else started here
					
						nRetval = mFrameMessage(nIndex);

						if(nRetval != SCU_FAIL)
						{

							mSendandReceiveSerData();		// 
								
						}									// If fails, no action being taken, just exits, check that
					}
				}
			}
			else
			{

				sleep(10);
			}

			for (nIndex=0; nIndex<nFileCount; nIndex++)
			{
				free(pNamelist[nIndex]);
			}

		}

		close(mnFileDesp);


	} // else ended here
}


int32 cScuExtIfcSerial::mInitializeSerialPort()
{

	struct termios stSerialTerm;
	uint16 nIndex = INITIAL_VAL;
	int32 nRetval = SCU_FAIL;

	// O_SYNC tells the driver not to return until the transfer is complete
	mnFileDesp = open(mcDevice, O_RDWR | O_SYNC );

	if(mnFileDesp == ERROR_VAL)
	{
		cScuEvLogIfc::mLogEventErr(PID_EXTIFC, time(0), EV_EXTIFC_SERIAL_DEVICE_OPEN_FAIL);
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial Device open Fail"));
	}	
	else
	{

		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial Device open Success"));
		
		if(ERROR_VAL == tcgetattr(mnFileDesp, &stSerialTerm))
		{
			close(mnFileDesp);
			EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Failed to retrieve the Serial port attributes"));	
		}
		else
		{

			// Assigned control Flag settings
			stSerialTerm.c_cflag = (tstSerialPortParams->mtenumBaudRate | CS8 | tstSerialPortParams->mtenumStpBit | INITIAL_VAL | INITIAL_VAL | INITIAL_VAL | CLOCAL | CREAD | HUPCL);

			if(tstSerialPortParams->mtenumParity != NONE)
			{

				stSerialTerm.c_cflag |= PARENB;

				if(tstSerialPortParams->mtenumParity == ODD)
				{
					stSerialTerm.c_cflag |= PARODD;
				}
				
			}
				
			// Clear the output flag
			
			stSerialTerm.c_oflag = INITIAL_VAL;
			stSerialTerm.c_lflag = INITIAL_VAL;     // ICANON

			// Clear the Input Flag
			stSerialTerm.c_iflag = INITIAL_VAL;

			// Sets the Output Baud rate
			cfsetospeed(&stSerialTerm,tstSerialPortParams->mtenumBaudRate);

			for (nIndex = INITIAL_VAL; nIndex < NCCS; nIndex++)	// the control codes
			{
				stSerialTerm.c_cc[nIndex] = CONTROL_CODE; 				  
			}

			stSerialTerm.c_cc[VMIN] = ONE_CHAR;
			stSerialTerm.c_cc[VTIME] = INITIAL_VAL;

			// Set the new Attributes to the serial port device
			if (SCU_FAIL == tcsetattr(mnFileDesp, TCSANOW, &stSerialTerm))
			{

				close(mnFileDesp);

				EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Failed to set the Serial port attributes"));

			}
			else
			{
				if (SCU_FAIL == tcflush(mnFileDesp, TCIOFLUSH))
				{
					close(mnFileDesp);

					EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Failed to flush the Input/Output of serial port"));

				}
				else
				{
					EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial port Initialized successfully"));

					printf("\n *********************** Serial port Initialized successfully");
					fflush(stdout);
					
					nRetval = SCU_SUCCESS;
				}
			}
		}
		
	}
	return nRetval;
}




// Check file availability function
int32 cScuExtIfcSerial::CheckFileAvailability()
{
	int32 nRetval;

	nRetval = scandir(Dirname, &pNamelist, NULL, alphasort);

					printf("\n *********************** scandir count %d", nRetval);
					fflush(stdout);
	return nRetval;

}

// Framing the message
int32 cScuExtIfcSerial::mFrameMessage(int32 Index)
{

	stInitialFrame tstDataInitFrame = {'D', 23, 0x0D};
	stEndFrame tstDataEndFrame = {{0,0}, 0xFFFF, 0x1C, 0x0D};
	int32 nRetval = 0;
	struct stat stSerFileStatus;
	char *cTempDataBufPtr = NULL;				//2294 Verify it 

	memset(cSerFileName, "0", sizeof(cSerFileName));

	// memset(&tstTotalDataBytes, 0, sizeof(tstTotalDataBytes));
	sprintf(cSerFileName, "%s%s", cDirname, pNamelist[Index]->d_name);


	if(stat(cSerFileName, &stSerFileStatus) == INITIAL_VAL)
	{

		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial File Present"));

		nRetval = mReadDataBuffer(stSerFileStatus.st_size);

		if(nRetval != SCU_SUCCESS)
		{
			return SCU_FAIL;
		}
		else
		{

			nRetval = mCalCheckSumFn(tstDataEndFrame.nCheckSumValue, stSerFileStatus.st_size);

			mFindMessageSeg();
		}
	}
	else
	{
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Serial File not present in the serial folder"));
		return SCU_FAIL;
	}


	printf("\n Size of the File %d", stSerFileStatus.st_size);
	fflush(stdout);

	nFrameLength = (stSerFileStatus.st_size + 14 + 1);          // strong it the operation
	
	cDataFrame = (char *)malloc(nFrameLength);

	if(cDataFrame == NULL)
	{

		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Failed to allocate a memory for Serial Frame"));

		return SCU_FAIL;
	
	}

	memset(cDataFrame, 0, sizeof(nFrameLength));

	cTempDataBufPtr = cDataFrame;

	*cTempDataBufPtr = tstSerialPortParams->mStartByte;					// 2294 Strong it the operation & checksum calc

	cTempDataBufPtr++;								// Check arun for written the code in pointers correct(pointers is incremented)

	memcpy(cTempDataBufPtr, &tstDataInitFrame, sizeof(stInitialFrame));

	cTempDataBufPtr += 5;

	memcpy(cTempDataBufPtr, cDataBuffer, stSerFileStatus.st_size);

	tstDataEndFrame.tstTotalDataBytes.nDataSize = stSerFileStatus.st_size + 5;

	cTempDataBufPtr += stSerFileStatus.st_size;
	
	memcpy(cTempDataBufPtr, &tstDataEndFrame, sizeof(stEndFrame));

					printf("\n *********************** Framed messages **************");
					fflush(stdout);	
	return SCU_SUCCESS;	

}

int32 cScuExtIfcSerial::mReadDataBuffer(uint32 nFileSize)
{

	int32 nSerialFileDesp = INITIAL_VAL;
	int32 nRetval = INITIAL_VAL;
	
	nSerialFileDesp = open(cSerFileName, O_RDONLY);

	if(nSerialFileDesp == ERROR_VAL)
	{
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Faile to open a Serial File"));

		return SCU_FAIL;
	}	

	cDataBuffer = (char *)malloc(nFileSize);

	if(cDataBuffer == NULL)
	{

		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Failed to allocate a memory for Data Buffer"));

		return SCU_FAIL;
	}

	memset(cDataBuffer, 0, sizeof(nFileSize));

	nRetval = read(nSerialFileDesp, cDataBuffer, nFileSize);   //2294 strong if by reading multiple times

	if(nRetval != nFileSize)
	{
			
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Failed to Read from Bytes"));

		return SCU_FAIL;		
	}

	nRetval = close(nSerialFileDesp);

	if(nRetval != SCU_SUCCESS)
	{
		EXTIFC_DBG_MSG(EXTIFC_PR_TXT,("Failed to close a file"));

		return SCU_FAIL;		
	}

					printf("\n *********************** Read Data**************");
					fflush(stdout);	
	return nRetval;
}

bool cScuExtIfcSerial::mCalCheckSumFn(uint16 &nChecksumValue, uint32 uFilesize) //2294 uFilesize should always be int8 or any value
{

						//2294 strong checksum as per the jaya decision
	char *TempDataBufPtr = NULL;

	TempDataBufPtr = cDataBuffer;
	
	while(uFilesize>0)
	{
		
		nChecksumValue = (nChecksumValue >> 8) ^ crctab16[(nChecksumValue ^ *TempDataBufPtr) & 0xff];	
		uFilesize--;	 
		TempDataBufPtr++;   
	}

	nChecksumValue = ~nChecksumValue;

					printf("\n *********************** checksum %X", nChecksumValue);
					fflush(stdout);

}

int32 cScuExtIfcSerial::mFlushBuffers()
{
	int32 Retval;

	Retval = tcflush(mcDevice, TCIOFLUSH);

	if(Retval != SCU_FAIL)
	{
		return SCU_SUCCESS;
	}
	else
	{
		return SCU_FAIL;
	}
}

int32 cScuExtIfcSerial::mWriteSerialData()
{
	int32 Retval = SCU_FAIL;

	Retval = write(mcDevice, cDataFrame, nFrameLength-1);

					printf("\n *********************** write data %ld out of %ld", Retval, nFrameLength-1);
					fflush(stdout);	

	if(Retval == nFrameLength-1)
	{
		return Retval;
	}
	else
	{
		return SCU_FAIL;
	}
}


int32 cScuExtIfcSerial::mProcessReadData()
{
	int32 nRetval;

	nRetval = mReadSerialData();               // 2294 if returns value fails, what we r doing, check it very important

	if(nRetval == SCU_SUCCESS)
	{
			
		nRetval = cValidateReadData();
	}
					printf("\n *********************** mProcessReadData status %d", nRetval);
					fflush(stdout);
	return nRetval;
}


int32 cScuExtIfcSerial::mReadSerialData()
{
	int32 cReadBufferSize = 0;
	struct timeval mstTimeout;
	fd_set mFdSet;
	
	FD_ZERO(&mFdSet);
	FD_SET(mcDevice, &mFdSet);

	cReadBufferSize = 16 + nHL7Msg10SegLenth;

	cReadBuffer = (char *)malloc(cReadBufferSize);
	
	memset(cReadBuffer, 0, sizeof(cReadBufferSize));
	
	
	printf("\n Before Block");
	fflush(stdout);

	do
	{	
		mstTimeout.tv_sec = 5;              // 2294 as of now given 5 seconds
		mstTimeout.tv_usec = 0;

		Retval = select((mcDevice + 1), &mFdSet, NULL, NULL, &mstTimeout);

		if(Retval == -1)
		{
			printf("\n Select call failed");
			fflush(stdout);
			return SCU_FAIL;
		}
		else if(Retval == 0)
		{
			printf("\n Select call timeout happened");
			fflush(stdout);
			return 1;					
		}
		else
		{
			Retval = read(mcDevice, cReadBuffer, ONE_CHAR);
		}


	}while((Retval == 1) && (*(cReadBuffer+0) != cStartByte));              // If Read call fails, while(1) is always be running, check it


	if(Retval != -1)
	{
		mstTimeout.tv_sec = 5;              // 2294 as of now given 5 seconds
		mstTimeout.tv_usec = 0;	

		Retval = select((mcDevice + 1), &mFdSet, NULL, NULL, &mstTimeout);

		if(Retval == -1)
		{
			printf("\n Select call failed");
			fflush(stdout);
			return SCU_FAIL;
		}
		else if(Retval == 0)
		{
			printf("\n Select call timeout happened");
			fflush(stdout);
			return 1;					
		}
		else
		{
			Retval = read(mcDevice, (cReadBuffer+1), ONE_CHAR);
		}

		if(*(cReadBuffer+1) == 'A') && (Retval != -1))
		{

			mstTimeout.tv_sec = 5;              // 2294 as of now given 5 seconds
			mstTimeout.tv_usec = 0;	

			Retval = select((mcDevice + 1), &mFdSet, NULL, NULL, &mstTimeout);

			if(Retval == -1)
			{
				printf("\n Select call failed");
				fflush(stdout);
				return SCU_FAIL;
			}
			else if(Retval == 0)
			{
				printf("\n Select call timeout happened");
				fflush(stdout);
				return 1;					
			}
			else
			{
				Retval = read(mcDevice, (cReadBuffer+2), cReadBufferSize-3);
			}

			if(Retval == (cReadBufferSize-3))
			{
				printf("\n Received ACK");
				fflush(stdout);
				return SCU_SUCCESS;
			}
			else
			{
				return SCU_FAIL;
			}
				
		}
		else if(*(cReadBuffer+1) == 'N') && (Retval != -1))
		{
			
			mstTimeout.tv_sec = 5;              // 2294 as of now given 5 seconds
			mstTimeout.tv_usec = 0;	

			Retval = select((mcDevice + 1), &mFdSet, NULL, NULL, &mstTimeout);

			if(Retval == -1)
			{
				printf("\n Select call failed");
				fflush(stdout);
				return SCU_FAIL;
			}
			else if(Retval == 0)
			{
				printf("\n Select call timeout happened");
				fflush(stdout);
				return 1;					
			}
			else
			{
				Retval = read(mcDevice, (cReadBuffer+2), cReadBufferSize-2);
			}

			if(Retval == (cReadBufferSize-2))
			{
				printf("\n Received NACK");
				fflush(stdout);			
				return SCU_SUCCESS;
			}
			else
			{
				return SCU_FAIL;
			}
		}
		else
		{

			return SCU_FAIL;
		}
	}
	else
	{
		// Read Fail
		return SCU_FAIL;
	}
		

}

int32 cScuExtIfcSerial::cValidateReadData()
{
	stInitialFrame tstReadInitialData;
	stEndFrame tstReadEndFrame = {{0,0}, 0xFFFF, 0x1C, 0x0D};;
	stDataSegCode tstReadDataBytes;
	uint16 nCalReadChksumVal = 0xFFFF;

	memcpy(&tstReadInitialData, cReadBuffer+1, sizeof(stInitialFrame));

	if((tstReadInitialData.cBlockType == 'A') && (tstReadInitialData.nProtocolID == 23) && (tstReadInitialData.cCarriageReturn == 0x0D))
	{
		memcpy(&tstReadDataBytes.nHL7Msg10Seg, cReadBuffer+5, nHL7Msg10SegLenth);           // 2294 sizeof msg10 should come to exact correctly

		if(strcmp(tstReadDataBytes.nHL7Msg10Seg, cSentDataMsg10Seg) == 0)
		{
			memcpy(&tstReadEndFrame, cReadBuffer+5+nHL7Msg10SegLenth, sizeof(stEndFrame));

			
			mCalCheckSumFn(nCalReadChksumVal, (5+nHL7Msg10SegLenth));			// 2294 check length size sending to fn

			if((tstReadEndFrame.tstTotalDataBytes.nDataSize == (5+nHL7Msg10SegLenth)) && (tstReadEndFrame.nCheckSumValue == nCalReadChksumVal)\
				(tstReadEndFrame.cEndBlockChar == 0x1C) && (tstReadEndFrame.cCarriageReturn == 0x0D))
			{
				return 0;
				printf("\n ACK received successfully");
				fflush(stdout);
			}
			else
			{
				return -1;
				printf("\n ACK receive FAIL");
				fflush(stdout);
			}
		}
		else
		{
				return 3;											// 2294 if msg 10 is not equal wait for another message
				printf("\nWrong ACK receive");
				fflush(stdout);
		}
	}
	else if((tstReadInitialData.cBlockType == 'N') && (tstReadInitialData.nProtocolID == 23) && (tstReadInitialData.cCarriageReturn == 0x0D))
	{
		
		memcpy(&tstReadDataBytes, cReadBuffer+5, nHL7Msg10SegLenth+2);

		if(strcmp(tstReadDataBytes.nHL7Msg10Seg, cSentDataMsg10Seg) == 0)
		{
			memcpy(&tstReadEndFrame, (cReadBuffer+5+nHL7Msg10SegLenth+2), sizeof(stEndFrame));

			
			mCalCheckSumFn(nCalReadChksumVal, (7+nHL7Msg10SegLenth));			// 2294 check length size sending to fn

			if((tstReadEndFrame.tstTotalDataBytes.nDataSize == (7+nHL7Msg10SegLenth)) && (tstReadEndFrame.nCheckSumValue == nCalReadChksumVal)\
				(tstReadEndFrame.cEndBlockChar == 0x1C) && (tstReadEndFrame.cCarriageReturn == 0x0D))
			{
				return 2;
				printf("\n NACK received successfully");
				fflush(stdout);
			}
			else
			{
				return -1;
				printf("\n NACK receive FAIL");
				fflush(stdout);
			}			
		
		}
		else
		{
			return 3;											// 2294 if msg 10 is not equal wait for another message
			printf("\nWrong NACK receive");
			fflush(stdout);
		}
	}
	else
	{
		return -1;
		printf("\nWrong ACK/NACK received");
		fflush(stdout)

	}
}

void cScuExtIfcSerial::mFindMessageSeg()
{
	char *cMsgHeaderPointer = NULL;
	bool bFound = true;
	char cCount = 0;
	char cIndex = 0;
	
	cMsgHeaderPointer = strstr(cDataBuffer,"MSH");

	while(bFound)
	{
		
		if(*cMsgHeaderPointer == '|')
		{
			
			cCount++;
			if(cCount == 9)
			{
				bFound = false;
			}

		}

		cMsgHeaderPointer++;
	}

	while(*cMsgHeaderPointer != '|')
	{
		cSentDataMsg10Seg[cIndex] = *cMsgHeaderPointer;                      // double check with the arun for correct confirmation
		cIndex++;
		cMsgHeaderPointer++;
	}

	nHL7Msg10SegLenth = strlen(cSentDataMsg10Seg);
				printf("\n Found MSH.10 segment");
				fflush(stdout);	
		
}


int32 cScuExtIfcSerial::mSendandReceiveSerData()
{

	bool nResendMessage = true;
	static int32 nCount = 0;
	int32 nRetval = 0;

	while(nResendMessage)				// check for first time also, resent message is set to true
	{

		nRetval = mFlushBuffers();

		if(nRetval != SCU_FAIL)
		{
			nRetval = mWriteSerialData();
			printf("\n RetVal of mWriteSerialData %d", nRetval);
			fflush(stdout);			
		}
		else
		{
			printf("\n Flush Buffer failure");
			fflush(stdout);
			return nRetval;
			// 2294 check here what it will happen if flush buffer is failed
		}


		if(nRetval != SCU_FAIL)
		{
			if(tstSerialPortParams.mtenumAckNack == YES)
			{
				do
				{

					nRetval = mProcessReadData();

					printf("\n RetVal of mProcessReadData %d", nRetval);
					fflush(stdout);
					
				}while(nRetval == 3);

				if((nRetval == 2) && ((NackField == 'C') || (NackField == 'X')))
				{
					nResendMessage = true;
				}
				else if(nRetval == 1)
				{
					nResendMessage = true;
				}
				else
				{	// this can be both system call failure or read success
					// NACK recived with data byte 'G' and 'B'
					nResendMessage = false;						// 2294 Check here system fails, wat have to take reactions
				printf("\n Valid data recived");
				fflush(stdout);					
				}
			}
			else
			{
				printf("\n Valid data recived");
				fflush(stdout);
				// wait for 100 ms timout
				nResendMessage = false;
			}

		}
		else if((nRetval == SCU_FAIL) && (tstSerialPortParams.mtenumAckNack == NO))
		{

			nResendMessage = true;												// Check when system command failures, just resending the msg, not taking any action

		}
		else
		{
			// Failure of write serial data
			nResendMessage = false;								// 2294 Check here system fails, wat have to take reactions
		}


		if(nResendMessage == true)
		{
			
			if(nCount < 3)
			{
				nCount++;
			}
			else
			{
				mDelFile();
				nCount = 0;
				break;
			}
		}
		else
		{
			// Here deleting for both real success and any system failures also
			mDelFile();
			nCount = 0;
		}
	}
}

void cScuExtIfcSerial::mDelFile()
{

				printf("\n Deleted File");
				fflush(stdout);
	unlink(cSerFileName);

}