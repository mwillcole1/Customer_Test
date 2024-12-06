/*For more information see notes.txt in the Documentation folder */
#include "usrcode.h"

#define _EnumMode_			// for PPScriptMode, replace this with #define _PPScriptMode_

#include "../Include/pp_proj.h"

extern struct SHM        *pshm;  // Pointer to shared memory
extern volatile unsigned *piom;  // Pointer to I/O memory
extern void              *pushm; // Pointer to user memory

void user_phase(struct MotorData *Mptr)
{
	static int count;
	static int SumCount;
	
	// DECLARATIONS TO REFERENCE I/O FROM CK3W-MD7110 AT ADDRESS 0
	int Input0,Input1,Input2,Input3,Input4,Input5,Input6,Input7;	// Digital Input declarations
	int Input8,Input9,Input10,Input11,Input12,Input13,Input14,Input15;
	volatile GateArray3 *MD7110_Idx0_IO;         // Gate Pointer declaration
	MD7110_Idx0_IO = GetGate3MemPtr(0);

	if (pshm->P[TestStart] == 1)
	{
		Input0 = (MD7110_Idx0_IO->GpioData[0] & 0x0001);
		Input1 = (MD7110_Idx0_IO->GpioData[0] & 0x0002) >> 1;
		Input2 = (MD7110_Idx0_IO->GpioData[0] & 0x0004) >> 2;
		Input3 = (MD7110_Idx0_IO->GpioData[0] & 0x0008) >> 3;
		Input4 = (MD7110_Idx0_IO->GpioData[0] & 0x0010) >> 4;
		Input5 = (MD7110_Idx0_IO->GpioData[0] & 0x0020) >> 5;
		Input6 = (MD7110_Idx0_IO->GpioData[0] & 0x0040) >> 6;
		Input7 = (MD7110_Idx0_IO->GpioData[0] & 0x0080) >> 7;
		Input8 = (MD7110_Idx0_IO->GpioData[0] & 0x0100) >> 8;
		Input9 = (MD7110_Idx0_IO->GpioData[0] & 0x0200) >> 9;
		Input10 = (MD7110_Idx0_IO->GpioData[0] & 0x0400) >> 10;
		Input11 = (MD7110_Idx0_IO->GpioData[0] & 0x0800) >> 11;
		Input12 = (MD7110_Idx0_IO->GpioData[0] & 0x1000) >> 12;
		Input13 = (MD7110_Idx0_IO->GpioData[0] & 0x2000) >> 13;
		Input14 = (MD7110_Idx0_IO->GpioData[0] & 0x4000) >> 14;
		Input15 = (MD7110_Idx0_IO->GpioData[0] & 0x8000) >> 15;
	}

	if (pshm->P[TestStart] == 0 && count > 0)
	{
		count = 0;
	}
	if (pshm->P[TestCalculation] == 0 && SumCount > 0)
	{
		SumCount = 0;
	}

	pshm->P[1000]++;	// DEBUG COUNTER - WATCH P1000 VARIABLE TO MAKE SURE CODE IS RUNNING
	
	if (pshm->P[TestStart] == 1)		// START MONITORING INPUT
	{
		if (Input0 == 1 && pshm->P[DropCounted] == 0)
		{
			pshm->P[TimeStart] = GetCPUClock();	// RECORD START TIME
			
			pshm->P[DropletCount]++;	// INCREMENT COUNT
			pshm->P[DropCounted] = 1;
		}

		if (pshm->P[DropCounted] == 1 && Input0 == 0)
		{
			pshm->P[TimeEnd] = GetCPUClock();	// RECORD END TIME
			pshm->P[TimeDuration + count] = pshm->P[TimeEnd] - pshm->P[TimeStart];	// LOAD TIME DURATION INTO ARRAY ELEMENT

			count++;
			pshm->P[ExpectedCount] = count;	
			pshm->P[DropCounted] = 0;	
		}
	}

	if (pshm->P[TestCalculation] == 1)	// RUN CALCULATIONS
	{
		if (SumCount < pshm->P[DropletCount])
		{
			pshm->P[TimeSum] = pshm->P[TimeDuration + SumCount] + pshm->P[TimeSum];
			SumCount++;
		}
	
		if (SumCount == pshm->P[DropletCount])
		{
			pshm->P[TimeAverage] = pshm->P[TimeSum] / pshm->P[DropletCount];
			pshm->P[TestCalculation] = 0;
		}
	}
}


double user_pid_ctrl(struct MotorData *Mptr)
{
	double *p;
	p = pushm;
	return 0;
}

void CaptCompISR(void)
{
	unsigned *pUnsigned = pushm;
	*pUnsigned = *pUnsigned + 1;
}

double GetLocal(struct LocalData *Ldata, int m)
{
	return *(Ldata->L + Ldata->Lindex + m);
}

void SetLocal(struct LocalData *Ldata, int m, double value)
{
	*(Ldata->L + Ldata->Lindex + m) = value;
}

double *GetLocalPtr(struct LocalData *Ldata, int m)
{
	return (Ldata->L + Ldata->Lindex + m);
}

double CfromScript(double cfrom_type, double arg2, double arg3, double arg4, double arg5, double arg6, double arg7, struct LocalData *Ldata)
{
	int icfrom_type = (int)cfrom_type;
	double *C, *D, *L, *R, rtn; // C, D, R - only needed if doing Kinematics

	C = GetCVarPtr(Ldata);  // Only needed if doing Kinematics
	D = GetDVarPtr(Ldata);  // Only needed if doing Kinematics
	L = GetLVarPtr(Ldata);  // Only needed if using Ldata or Kinematics
	R = GetRVarPtr(Ldata);  // Only needed if doing Kinematics
	rtn = -1.0;
	return rtn;
}

