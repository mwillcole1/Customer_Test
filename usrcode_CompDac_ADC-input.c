/*For more information see notes.txt in the Documentation folder */
#include "usrcode.h"
#define _PPScriptMode_		// for enum mode, replace this with #define _EnumMode_	

#include "../Include/pp_proj.h"

extern struct SHM        *pshm;  // Pointer to shared memory
extern volatile unsigned *piom;  // Pointer to I/O memory
extern void              *pushm; // Pointer to user memory

void user_phase(struct MotorData *Mptr)
{
	// DEBUG COUNTER, EXECUTING?
	pshm->P[1000]++;

	// STRUCTURE TO ACCESS GATE3[1] ELEMENTS
	volatile GateArray3 *Gate31;
	Gate31 = GetGate3MemPtr(1);

	
	// ACCESS GATE3 ADC STRUCTURE ELEMENT FOR ANALOG INPUTS ON CK3W-AD3100
	double C_ADC0 = Gate31->Chan[0].AdcAmp[0];
	double C_ADC1 = Gate31->Chan[0].AdcAmp[1];
	double C_ADC2 = Gate31->Chan[0].AdcAmp[2];
	double C_ADC3 = Gate31->Chan[0].AdcAmp[3];
	double C_ADC4 = Gate31->Chan[1].AdcAmp[0];
	double C_ADC5 = Gate31->Chan[1].AdcAmp[1];
	double C_ADC6 = Gate31->Chan[1].AdcAmp[2];
	double C_ADC7 = Gate31->Chan[1].AdcAmp[3];

	// TEST, READING ADCs?	MUST DEFINE ALL 'ADCx_input' IN GLOBAL VARIABLES
	pshm->P[1001] = C_ADC0;
	ADC0_input = C_ADC0;
	ADC1_input = C_ADC1;
	ADC2_input = C_ADC2;
	ADC3_input = C_ADC3;
	ADC4_input = C_ADC4;
	ADC5_input = C_ADC5;
	ADC6_input = C_ADC6;
	ADC7_input = C_ADC7;	

	int MtrNo = 1;
	
	// CLOSED LOOP?
	if(pshm->Motor[MtrNo].ClosedLoop == 1)
	{
		// WAIT FOR MOTION
		if(pshm->Motor[MtrNo].DesVelZero == 0)
		{
			pshm->Motor[MtrNo].CompDac = Analog_ScaleFactor * ADC0_input;	// MUST DEFINE 'Analog_ScaleFactor' IN GLOBAL VARIABLES
		}
		
		// STOPPED?
		if(pshm->Motor[MtrNo].DesVelZero == 1)
		{
			pshm->Motor[MtrNo].CompDac = 0;
		}
	}
	else
	{
		pshm->Motor[MtrNo].CompDac = 0;
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

