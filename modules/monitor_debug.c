#include "monitor_debug.h"

static RecPackTypeDef RecParam = {0};
static RecCommTypeDef RecComm = {0};

unsigned char strFlag = 1;
static COMM_DATA commData;
unsigned char  debugSel = 1;
float  Debug_Param[15] = {0};

extern FLOAT_GYRO gyroUnit;
extern FLOAT_ACC accUnit;
extern FLOAT_RPY curEur;

typedef enum
{
    _9Asix_IMU = 1,
    EurRatePID = 2,
    OSTaskSta,
    FlowSta,
    GPSSta,
    AltSta,
    RF_Sta,
    PosLoop,
    AltLoop,
    AttLoop,
    View_1,
    View_2,
} PageName;

void monitor_attitude(void){
	unsigned char *buffer = get_monitor_buffer_ptr();
    unsigned char *p;
    unsigned int count = 0;
	unsigned int len = 0;
	unsigned char uartSendFlag = 0;
	unsigned char strFlag = 0;
	static unsigned char oldDebugSel = 0;

	if(oldDebugSel != debugSel)
    {
        strFlag = 1;
        uartSendFlag = 1;
    }
    oldDebugSel = debugSel;
	static unsigned int timeCount = 0;
    timeCount ++;
    if(timeCount >= 200)
    {
        timeCount = 0;
        strFlag = 1;
        uartSendFlag = 1;
    }

    //param setting
    if(MonitorRecData(&RecParam))
    {
        if(RecParam.GetData.DataID < 12)
        {
            Debug_Param[RecParam.GetData.DataID] = RecParam.GetData.uData.fData;
        }
    }

    //command
    if(MonitorCommData(&RecComm))
    {
        //choose param list
        if(RecComm.CommData.CommType == 0) debugSel = RecComm.CommData.CommID;
        //spcial command
        if(RecComm.CommData.CommType == 2)
        {
            if(RecComm.CommData.CommID == 1)
            {
                __set_FAULTMASK(1);
                NVIC_SystemReset();
            };//reset MCU
            if(RecComm.CommData.CommID == 2) {}; //moto OFF
            if(RecComm.CommData.CommID == 3) {}; //ACC_cali
            if(RecComm.CommData.CommID == 4) {}; //MAG_cali
        }
        if(RecComm.CommData.CommType == 1)
        {
            if(RecComm.CommData.CommID == 1)
            {
                if(debugSel < 12)debugSel ++;
            }//Page +
            if(RecComm.CommData.CommID == 2)
            {
                if(debugSel > 0)
                    debugSel --;
            }//PAge -
        }
    }
	switch(debugSel)
    {
    case _9Asix_IMU:
		commData.data[0].f_data = gyroUnit.gyroX;
		commData.data[1].f_data = gyroUnit.gyroY;
		commData.data[2].f_data = gyroUnit.gyroZ;
		commData.data[3].f_data = curEur.Pitch;
		commData.data[4].f_data = curEur.Roll;
		commData.data[5].f_data = curEur.Yaw;
		commData.data[6].f_data = accUnit.accX;
		commData.data[7].f_data = accUnit.accY;
		commData.data[8].f_data = accUnit.accZ;
		if(strFlag == 1)
		{
		  strFlag = 0; 
		  p= (unsigned char*)"gyroX,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		  p= (unsigned char*)"gyroY,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		  p= (unsigned char*)"gyroZ,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		 
		  p= (unsigned char*)"Pitch,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		 
		  p= (unsigned char*)"Roll,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		 
		  p= (unsigned char*)"Yaw,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		 
		  p= (unsigned char*)"accX,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		 
		  p= (unsigned char*)"accY,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		 
		  p= (unsigned char*)"accZ,";
		  len = str_len(p);
		  str_add(buffer+count,p,len);
		  count += len;
		 
		  comm_str_send(buffer,count);
		}
		break;
    case EurRatePID :
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"Eur.Pitch,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"Eur.Rool,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"Eur.Yaw,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"exp.Pitch,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"exp.Rool,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"exp.Yaw,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"pidPitch.Oput,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)","  ;
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
    case OSTaskSta:
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"CPU_Usage,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"IMU_temperature,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"0,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"GyrPeace,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"BaroErr,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"task2_1s,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"task3_1s,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"task5_1s,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
    case FlowSta:
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0 ;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"FlowSpeed.x,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"FlowSpeed.y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"FlowaccBF_LPF.X,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"FlowaccBF_LPF.Y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;


            p = (unsigned char *)"states[0],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;


            p = (unsigned char *)"states[1],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"Flow_ExpAngPit,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;


            p = (unsigned char *)"Flow_ExpAngRol,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"Flow_alt,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
    case GPSSta:
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"Pos_N,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"Pos_E,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"Vel_N,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"VEL_E,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"hMSL,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"vel_x,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"vel_y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;


            p = (unsigned char *)"PosEst_x,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"PosEst_y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
		case AltSta:
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"Son_Dis,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"Sonar+Off,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

			
            p = (unsigned char *)"time_const_sonic,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"AutoTakeOff,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"sonicAlt+Offse,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"vel_acc,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"EstAlt,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"velRateCorr,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"GET_Sonic_STEP,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;

    case RF_Sta:
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"fixType,";

            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"D[1],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"D[2],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"D[3],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"D[4],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"D[5],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"D[6],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"D[7],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"D[8],";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
		case PosLoop:
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"PosEst_x,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"PosEst_y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"volt,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"vel_tar_x,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"vel_tar_y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"pos_tar_x,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"pos_tar_y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"curEur.Yaw,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
		case AltLoop:
		commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"AccCorVel,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"ESC_1,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"ESC_2,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"ESC_3,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"ESC_4,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"MotorErrFlag,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"TakeOffAlt,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"CurExpAlt,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"thrAltOut,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
    case AttLoop:
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"BF_X,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"BF_Y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"YawTar,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"pidPitch.Output,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"pidRoll.Output,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"PitRate.Err,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"RolRate.Err,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"----,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"----,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
    case View_1:
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"YawOutput,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)",";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
    case View_2:

        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)"EF_X,"; //gx_Off
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"EF_Y,"; //gy_Off
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"EF_Z,"; //gz_Off
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"BF_X,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"BF_Y,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"BF_Z,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"errEur_rol,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"errEur_pit,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"errEur_yaw,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
    default:
        debugSel = 1;
        commData.data[0].f_data = 0;
        commData.data[1].f_data = 0;
        commData.data[2].f_data = 0;
        commData.data[3].f_data = 0;
        commData.data[4].f_data = 0;
        commData.data[5].f_data = 0;
        commData.data[6].f_data = 0;
        commData.data[7].f_data = 0;
        commData.data[8].f_data = 0;
        if(strFlag == 1)
        {
            strFlag = 0;
            p = (unsigned char *)","; //gx_Off
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)","; //gy_Off
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)","; //gz_Off
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"----,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"----,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"----,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"----,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"----,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            p = (unsigned char *)"----,";
            len = str_len(p);
            str_add(buffer + count, p, len);
            count += len;

            comm_str_send(buffer, count);
        }
        break;
    
	}
	if(uartSendFlag == 0){
		comm_data_send(&commData);
	}
}

