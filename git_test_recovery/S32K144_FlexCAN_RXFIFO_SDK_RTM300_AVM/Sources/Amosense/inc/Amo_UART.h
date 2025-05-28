#ifndef AMO_UART_H_
#define AMO_UART_H_

#include <stdarg.h>
#include "Cpu.h"
//#include "main_amo.h"

#define NEW_PWR_CMD

/* Welcome message displayed at the console */
#define welcomeMsg "S32K144 [ JPC UART0 ]\r\n"

/* Error message displayed at the console, in case data is received erroneously */
#define errorMsg "An error occurred! The application will stop!\r\n"
#define	defaultMsg "Switch default execution!\r\n"


/* Timeout in ms for blocking operations */
#define TIMEOUT     		(100U)

/* Receive buffer size */
#define BUFFER_SIZE (256u)

/* UART packet definition */
#define PWR_CMD						(0x2A)
#define MODE_CMD					(0x2B)
#define NATURAL_CMD				(0x2C)
#define DIR_CMD						(0x2D)
#define OPEN_CLOSE_CMD		(0X2E)

#define DIS_POLLING_CMD			(0x60) //DIS: Display App
#define DIS_INIT_CMD				(0x61)
#define DIS_SET_TARGET_CMD	(0x63)
#define DIS_BRAKE_CMD				(0x70)

#define SYS_INFO_CMD		(0x01)
#define SET_LIMIT_SPEED (0x02)
#define CMD_REQ					(0x10)
#define POLLING_CMD			(0x20) //SCB: Seat Control Board
#define INIT_CMD				(0x21)
#define UPDATE_CMD			(0x22)
#define SET_TARGET_CMD	(0x23)
#define BRAKE_CMD				(0x30)

#define RES_ACK				(0x00)
#define RES_NAK				(0X01)

#define STARTCODE			(0xAA)
#define ENDCODE				(0x55)
#define	CRETURN				(0x0D) //carriage return
#define	LINEFEED			(0x0A)


#define MIN_LENGTH			(5u)//(6u)	//packet length minimum value
#define MAX_LENGTH			(25u) //(15u)	//packet length maximum value //9ul
#define UART_IN_MAX 		(25u)	//maximum number of packetData

#define INFO_C2D_LENGTH 	(0x19)
#define POLLING_C2W_LENGTH	(0x08)
#define UPDATE_LENGTH		(0x08)

#define CLOSE_ANGLE_VAL		(80u) //80 degree

#define NAK_MSG			"NAK_MSG\r\n"


/*
#define START_IDX	0
#define CMD_IDX		1
#define LENGTH_IDX	2
*/

typedef enum
{
	startByte = 0,
	cmdByte,
	endByte
}RxPacketByte_Info;

typedef enum
{
	startIdx = 0,
	cmdIdx,
	lengthIdx,
	data0_Idx,
	data1_Idx,
	data2_Idx,
	data3_Idx,
	data4_Idx,
	data5_Idx
}PacketIndex_Info;

typedef enum
{
	startIdx_key = 0,
	cmdIdx_key,
	lengthIdx_key,
	destinationIdx_key,
	targetLittleIdx_key,
	checksumIdx_key=6,
	endIdx_key
}PacketIndex_Key_Info;
	
typedef enum
{
	startIdx_stop = 0,
	cmdIdx_stop,
	lengthIdx_stop,
	destinationIdx_stop,
	checksumIdx_stop,
	endIdx_stop
}PacketIndex__Stop_Info;
	
typedef enum
{
	vertiDis_lowByte_idx = 5,
	vertiDis_highByte_idx,
	maxVertiDis_lowByte_idx,
	maxVertiDis_highByte_idx,
	horizonDis_lowByte_idx,
	horizonDis_highByte_idx,
	maxHorizonDis_lowByte_idx,
	maxHorizonDis_highByte_idx
	
}TxPacketDis_C2D_t;

/*
typedef struct
{
	uint8_t uartMemClr;
	uint8_t uartCheckSum;
	uint8_t	uartCommErr;
}UartErrFlag_t;
*/


typedef struct
{
	uint8_t sPosi_LowByte;
	uint8_t sPosi_HighByte;
}SeatPosition_Value_t;


typedef struct
{
	uint8_t uartComm;
	uint8_t uartCheckSum;
	uint8_t	uartUndefined;
	uint8_t uartParam;
	uint8_t uartOpenLine;
	uint8_t uartAction;
}UartErrFlag_t;

/*
typedef struct
{
	uint8_t uartMemClr;
	uint8_t uartCheckSum;
	uint8_t	uartCommErr;
}Flexio_UartErrFlag_t;
*/

typedef struct
{
	uint8_t uartComm;
	uint8_t uartCheckSum;
	uint8_t	uartUndefined;
	uint8_t uartParam;
	uint8_t uartOpenLine;
	uint8_t uartAction;
}Flexio_UartErrFlag_t;

typedef struct rxPacketInfo_tag
{
	uint8_t startCode;
	uint8_t endCode;
	uint8_t cReturn;
	uint8_t lineFeed;
}RxPacketInfo_t;


typedef struct UartErrInfo_tag
{
	uint8_t noneErr;
	uint8_t commErr;
	uint8_t checksumErr;
	uint8_t undefinedErr;
	uint8_t paramErr;
	uint8_t openlineErr;
}UartErrInfo_t;

typedef struct AirVentNum_tag
{
	uint8_t Fdriver_Lvent;
	uint8_t Fdriver_Rvent;
	uint8_t Fpassenger_Lvent;
	uint8_t Fpassenger_Rvent;
}AirVentNum_t;

typedef struct AirVentMode_tag
{
	uint8_t Body_Led;
	uint8_t Leg;
	uint8_t Body;
	uint8_t Window_Leg;
}AirVentMode_t;



typedef struct AangleValue_tag
{
	int16_t vertiAngle;
	int16_t horizonAngle;
	int16_t offVertiAngle;
	int16_t offHorizonAngle;
	int16_t maxMinusAngle_Verti;
	int16_t maxMinusAngle_Horizon;
	int16_t	maxAngle_Verti;
	int16_t maxAngle_Horizon;
	uint8_t vertiOff_Value;
	uint8_t horizonOff_Value;
	uint8_t closeAngle_Value;

}AngleValue_t;

typedef struct PixelValue_tag
{
	int16_t vertiPixel;
	int16_t maxVertiPixel;
	int16_t horizonPixel;
	int16_t maxHorizonPixel;
	
}PixelValue_t;

typedef struct CurrentPos_tag
{
	uint16_t cPosOfSliding;
	uint16_t cPosOfHeight;
	uint16_t cPosOfSwivel;
	uint16_t cPosOfReclining;

	uint16_t cPosOfSliding_NewValue;
	uint16_t cPosOfSliding_OldValue;

}CurrentPos_t;

typedef struct targetPos_Ang_tag
{
	uint16_t slidingMotor;
	uint16_t HeightMotor;
	uint16_t swivelMotor;
	uint16_t recliningMotor;

	uint16_t slidingMotor_NewValue;
	uint16_t slidingMotor_OldValue;

}targetPos_Ang_t;



typedef struct OnOff_tag
{
	uint8_t On;
	uint8_t Off;

}OnOff_t;

typedef struct OpenClose_tag
{
	uint8_t Open;
	uint8_t Close;

}OpenClose_t;

#if defined(NEW_PWR_CMD)

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t pwr;
	uint8_t mode;
	uint8_t natural;
	uint8_t openClose_no1;
	uint8_t openClose_no2;
	uint8_t openClose_no3;
	uint8_t openClose_no4;
	uint8_t checksum;
	uint8_t end;	
}VentPower_D2C_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t pwr;
	uint8_t mode;
	uint8_t natural;
	uint8_t openClose_no1;
	uint8_t openClose_no2;
	uint8_t openClose_no3;
	uint8_t openClose_no4;
	uint8_t checksum;
	uint8_t end;	
}Flexio_VentPower_D2C_t;


typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t pwr;
	uint8_t mode;
	uint8_t natural;
	uint8_t openClose_no1;
	uint8_t openClose_no2;
	uint8_t openClose_no3;
	uint8_t openClose_no4;
	uint8_t checksum;
	uint8_t end;	

}VentPower_C2D_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t pwr;
	uint8_t mode;
	uint8_t natural;
	uint8_t openClose_no1;
	uint8_t openClose_no2;
	uint8_t openClose_no3;
	uint8_t openClose_no4;
	uint8_t checksum;
	uint8_t end;	

}Flexio_VentPower_C2D_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t dest;
	uint8_t targetPo_Ang;
	uint8_t checksum;
	uint8_t end;

}Set_TargetParam_C2M_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t dest;
	uint8_t checksum;
	uint8_t end;

}Brake_C2M_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t flagOfMoving;
	uint8_t flagOfForceStop;
	uint8_t cPosOfSliding_Lowbyte;
	uint8_t cPosOfSliding_Highbyte;
	uint8_t errorOfSlidng;
	uint8_t cPosOfHeight_Lowbyte;
	uint8_t cPosOfHeight_Highbyte;
	uint8_t errorOfHeight;
	uint8_t cPosOfSwivel_Lowbyte;
	uint8_t cPosOfSwivel_Highbyte;
	uint8_t errorOfSwivel;
	uint8_t cPosOfReclining_Lowbyte;
	uint8_t cPosOfReclining_Highbyte;
	uint8_t errorOfReclining;
	uint8_t checksum;
	uint8_t end;

}Polling_C2D_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t dest;
	uint8_t targetPo_Ang_Lowbyte;
	uint8_t targetPo_Ang_Highbyte;
	uint8_t cPosOfSliding_Highbyte;
	uint8_t checksum;
	uint8_t end;

}SetTargetParam_C2D_t;


typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t dest;
	uint8_t targetPo_Ang;
	uint8_t checksum;
	uint8_t end;

}Set_TargetParam_K2C_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t dest;
	uint8_t checksum;
	uint8_t end;

}Brake_K2C_t;

#else

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t pwr;
	uint8_t checksum;
	uint8_t end;	
}VentPower_D2C_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t pwr;
	uint8_t checksum;
	uint8_t end;	
}VentPower_C2D_t;
#endif /* NEW_PWR_CMD */

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t mode;
	uint8_t onoff;
	uint8_t checksum;
	uint8_t end;	
}VentMode_D2C_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t mode;
	uint8_t onoff;
	uint8_t checksum;
	uint8_t end;	
}Flexio_VentMode_D2C_t;


typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t mode;
	uint8_t onoff;
	uint8_t checksum;
	uint8_t end;	
}VentMode_C2D_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t mode;
	uint8_t onoff;
	uint8_t checksum;
	uint8_t end;	
}Flexio_VentMode_C2D_t;


typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t natural;
	uint8_t checksum;
	uint8_t end;
}VentNatural_D2C_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t natural;
	uint8_t checksum;
	uint8_t end;
}Flexio_VentNatural_D2C_t;


typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t natural;
	uint8_t checksum;
	uint8_t end;
}VentNatural_C2D_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t natural;
	uint8_t checksum;
	uint8_t end;
}Flexio_VentNatural_C2D_t;


#if 0
typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t ventNum;
	uint8_t vertiAngle;
	uint8_t horizonAngle;
	uint8_t checksum;
	uint8_t end;
}VentDirection_D2C_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t ventNum;
	uint8_t vertiAngle;
	uint8_t horizonAngle;
	uint8_t checksum;
	uint8_t end;
}VentDirection_C2D_t;
#else

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t ventNum;
	uint8_t checksum;
	uint8_t end;
}VentDirection_D2C_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t ventNum;
	uint8_t checksum;
	uint8_t end;
}Flexio_VentDirection_D2C_t;


typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t ventNum;
	uint8_t vertiDis_Lowbyte;
	uint8_t vertiDis_Highbyte;
	uint8_t maxVertiDis_Lowbyte;
	uint8_t maxVertiDis_Highbyte;
	uint8_t horizonDis_Lowbyte;
	uint8_t horizonDis_Highbyte;
	uint8_t maxHorizonDis_Lowbyte;
	uint8_t maxHorizonDis_Highbyte;
	uint8_t checksum;
	uint8_t end;

}VentDirection_C2D_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t ventNum;
	uint8_t vertiDis_Lowbyte;
	uint8_t vertiDis_Highbyte;
	uint8_t maxVertiDis_Lowbyte;
	uint8_t maxVertiDis_Highbyte;
	uint8_t horizonDis_Lowbyte;
	uint8_t horizonDis_Highbyte;
	uint8_t maxHorizonDis_Lowbyte;
	uint8_t maxHorizonDis_Highbyte;
	uint8_t checksum;
	uint8_t end;

}Flexio_VentDirection_C2D_t;

#endif

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t ventNum;
	uint8_t openClose;
	uint8_t checksum;
	uint8_t end;
}VentOpenClose_D2C_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t ventNum;
	uint8_t openClose;
	uint8_t checksum;
	uint8_t end;
}Flexio_VentOpenClose_D2C_t;


typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t ventNum;
	uint8_t openClose;
	uint8_t checksum;
	uint8_t end;
}VentOpenClose_C2D_t;

typedef struct
{
	uint8_t start;
	uint8_t cmd;
	uint8_t len;
	uint8_t response;
	uint8_t ventNum;
	uint8_t openClose;
	uint8_t checksum;
	uint8_t end;
}Flexio_VentOpenClose_C2D_t;



typedef union
{
	uint8_t 	rxPacket_data[UART_IN_MAX]; //10
	
	Polling_C2D_t	polling_C2D;
	SetTargetParam_C2D_t	setTarget_C2D;

	VentPower_D2C_t		pwr_D2C;
	VentMode_D2C_t		mode_D2C;
	VentNatural_D2C_t	natural_D2C;
	VentDirection_D2C_t	direction_D2C;
	VentOpenClose_D2C_t	openClose_D2C;

}UART_RxPacket;

typedef union
{
	uint8_t 	rxPacket_data_u1[UART_IN_MAX]; //10

	VentPower_D2C_t		pwr_D2C;
	VentMode_D2C_t		mode_D2C;
	VentNatural_D2C_t	natural_D2C;
	VentDirection_D2C_t	direction_D2C;
	VentOpenClose_D2C_t	openClose_D2C;

}UART_RxPacket_U1;

typedef union
{
	uint8_t 	flexio_rxPacket_data[UART_IN_MAX]; //10
	
	Flexio_VentPower_D2C_t		pwr_D2C;
	Flexio_VentMode_D2C_t		mode_D2C;
	Flexio_VentNatural_D2C_t	natural_D2C;
	Flexio_VentDirection_D2C_t	direction_D2C;
	Flexio_VentOpenClose_D2C_t	openClose_D2C;

}Flexio_UART_RxPacket;



typedef union
{
	uint8_t 	txPacket_data[UART_IN_MAX]; //15
	
	VentPower_C2D_t		pwr_C2D;
	VentMode_C2D_t		mode_C2D;
	VentNatural_C2D_t	natural_C2D;
	VentDirection_C2D_t	direction_C2D;
	VentOpenClose_C2D_t	openClose_C2D;

}UART_TxPacket;

typedef union
{
	uint8_t 	polling_txPacket_data[UART_IN_MAX]; //15

}Polling_TxPacket;

typedef union
{
	uint8_t 	update_txPacket_data[UART_IN_MAX]; //15

}Update_TxPacket;

typedef union
{
	uint8_t 	txPacket_data_u1[UART_IN_MAX]; //15
	
	VentPower_C2D_t		pwr_C2D;
	VentMode_C2D_t		mode_C2D;
	VentNatural_C2D_t	natural_C2D;
	VentDirection_C2D_t	direction_C2D;
	VentOpenClose_C2D_t	openClose_C2D;

}UART_TxPacket_U1;

typedef union
{
	uint8_t 	txPacket_data_u1_c2w[UART_IN_MAX]; //15

	VentPower_C2D_t		pwr_C2D;
	VentMode_C2D_t		mode_C2D;
	VentNatural_C2D_t	natural_C2D;
	VentDirection_C2D_t	direction_C2D;
	VentOpenClose_C2D_t	openClose_C2D;

}UART_TxPacket_U1_C2W;


typedef union
{
	uint8_t 	txPacket_data_key[UART_IN_MAX];

	Set_TargetParam_C2M_t		targetParam_C2M;
	Brake_C2M_t							brake_C2M;

}UART_KeyTxPacket;

typedef union
{
	uint8_t 	rxPacket_data_key[UART_IN_MAX];

	Set_TargetParam_K2C_t		targetParam_K2C;
	Brake_K2C_t							brake_K2C;

}UART_KeyRxPacket;

typedef union
{
	uint8_t 	flexio_txPacket_data[UART_IN_MAX]; //15
	
	Flexio_VentPower_C2D_t		pwr_C2D;
	Flexio_VentMode_C2D_t		mode_C2D;
	Flexio_VentNatural_C2D_t	natural_C2D;
	Flexio_VentDirection_C2D_t	direction_C2D;
	Flexio_VentOpenClose_C2D_t	openClose_C2D;

}Flexio_UART_TxPacket;


typedef struct 
{
	uint8_t	uartEventFlag;
	uint8_t	uartTimerFlag;
	uint8_t	uartStartFlag;
	uint8_t uartGarbageFlag;
	UartErrFlag_t uartErrFlag;
	uint8_t uartKeyFlag;
	uint8_t sliding_MovingFlag;
	uint8_t sliding_DirMovingFlag; //Direction = (Forward: True), (Backward: False)

}Uart_flag_t;

typedef struct 
{
	uint8_t	uartEventFlag;
	uint8_t	uartTimerFlag;
	uint8_t	uartStartFlag;
	uint8_t uartGarbageFlag;
	UartErrFlag_t uartErrFlag;
	uint8_t uartKeyFlag;

}Uart_flag_t_u1;

typedef struct 
{
	uint8_t	uartEventFlag;
	uint8_t	uartTimerFlag;
	uint8_t	uartStartFlag;
	uint8_t uartGarbageFlag;
	UartErrFlag_t uartErrFlag;
	uint8_t uartKeyFlag;
}Key_flag_t;

typedef struct 
{
	uint8_t	uartEventFlag;
	uint8_t	uartTimerFlag;
	uint8_t	uartStartFlag;
	uint8_t uartGarbageFlag;
	Flexio_UartErrFlag_t uartErrFlag;

}Flexio_Uart_flag_t;

typedef struct
{
	uint8_t Close_Flag;
	uint8_t Natural_Off_Flag;
	uint8_t windMode_FDR_Flag;
	uint8_t windMode_FPL_Flag;
	uint8_t Open_FDR_Flag; //FrontDriver Right
	uint8_t Open_FPL_Flag; //FrontPassanger Left
//	uint8_t Close_FDR_Flag;
//	uint8_t Close_FPL_Flag;
//	uint8_t Num_FDRH_Flag;
//	uint8_t Num_FDRV_Flag;
//	uint8_t Num_FPLH_Flag;
//	uint8_t Num_FPLV_Flag;
	
}VentStatus_flag_t;

typedef struct
{
	uint8_t 	txPacket_err[5];
	
}UART_ErrPacket;

typedef struct
{
	uint8_t 	txPacket_err[5];
	
}UART_ErrPacket_U1;


typedef struct
{
	uint8_t 	txPacket_err[5];
	
}Flexio_UART_ErrPacket;

extern Uart_flag_t			UART_FLAG;
extern Uart_flag_t_u1	UART_FLAG_U1;
extern Key_flag_t				KEY_FLAG;
extern UART_ErrPacket		UART_ERR_P;
extern UART_ErrPacket_U1		UART_ERR_P_U1;
extern Flexio_Uart_flag_t			FLEXIO_UART_FLAG;
extern Flexio_UART_ErrPacket		FLEXIO_UART_ERR_P;
extern uint8_t rxBuffer[BUFFER_SIZE];   /* Circular buffer for Rx */
extern volatile uint8_t rxWrite;       /* Write location in Rx buffer */
extern volatile uint8_t rxRead;        /* Read location in Rx buffer */
extern uint8_t txBuffer;
extern flexio_device_state_t flexIODeviceState;
extern flexio_uart_state_t   uartStateTX;
extern flexio_uart_state_t   uartStateRX;

#ifdef	AMO_UART_C_

uint8_t CheckSum_Check(UART_RxPacket *packet);
uint8_t Flexio_CheckSum_Check(Flexio_UART_RxPacket *packet);
uint8_t CheckSum_Check_key(UART_KeyRxPacket *packet);

uint8_t CheckSum_Cal_keyUart(UART_KeyTxPacket *packet);
uint8_t CheckSum_Cal(UART_TxPacket *packet);
uint8_t CheckSum_Cal_U1(UART_TxPacket_U1 *packet);
uint8_t CheckSum_Cal_Update(Update_TxPacket *packet);
uint8_t Flexio_CheckSum_Cal(Flexio_UART_TxPacket *packet);
uint8_t CheckSum_Cal_U1_C2W(UART_TxPacket_U1_C2W *packet);


uint8_t CalAngle_Verti(UART_TxPacket *uartTxDataBuf);
uint8_t CalAngle_Horizon(UART_TxPacket *uartTxDataBuf);
uint8_t CloseAngle_Verti(void);
uint8_t CloseAngle_Horizon(void);
uint16_t Demo_Rpm_Fix(UART_TxPacket *uartTxDataBuf);
void Led_Ctrl_OnPwrCmd(UART_TxPacket *uartTxDataBuf);



void Delay(volatile int cycles);

void MemClear(void);
void MemClear_U1(void);
void MemClear_Key(void);
void Flexio_MemClear(void);

void Req_Slave_Param(Flexio_UART_TxPacket *uartTxDataBuf);

void ToQueue_Packet(void);
void ToQueue_Packet_U1(void);
void Flexio_ToQueue_Packet(void);
void ToQueue_Packet_Key(void);
void ToQueue_Packet_Stop(void);

void UartParsing(UART_RxPacket *uartRxDataBuf, UART_TxPacket *uartTxDataBuf);
void UartParsing_U1(UART_RxPacket_U1 *uartRxDataBuf, UART_TxPacket_U1 *uartTxDataBuf);
//void Flexio_UartParsing(Flexio_UART_RxPacket *uartRxDataBuf, Flexio_UART_TxPacket *uartTxDataBuf);
void Flexio_UartParsing(Flexio_UART_RxPacket *uartRxDataBuf, Flexio_UART_TxPacket *uartTxDataBuf, UART_TxPacket_U1_C2W *uartTxDataBuf_u1);
void Key_UartParsing(UART_KeyRxPacket *uartRxDataBuf, UART_KeyTxPacket *uartTxDataBuf);

void ErrCmd_Send(Uart_flag_t *uart_flag, UART_ErrPacket *uart_errP);
void ErrCmd_Send_U1(Uart_flag_t_u1 *uart_flag, UART_ErrPacket_U1 *uart_errP);
void Flexio_ErrCmd_Send(Flexio_Uart_flag_t *uart_flag, Flexio_UART_ErrPacket *uart_errP);

void Uart_Init(void);
#ifdef FLEX_UART_ON
void Flexio_Uart_Init(void);
#endif
void Uart_GetReceive(void);
void Uart_GetReceive_U1(void);
void Flexio_Uart_GetReceive(void);

void Uart_GetStatus(void);

SeatPosition_Value_t GetBytesSeatPosition(uint8_t SeatPosition);

#else

extern uint8_t CheckSum_Check(UART_RxPacket *packet);
extern uint8_t CheckSum_Check_U1(UART_RxPacket_U1 *packet);
extern uint8_t Flexio_CheckSum_Check(Flexio_UART_RxPacket *packet);
extern uint8_t CheckSum_Check_key(UART_KeyRxPacket *packet);

extern uint8_t CheckSum_Cal(UART_TxPacket *packet);
extern uint8_t CheckSum_Cal_U1(UART_TxPacket_U1 *packet);
extern uint8_t CheckSum_Cal_Update(Update_TxPacket *packet);
extern uint8_t CheckSum_Cal_U1_C2W(UART_TxPacket_U1_C2W *packet);
extern uint8_t CheckSum_Cal_keyUart(UART_KeyTxPacket *packet);
extern uint8_t Flexio_CheckSum_Cal(Flexio_UART_TxPacket *packet);


extern uint8_t CalAngle_Verti(UART_TxPacket *uartTxDataBuf);
extern uint8_t CalAngle_Horizon(UART_TxPacket *uartTxDataBuf);
extern uint8_t CloseAngle_Verti(void);
extern uint8_t CloseAngle_Horizon(void);
extern uint16_t Demo_Rpm_Fix(UART_TxPacket *uartTxDataBuf);
extern void Led_Ctrl_OnPwrCmd(UART_TxPacket *uartTxDataBuf);


extern void Delay(volatile int cycles);

extern void MemClear(void);
extern void MemClear_U1(void);
extern void MemClear_Key(void);
extern void Flexio_MemClear(void);

extern void Req_Slave_Param(Flexio_UART_TxPacket *uartTxDataBuf);

extern void ToQueue_Packet(void);
extern void ToQueue_Packet_U1(void);
extern void Flexio_ToQueue_Packet(void);
extern void ToQueue_Packet_Key(void);
extern void ToQueue_Packet_Stop(void);

extern void UartParsing(UART_RxPacket *uartRxDataBuf, UART_TxPacket *uartTxDataBuf);
extern void UartParsing_U1(UART_RxPacket_U1 *uartRxDataBuf, UART_TxPacket_U1 *uartTxDataBuf);
//extern void Flexio_UartParsing(Flexio_UART_RxPacket *uartRxDataBuf, Flexio_UART_TxPacket *uartTxDataBuf);
extern void Flexio_UartParsing(Flexio_UART_RxPacket *uartRxDataBuf, Flexio_UART_TxPacket *uartTxDataBuf, UART_TxPacket_U1_C2W *uartTxDataBuf_u1);
extern void Key_UartParsing(UART_KeyRxPacket *uartRxDataBuf, UART_KeyTxPacket *uartTxDataBuf);

extern void ErrCmd_Send(Uart_flag_t *uart_flag, UART_ErrPacket *uart_errP);
extern void ErrCmd_Send_U1(Uart_flag_t_u1 *uart_flag, UART_ErrPacket_U1 *uart_errP);
extern void Flexio_ErrCmd_Send(Flexio_Uart_flag_t *uart_flag, Flexio_UART_ErrPacket *uart_errP);


extern void Uart_Init(void);
extern void Flexio_Uart_Init(void);

extern void Uart_GetReceive(void);
extern void Uart_GetReceive_U1(void);
extern void Flexio_Uart_GetReceive(void);

extern void Uart_GetStatus(void);

extern SeatPosition_Value_t GetBytesSeatPosition(uint8_t SeatPosition);

extern flexio_uart_state_t   uartStateTX;
#endif /* AMO_UART_C_ */

#endif /* AMO_UART_H_ */

