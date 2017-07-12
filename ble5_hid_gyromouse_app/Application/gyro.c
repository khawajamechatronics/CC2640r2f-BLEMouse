/*
 * kalman.c
 *
 *  Created on: Jul 8, 2017
 *      Author: Henorvell
 */
#include "mpu6500.h"
#include "util.h"

#include <icall.h>
#include <stdint.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SPI.h>

#include "icall_ble_api.h"
//#include "ll_common.h"

#include "peripheral.h"

#include "gyro.h"
#include "hidemukbd.h"
#include "board.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
KALMAN_STRUCT *kalman;
char gyro_work = 0;
// Entity ID globally used to check for source and/or destination of messages
//static ICall_EntityID gyroEntity;

// Event globally used to post local events and pend on system and
// local events.
//static ICall_SyncHandle gyroEvent;

// Task configuration
Task_Struct gyroTask;
Char gyroTaskStackSize[644];

/* Pin driver handles */
static PIN_Handle ledPinHandle;

static PIN_State ledPinState;

PIN_Config ledPinTable[] = {
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

#define GR_STATE_CHANGE_EVT                    Event_Id_05

typedef struct
{
  appEvtHdr_t hdr; // Event header
  uint8_t x,y;
} mousepos_t;

static void Gyro_init(void);
static void Gyro_taskFxn(UArg a0, UArg a1);
static uint8_t Gyro_enqueueMsg(uint8_t x, uint8_t y);

void set_gyro_work(char a)
{
    gyro_work=a;
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED0, a);
    PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, a);
}

void Gyro_createTask(void)
{
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stack = gyroTaskStackSize;
    taskParams.stackSize = 644;
    taskParams.priority = 1;
    Task_construct(&gyroTask, Gyro_taskFxn, &taskParams, NULL);
}

void Gyro_init(void)
{
    kalman = malloc(sizeof(KALMAN_STRUCT));
    Kanman_Init(kalman);
    SPI_init();
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    //ICall_registerApp(&gyroEntity, &gyroEvent);
    //appMsgQueue = Util_constructQueue(&appMsg);
}

void Gyro_taskFxn(UArg a0, UArg a1)
{
    // Initialize the application.
    Gyro_init();
    // Application main loop.
    for (;;)
    {
        if(gyro_work)
        {
            //Gyro_enqueueMsg(10,10);
        }
        //Semaphore_pend(semHandle, BIOS_WAIT_FORERVER);
    }
}

static uint8_t Gyro_enqueueMsg(uint8_t x, uint8_t y)
{
  mousepos_t *pMsg;

  // Create dynamic pointer to message.
  if (pMsg = ICall_malloc(sizeof(mousepos_t)))
  {
    //pMsg->hdr.event = event;
    pMsg->hdr.event = GR_STATE_CHANGE_EVT;
    pMsg->hdr.state = NULL;

    // Enqueue the message.
    //return Util_enqueueMsg(appMsgQueue, syncEvent, (uint8_t *)pMsg);
  }

  return FALSE;
}

void Kanman_Init(KALMAN_STRUCT * kalman)
{
    int i;
    //输出
    (*kalman).Angel = 0.0;  //最优估计的角度   是最终角度结果
    (*kalman).Gyro_x = 0.0; //最优估计角速度

    //固定参量
    (*kalman).Q_Angle = 0.001;      //{0.001,0.001,0.001};  //陀螺仪噪声协方差  0.001是经验值
    (*kalman).Q_Gyro = 0.003;       //{0.003,0.003,0.003};  //陀螺仪漂移噪声协方差    是mpu6050的经验值
    (*kalman).R_Angle = 0.5;        //{0.5,0.5,0.5};    //是加速度计噪声的协方差

    (*kalman).C_0 = 1;      //{1,1,1};  //H矩阵的一个观测参数 是常数

    //中间量
    (*kalman).Q_Bias = 0;       //{0,0,0};      //陀螺仪飘移预估值
    (*kalman).Angle_err = 0;    //{0,0,0};      //计算中间值 Angle 观测值-预估值

    (*kalman).PCt_0 = 0;            //{0,0,0},  //计算中间值
    (*kalman).PCt_1 = 0;            //{0,0,0},
    (*kalman).E     = 0;            //{0,0,0};
    (*kalman).t_0   = 0;            //{0,0,0},  //t:计算中间变量
    (*kalman).t_1   = 0;            //{0,0,0},

    (*kalman).K_0 = 0;          //{0,0,0},  //K:卡尔曼增益
    (*kalman).K_1 = 0;          //{0,0,0},

    for(i = 0;i < 4;i++)    //{0,0,0,0} //计算P矩阵的中间矩阵
    {
        (*kalman).Pdot[i] = 0;
    }

    (*kalman).PP[0][0] = 1;
    (*kalman).PP[0][1] = 0;
    (*kalman).PP[1][0] = 0;
    (*kalman).PP[1][1] = 1;
}


void Kanman_Filter(KALMAN_STRUCT * kalman,float Gyro,float Accel,unsigned int dt)    //Gyro陀螺仪的测量值  |  Accel加速度计的角度计  |  dt的时间考虑用小数 或 更小的分度表示
{
    float dt_f;
    //把dt这个单位是ms的u32型变量里的值转换为float型的以秒为单位的值
    dt_f = (float)dt;
    dt_f = dt_f / 1000;
    //x轴指向前，y轴指向左的坐标系  要算俯仰角
    //那么输入的应该是y轴的角速度（Gyro）和y轴的倾角加速度计估计值
    //坐标系情况大概是这样
    //角度测量模型方程 角度估计值=上一次最有角度+（角速度-上一次的最优零飘）*dt_f
    //就漂移来说，认为每次都是相同的Q_bias=Q_bias
    //估计角度
    (*kalman).Angel += (Gyro - (*kalman).Q_Bias) * dt_f;

    //计算估计模型的方差
    (*kalman).Pdot[0] = (*kalman).Q_Angle - (*kalman).PP[0][1] - (*kalman).PP[1][0];
    (*kalman).Pdot[1] = -(*kalman).PP[1][1];
    (*kalman).Pdot[2] = -(*kalman).PP[1][1];
    (*kalman).Pdot[3] = (*kalman).Q_Gyro;

    (*kalman).PP[0][0] += (*kalman).Pdot[0] * dt_f;
    (*kalman).PP[0][1] += (*kalman).Pdot[1] * dt_f;
    (*kalman).PP[1][0] += (*kalman).Pdot[2] * dt_f;
    (*kalman).PP[1][1] += (*kalman).Pdot[3] * dt_f;

    //计算卡尔曼增益
    (*kalman).PCt_0 = (*kalman).C_0 * (*kalman).PP[0][0];   //矩阵乘法的中间变量
    (*kalman).PCt_1 = (*kalman).C_0 * (*kalman).PP[0][1];   //C_0=1
    (*kalman).E = (*kalman).R_Angle + (*kalman).C_0 * (*kalman).PCt_0;  //分母
    (*kalman).K_0 = (*kalman).PCt_0 / (*kalman).E;  //卡尔曼增益，两个，一个是Angle的，一个是Q_bias的
    (*kalman).K_1 = (*kalman).PCt_1 / (*kalman).E;

    //计算最优角度、最优零飘
    (*kalman).Angle_err = Accel - (*kalman).Angel;
    (*kalman).Angel += (*kalman).K_0 * (*kalman).Angle_err; //计算最优的角度
    (*kalman).Q_Bias += (*kalman).K_1 * (*kalman).Angle_err;    //计算最优的零飘

    (*kalman).Gyro_x = Gyro -(*kalman).Q_Bias;  //计算得最优角速度

    //更新估计模型的方差
    (*kalman).t_0 = (*kalman).PCt_0;    //矩阵计算中间变量，相当于a
    (*kalman).t_1 = (*kalman).C_0 * (*kalman).PP[0][1]; //矩阵计算中间变量，相当于b

    (*kalman).PP[0][0] -= (*kalman).K_0 * (*kalman).t_0;
    (*kalman).PP[0][1] -= (*kalman).K_0 * (*kalman).t_1;
    (*kalman).PP[1][0] -= (*kalman).K_1 * (*kalman).t_0;
    (*kalman).PP[1][1] -= (*kalman).K_1 * (*kalman).t_1;
}

int MPU6500_Init(unsigned int lpf)
{
    unsigned int default_filter = 1;

    //选择mpu6500内部数字低筒滤波器带宽
    //不开启内部低通滤波，陀螺仪采样率 8MHz
    //  开启内部低通滤波，陀螺仪采样率 1MHz
    //加速度计采样率1MHz
    switch(lpf)
    {
    case 5:
        default_filter = MPU6500_DLPF_BW_5;
        break;
    case 10:
        default_filter = MPU6500_DLPF_BW_10;
        break;
    case 20:
        default_filter = MPU6500_DLPF_BW_20;
        break;
    case 42:
        default_filter = MPU6500_DLPF_BW_42;
        break;
    case 98:
        default_filter = MPU6500_DLPF_BW_98;
        break;
    case 188:
        default_filter = MPU6500_DLPF_BW_188;
        break;
    case 256:
        default_filter = MPU6500_DLPF_BW_256;
        break;
    default:
        default_filter = MPU6500_DLPF_BW_42;
        break;
    }

    //delay_ms(200);

    /*//设备复位
//  IIC_Write_1Byte(MPU6500_SLAVE_ADDRESS,MPU6500_RA_PWR_MGMT_1, 0x80);

    //这里使用的Delay()只能在初始化阶段使用，任务调度中使用这种Delay()，会卡死整个调度
    MPU6500_setSleepEnabled(0); //进入工作状态
    delay_ms(10);
    MPU6500_setClockSource(MPU6500_CLOCK_PLL_ZGYRO);    //设置时钟  0x6b   0x03
                                                        //时钟源选择，MPU6500_CLOCK_INTERNAL表示内部8M晶振
    delay_ms(10);
    MPU6500_set_SMPLRT_DIV(1000);  //1000hz
    delay_ms(10);
    MPU6500_setFullScaleGyroRange(MPU6500_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
    delay_ms(10);
    MPU6500_setFullScaleAccelRange(MPU6500_ACCEL_FS_8); //加速度度最大量程 +-8G
    delay_ms(10);
    MPU6500_setDLPF(default_filter);  //42hz
    delay_ms(10);
    MPU6500_setI2CMasterModeEnabled(0);  //不让MPU6500 控制AUXI2C
    delay_ms(10);
    MPU6500_setI2CBypassEnabled(1);  //主控制器的I2C与    MPU6500的AUXI2C  直通。控制器可以直接访问HMC5883L
    delay_ms(10);*/

    return 0;
}

//读取MPU6500输出寄存器数值
void MPU6500_Read(MPU6500_STRUCT * mpu6500)
{
    //IIC_Read_nByte(MPU6500_SLAVE_ADDRESS,MPU6500_RA_ACCEL_XOUT_H,14,mpu6500->mpu6500_buffer);

    /*拼接buffer原始数据*/
    mpu6500->Acc_I16.x = ((((int16_t)mpu6500->mpu6500_buffer[0]) << 8) | mpu6500->mpu6500_buffer[1]) ;
    mpu6500->Acc_I16.y = ((((int16_t)mpu6500->mpu6500_buffer[2]) << 8) | mpu6500->mpu6500_buffer[3]) ;
    mpu6500->Acc_I16.z = ((((int16_t)mpu6500->mpu6500_buffer[4]) << 8) | mpu6500->mpu6500_buffer[5]) ;

    mpu6500->Gyro_I16.x = ((((int16_t)mpu6500->mpu6500_buffer[8]) << 8) | mpu6500->mpu6500_buffer[9]) ;
    mpu6500->Gyro_I16.y = ((((int16_t)mpu6500->mpu6500_buffer[10]) << 8) | mpu6500->mpu6500_buffer[11]) ;
    mpu6500->Gyro_I16.z = ((((int16_t)mpu6500->mpu6500_buffer[12]) << 8) | mpu6500->mpu6500_buffer[13]) ;

    mpu6500->Tempreature = ((((int16_t)mpu6500->mpu6500_buffer[6]) << 8) | mpu6500->mpu6500_buffer[7]);

}
