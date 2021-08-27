#ifndef  EP3E
#define EP3E
#include "../../include/ecrt.h" // EtherCAT realtime interface

//从站配置所用的参数
#define EP3ESLAVEPOS 0, i               //迈信伺服EP3E在ethercat总线上的位置
#define MAXSINE 0x000007DD, 0x00000001  // EP3E的厂家标识和产品标识

//电机配置需要的参数
#define TASK_FREQUENCY 4000        //*Hz* 任务周期
#define ENCODER_RESOLUTION 131072  //编码器分辨率
#define HOME_VECOLITY 5            // r/s，回零速度
#define HOME_STEP HOME_VECOLITY *ENCODER_RESOLUTION / TASK_FREQUENCY  // pulse 回零步长
#define POSITION_STEP 1 / TASK_FREQUENCY  //位置模式下步长
#define Pi 3.141592654                    //圆周率

// CoE对象字典
#define RXPDO 0x1600
#define TXPDO 0x1A00
/*CiA 402数据对象(Data Object)*/
#define CTRL_WORD 0x6040         //控制字的数据对象
#define OPERATION_MODE 0x6060    //设定运行模式的数据对象
#define TARGET_VELOCITY 0x60FF   //目标速度的数据对象
#define TARGET_POSITION 0x607A   //目标位置的数据对象
#define STATUS_WORD 0x6041       //状态字的数据对象
#define MODE_DISPLAY 0x6061      //当前运行模式的数据对象
#define CURRENT_VELOCITY 0x606C  //当前速度的数据对象
#define CURRENT_POSITION 0x6064  //当前位置的数据对象


enum DRIVERSTATE {
    dsNotReadyToSwitchOn = 0,  //初始化 未完成状态
    dsSwitchOnDisabled,        //初始化 完成状态
    dsReadyToSwitchOn,         //主电路电源OFF状态
    dsSwitchedOn,              //伺服OFF/伺服准备
    dsOperationEnabled,        //伺服ON
    dsQuickStopActive,         //即停止
    dsFaultReactionActive,     //异常（报警）判断
    dsFault                    //异常（报警）状态
};

//迈信伺服驱动器里PDO入口的偏移量
/*我们需要定义一些变量去关联需要用到的从站的PD0对象*/
struct DRIVERVARIABLE {
    unsigned int operation_mode;   //设定运行模式
    unsigned int ctrl_word;        //控制字
    unsigned int target_velocity;  //目标速度 （pulse/s)
    unsigned int target_postion;   //目标位置 （pulse）

    unsigned int status_word;       //状态字
    unsigned int mode_display;      //实际运行模式
    unsigned int current_velocity;  //当前速度 （pulse/s）
    unsigned int current_postion;   //当前位置 （pulse）
};

//迈信伺服电机结构体
struct MOTOR {
    int targetPosition;   //电机的目标位置
    int opModeSet;         //电机运行模式的设定值,默认位置模式
    int opmode;            //驱动器当前运行模式
    int currentVelocity;  //电机当前运行速度
    int currentPosition;  //电机当前位置
    int status;          //驱动器状态字
    char str[20];//for test

    ec_slave_config_t *slave;  //从站配置，这里只有一台迈信伺服
    ec_slave_config_state_t slave_state;  //从站配置状态

    uint8_t *domain_pd;                     // Process Data
    struct DRIVERVARIABLE drive_variables;  //从站驱动器变量
    enum DRIVERSTATE driveState;  //驱动器状态
    
};

//填充相关PDOS信息
ec_pdo_entry_info_t EP3E_pdo_entries[] = {/*RxPdo 0x1600*/
                                            {CTRL_WORD, 0x00, 16},
                                            {OPERATION_MODE, 0x00, 8},
                                            {TARGET_VELOCITY, 0x00, 32},
                                            {TARGET_POSITION, 0x00, 32},
                                            /*TxPdo 0x1A00*/
                                            {STATUS_WORD, 0x00, 16},
                                            {MODE_DISPLAY, 0x00, 8},
                                            {CURRENT_VELOCITY, 0x00, 32},
                                            {CURRENT_POSITION, 0x00, 32}};
ec_pdo_info_t EP3E_pdos[] = {// RxPdo
                                {RXPDO, 4, EP3E_pdo_entries + 0},
                                // TxPdo
                                {TXPDO, 4, EP3E_pdo_entries + 4}};
ec_sync_info_t EP3E_syncs[] = {{0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
                                {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
                                {2, EC_DIR_OUTPUT, 1, EP3E_pdos + 0, EC_WD_DISABLE},
                                {3, EC_DIR_INPUT, 1, EP3E_pdos + 1, EC_WD_DISABLE},
                                {0xFF}};

#endif //EP3E