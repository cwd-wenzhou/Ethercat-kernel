/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2006-2008  Florian Pose, Ingenieurgemeinschaft IgH
 *
 *  This file is part of the IgH EtherCAT Master.
 *
 *  The IgH EtherCAT Master is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License version 2, as
 *  published by the Free Software Foundation.
 *
 *  The IgH EtherCAT Master is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General
 *  Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with the IgH EtherCAT Master; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *****************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/err.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
#include <linux/slab.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif

#include "../../include/ecrt.h" // EtherCAT realtime interface

/*****************************************************************************/

// Module parameters
#define FREQUENCY 100

// Optional features
#define CONFIGURE_PDOS  1
#define EL3152_ALT_PDOS 0
#define EXTERNAL_MEMORY 0
#define SDO_ACCESS      0
#define VOE_ACCESS      0

#define PFX "ec_ep3e: "

/*****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};
struct semaphore master_sem;

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static int slaves_num;//记录从站个数
static unsigned int counter = 0;
static unsigned int blink = 0;
// Timer
static struct timer_list timer;

/*****************************************************************************/

// process data
static uint8_t *domain1_pd; // process data memory

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

    ec_slave_config_t *slave;  //从站配置，这里只有一台迈信伺服
    ec_slave_config_state_t slave_state;  //从站配置状态

    uint8_t *domain_pd;                     // Process Data
    struct DRIVERVARIABLE drive_variables;  //从站驱动器变量

    int32_t targetPosition;   //电机的目标位置
    int8_t opModeSet;         //电机运行模式的设定值,默认位置模式
    int8_t opmode;            //驱动器当前运行模式
    int32_t currentVelocity;  //电机当前运行速度
    int32_t currentPosition;  //电机当前位置
    uint16_t status;          //驱动器状态字

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

static struct MOTOR motors[4];
/*****************************************************************************/

#if SDO_ACCESS
static ec_sdo_request_t *sdo;
#endif

#if VOE_ACCESS
static ec_voe_handler_t *voe;
#endif

/*****************************************************************************/

void check_domain1_state(ec_domain_t *domain, ec_domain_state_t *domain_state)
{
    ec_domain_state_t ds;

    down(&master_sem);
    ecrt_domain_state(domain, &ds);
    up(&master_sem);

    if (ds.working_counter != domain_state->working_counter)
        printk(KERN_INFO PFX "Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain_state->wc_state)
        printk(KERN_INFO PFX "Domain1: State %u.\n", ds.wc_state);

    *domain_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    down(&master_sem);
    ecrt_master_state(master, &ms);
    up(&master_sem);

    if (ms.slaves_responding != master_state.slaves_responding)
        printk(KERN_INFO PFX "%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printk(KERN_INFO PFX "AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printk(KERN_INFO PFX "Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/*****************************************************************************/

void check_slave_config_states(ec_slave_config_t *slave,
                          ec_slave_config_state_t *slave_state)
{
    ec_slave_config_state_t s;

    down(&master_sem);
    ecrt_slave_config_state(slave, &s);
    up(&master_sem);

    if (s.al_state != slave_state->al_state)
        printk(KERN_INFO PFX "EP3E: State 0x%02X.\n", s.al_state);
    if (s.online != slave_state->online)
        printk(KERN_INFO PFX "EP3E: %s.\n", s.online ? "online" : "offline");
    if (s.operational != slave_state->operational)
        printk(KERN_INFO PFX "EP3E: %soperational.\n",
                s.operational ? "" : "Not ");

    *slave_state = s;
}

/*****************************************************************************/

#if SDO_ACCESS
void read_sdo(void)
{
    switch (ecrt_sdo_request_state(sdo)) {
        case EC_REQUEST_UNUSED: // request was not used yet
            ecrt_sdo_request_read(sdo); // trigger first read
            break;
        case EC_REQUEST_BUSY:
            printk(KERN_INFO PFX "Still busy...\n");
            break;
        case EC_REQUEST_SUCCESS:
            printk(KERN_INFO PFX "SDO value: 0x%04X\n",
                    EC_READ_U16(ecrt_sdo_request_data(sdo)));
            ecrt_sdo_request_read(sdo); // trigger next read
            break;
        case EC_REQUEST_ERROR:
            printk(KERN_INFO PFX "Failed to read SDO!\n");
            ecrt_sdo_request_read(sdo); // retry reading
            break;
    }
}
#endif

/*****************************************************************************/

#if VOE_ACCESS
void read_voe(void)
{
    switch (ecrt_voe_handler_execute(voe)) {
        case EC_REQUEST_UNUSED:
            ecrt_voe_handler_read(voe); // trigger first read
            break;
        case EC_REQUEST_BUSY:
            printk(KERN_INFO PFX "VoE read still busy...\n");
            break;
        case EC_REQUEST_SUCCESS:
            printk(KERN_INFO PFX "VoE received.\n");
            // get data via ecrt_voe_handler_data(voe)
            ecrt_voe_handler_read(voe); // trigger next read
            break;
        case EC_REQUEST_ERROR:
            printk(KERN_INFO PFX "Failed to read VoE data!\n");
            ecrt_voe_handler_read(voe); // retry reading
            break;
    }
}
#endif

/*****************************************************************************/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
void cyclic_task(struct timer_list *t)
#else
void cyclic_task(unsigned long data)
#endif
{
    // receive process data
    down(&master_sem);
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    up(&master_sem);

    // check process data state (optional)
    //check_domain1_state();

    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = FREQUENCY;

        // calculate new process data
        blink = !blink;

        // check for master state (optional)
        check_master_state();

        // check for islave configuration state(s) (optional)
        //check_slave_config_states();

#if SDO_ACCESS
        // read process data SDO
        read_sdo();
#endif

#if VOE_ACCESS
        read_voe();
#endif
    }

    // write process data
    //EC_WRITE_U8(domain1_pd + off_dig_out, blink ? 0x06 : 0x09);

    // send process data
    down(&master_sem);
    ecrt_domain_queue(domain1);
    ecrt_master_send(master);
    up(&master_sem);

    // restart timer
    timer.expires += HZ / FREQUENCY;
    add_timer(&timer);
}

/*****************************************************************************/

void send_callback(void *cb_data)
{
    ec_master_t *m = (ec_master_t *) cb_data;
    down(&master_sem);
    ecrt_master_send_ext(m);
    up(&master_sem);
}

/*****************************************************************************/

void receive_callback(void *cb_data)
{
    ec_master_t *m = (ec_master_t *) cb_data;
    down(&master_sem);
    ecrt_master_receive(m);
    up(&master_sem);
}

/*****************************************************************************/

int __init init_mini_module(void)
{
    int ret = -1;
    int i;

#if CONFIGURE_PDOS
    //ec_slave_config_t *sc;
#endif
#if EXTERNAL_MEMORY
    unsigned int size;
#endif

    printk(KERN_INFO PFX "Starting...\n");

    master = ecrt_request_master(0);
    if (!master) {
        ret = -EBUSY;
        printk(KERN_ERR PFX "Requesting master 0 failed.\n");
        goto out_return;
    }

    sema_init(&master_sem, 1);
    ecrt_master_callbacks(master, send_callback, receive_callback, master);
    check_master_state();
    slaves_num = master_state.slaves_responding;

    printk(KERN_INFO PFX "Registering domain...\n");
    if (!(domain1 = ecrt_master_create_domain(master))) {
        printk(KERN_ERR PFX "Domain creation failed!\n");
        goto out_release_master;
    }

#if CONFIGURE_PDOS
    
    for (i=0;i<slaves_num;i++){
        if (!(motors[i].slave = ecrt_master_slave_config(
                        master, 0,i,MAXSINE))) {
            printk(KERN_ERR PFX "Failed to get slave configuration.\n");
            goto out_release_master;
        }

        printk(KERN_INFO PFX "Configuring PDOs...\n");

        if (ecrt_slave_config_pdos(motors[i].slave, EC_END, EP3E_syncs)) {
            printk(KERN_ERR PFX "Failed to configure PDOs.\n");
            goto out_release_master;
        }

        printk(KERN_INFO PFX "Registering PDO entries...\n");
        ec_pdo_entry_reg_t domain_regs[] = {
        {0,i, MAXSINE, CTRL_WORD, 0, &motors[i].drive_variables.ctrl_word},
        {0,i, MAXSINE, OPERATION_MODE, 0,&motors[i].drive_variables.operation_mode},
        {0,i, MAXSINE, TARGET_VELOCITY, 0,&motors[i].drive_variables.target_velocity},
        {0,i, MAXSINE, TARGET_POSITION, 0,&motors[i].drive_variables.target_postion},
        {0,i, MAXSINE, STATUS_WORD, 0, &motors[i].drive_variables.status_word},
        {0,i, MAXSINE, MODE_DISPLAY, 0, &motors[i].drive_variables.mode_display},
        {0,i, MAXSINE, CURRENT_VELOCITY, 0,&motors[i].drive_variables.current_velocity},
        {0,i, MAXSINE, CURRENT_POSITION, 0,&motors[i].drive_variables.current_postion},
        {}};
        if (ecrt_domain_reg_pdo_entry_list(domain1, domain_regs)) {
            printk(KERN_ERR PFX "PDO entry registration failed!\n");
            goto out_release_master;
        }
    }
    printk(KERN_INFO PFX "PDO entry registration successed!\n");
#endif

 

#if SDO_ACCESS
    printk(KERN_INFO PFX "Creating SDO requests...\n");
    if (!(sdo = ecrt_slave_config_create_sdo_request(sc_ana_in, 0x3102, 2, 2))) {
        printk(KERN_ERR PFX "Failed to create SDO request.\n");
        goto out_release_master;
    }
    ecrt_sdo_request_timeout(sdo, 500); // ms
#endif

#if VOE_ACCESS
    printk(KERN_INFO PFX "Creating VoE handlers...\n");
    if (!(voe = ecrt_slave_config_create_voe_handler(sc_ana_in, 1000))) {
        printk(KERN_ERR PFX "Failed to create VoE handler.\n");
        goto out_release_master;
    }
#endif

    

#if EXTERNAL_MEMORY
    if ((size = ecrt_domain_size(domain1))) {
        if (!(domain1_pd = (uint8_t *) kmalloc(size, GFP_KERNEL))) {
            printk(KERN_ERR PFX "Failed to allocate %u bytes of process data"
                    " memory!\n", size);
            goto out_release_master;
        }
        ecrt_domain_external_memory(domain1, domain1_pd);
    }
#endif

    printk(KERN_INFO PFX "Activating master...\n");
    if (ecrt_master_activate(master)) {
        printk(KERN_ERR PFX "Failed to activate master!\n");
#if EXTERNAL_MEMORY
        goto out_free_process_data;
#else
        goto out_release_master;
#endif
    }

#if !EXTERNAL_MEMORY
    // Get internal process data for domain
    domain1_pd = ecrt_domain_data(domain1);
#endif

    printk(KERN_INFO PFX "Starting cyclic sample thread.\n");
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 15, 0)
    timer_setup(&timer, cyclic_task, 0);
#else
    init_timer(&timer);
    timer.function = cyclic_task;
#endif
    timer.expires = jiffies + 10;
    add_timer(&timer);

    printk(KERN_INFO PFX "Started.\n");
    return 0;

#if EXTERNAL_MEMORY
out_free_process_data:
    kfree(domain1_pd);
#endif
out_release_master:
    printk(KERN_ERR PFX "Releasing master...\n");
    ecrt_release_master(master);
out_return:
    printk(KERN_ERR PFX "Failed to load. Aborting.\n");
    return ret;
}

/*****************************************************************************/

void __exit cleanup_mini_module(void)
{
    printk(KERN_INFO PFX "Stopping...\n");

    del_timer_sync(&timer);

#if EXTERNAL_MEMORY
    kfree(domain1_pd);
#endif

    printk(KERN_INFO PFX "Releasing master...\n");
    ecrt_release_master(master);

    printk(KERN_INFO PFX "Unloading.\n");
}

/*****************************************************************************/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Florian Pose <fp@igh-essen.com>");
MODULE_DESCRIPTION("EtherCAT minimal test environment");

module_init(init_mini_module);
module_exit(cleanup_mini_module);

/*****************************************************************************/
