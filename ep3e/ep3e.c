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
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/errno.h>

#include <linux/mm_types.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
//#include <asm/system.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/uaccess.h>

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
#include "ep3e.h"
#include "memdevice.h"
/*****************************************************************************/

// Module parameters
#define FREQUENCY 100

// Optional features
#define CONFIGURE_PDOS  1
#define EL3152_ALT_PDOS 0
#define EXTERNAL_MEMORY 0
#define SDO_ACCESS      0
#define VOE_ACCESS      0
#define DEVICE_CREATE   1
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

int8_t reset_count = 0;      //复位的时候计数用
int16_t position_count = 0;  //位置模式下计数
double ep3e_t;
// Timer
static struct timer_list timer;

/*****************************************************************************/

// process data
static uint8_t *domain1_pd; // process data memory

//static struct mem_devp[i].data->[4];
/*****************************************************************************/

/*****************************************************************************/
/******************************device create*****************************************/
static int mem_major = MEMDEV_MAJOR;

module_param(mem_major, int, S_IRUGO);//insmod sharedmem mem_major=5  这样就可从命令行传参数进来。若没有指定则使用上一行的默认值

struct mem_dev *mem_devp; /*设备结构体指针*/
struct cdev cdev;
static struct class *demo_class; 

/*文件打开函数*/
int mem_open(struct inode *inode, struct file *filp)
{
    struct mem_dev *dev;
    
    /*获取次设备号*/
    int num = MINOR(inode->i_rdev);
    if (num >= MEMDEV_MAX_DEVS)
            return -ENODEV;
    dev = &mem_devp[num];
    /*将设备描述结构指针赋值给文件私有数据指针*/
    filp->private_data = dev;
    printk(KERN_INFO PFX "mem_open RUN.data.str=%s\n",dev->data->str);
    return 0;
}

/*文件释放函数*/
int mem_release(struct inode *inode, struct file *filp)
{
  printk(KERN_INFO PFX "mem_release RUN.\n");
  return 0;
}


ssize_t mem_read(struct file *file, char __user *buf,size_t count, loff_t *offset)
{
  printk(KERN_INFO PFX "mem_read RUN.\n");
  return 0;
}

ssize_t mem_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
  printk(KERN_INFO PFX "mem_write RUN.\n");
  return 0;
}

static int mem_mmap(struct file* filp, struct vm_area_struct *vma)
{
  struct mem_dev *dev = filp->private_data; /*获得设备结构体指针*/
  printk(KERN_INFO PFX "memdev_mmap RUN.\n");
  vma->vm_flags |= VM_IO;
  vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
  if (remap_pfn_range(vma,vma->vm_start,virt_to_phys(dev->data)>>PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot))
      return  -EAGAIN;
  printk(KERN_INFO PFX "mmap func runs fine,str=%s targetposition=%d\n",dev->data->str,dev->data->targetPosition);
  return 0;
}

/*文件操作结构体*/
static const struct file_operations mem_fops =
{
  .owner = THIS_MODULE,
  .open = mem_open,
  .release = mem_release,
  .read = mem_read,
  .write = mem_write,
  .mmap = mem_mmap,
};

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
    int i;
    // receive process data
    down(&master_sem);
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);
    up(&master_sem);

    // check process data state (optional)
    check_domain1_state(domain1,&domain1_state);

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

    for (i=0;i<slaves_num;i++){
        mem_devp[i].data->status = EC_READ_U16(domain1_pd+mem_devp[i].data->drive_variables.status_word);
        mem_devp[i].data->opmode = EC_READ_U8(domain1_pd+mem_devp[i].data->drive_variables.mode_display);
        mem_devp[i].data->currentVelocity = EC_READ_S32(domain1_pd+mem_devp[i].data->drive_variables.current_velocity);
        mem_devp[i].data->currentPosition = EC_READ_S32(domain1_pd+mem_devp[i].data->drive_variables.current_postion);

        if((mem_devp[i].data->status & 0x004F) == 0x0000)
            mem_devp[i].data->driveState = dsNotReadyToSwitchOn;  //初始化 未完成状态
        else if((mem_devp[i].data->status & 0x004F) == 0x0040)
            mem_devp[i].data->driveState = dsSwitchOnDisabled;  //初始化 完成状态
        else if((mem_devp[i].data->status & 0x006F) == 0x0021)
            mem_devp[i].data->driveState = dsReadyToSwitchOn;  //主电路电源OFF状态
        else if((mem_devp[i].data->status & 0x006F) == 0x0023)
            mem_devp[i].data->driveState = dsSwitchedOn;  //伺服OFF/伺服准备
        else if((mem_devp[i].data->status & 0x006F) == 0x0027)
            mem_devp[i].data->driveState = dsOperationEnabled;  //伺服ON
        else if((mem_devp[i].data->status & 0x006F) == 0x0007)
            mem_devp[i].data->driveState = dsQuickStopActive;  //即停止
        else if((mem_devp[i].data->status & 0x004F) == 0x000F)
            mem_devp[i].data->driveState = dsFaultReactionActive;  //异常（报警）判断
        else if((mem_devp[i].data->status & 0x004F) == 0x0008)
            mem_devp[i].data->driveState = dsFault;  //异常（报警）状态

        if(mem_devp[i].data->powerBusy == true) {
            switch(mem_devp[i].data->driveState) {
                case dsNotReadyToSwitchOn:
                    break;

                case dsSwitchOnDisabled:
                    //设置运行模式
                    EC_WRITE_S8(domain1_pd + mem_devp[i].data->drive_variables.operation_mode,mem_devp[i].data->opModeSet);
                    EC_WRITE_U16(domain1_pd + mem_devp[i].data->drive_variables.ctrl_word,0x0006);
                    break;

                case dsReadyToSwitchOn:
                    EC_WRITE_U16(domain1_pd + mem_devp[i].data->drive_variables.ctrl_word,0x0007);
                    break;

                case dsSwitchedOn:
                    EC_WRITE_U16(domain1_pd + mem_devp[i].data->drive_variables.ctrl_word,0x000f);  // enable operation
                    mem_devp[i].data->targetPosition = mem_devp[i].data->currentPosition;  //将当前位置复制给目标位置，防止使能后电机震动
                    EC_WRITE_S32(domain1_pd + mem_devp[i].data->drive_variables.target_postion, mem_devp[i].data->targetPosition);
                    break;
                default:
                    mem_devp[i].data->powerBusy = false;
            }
        }
        mem_devp[i].data->targetVelocity=ENCODER_RESOLUTION/2/Pi*5;
        if(mem_devp[i].data->driveState == dsOperationEnabled && mem_devp[i].data->resetBusy == 0 &&
            mem_devp[i].data->powerBusy == 0 && mem_devp[i].data->quickStopBusy == 0) {
            if(mem_devp[i].data->opmode == 9) {  //速度模式
                EC_WRITE_S32(domain1_pd + mem_devp[i].data->drive_variables.target_velocity,mem_devp[i].data->targetVelocity);
            }
        }
    }

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
    dev_t devno = MKDEV(mem_major, 0);
    struct device *demo_device;
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

#if DEVICE_CREATE
    slaves_num=4;
      /* 静态申请设备号*/
  if (mem_major)
    ret = register_chrdev_region(devno, slaves_num, "memshare");
  else  /* 动态分配设备号 */
  {
    ret = alloc_chrdev_region(&devno, 0, slaves_num, "memshare");
    mem_major = MAJOR(devno);
    
  } 
 
  if (ret < 0){
    printk("chrdev failed\n");
    return ret;
  }
    
  /*初始化cdev结构*/
  cdev_init(&cdev, &mem_fops);
  cdev.owner = THIS_MODULE;
  cdev.ops = &mem_fops;
  
  /*创建设备类*/  
  demo_class = class_create(THIS_MODULE,"demo_class");  
  if(IS_ERR(demo_class)){  
    ret =  PTR_ERR(demo_class);  
    goto class_err;  
  }  

  /*创建设备文件，通知用户在“/dev/”目录下创件名字为demoX的设备文件*/  
  for(i=0; i<slaves_num; i++){ //最多可创建255个设备节点(register_chrdev函数会申请0-254范围的从设备号)  
    /* 注册字符设备 */
    cdev_add(&cdev, MKDEV(mem_major, i), 1);
    demo_device = device_create(demo_class,NULL, MKDEV(mem_major, i), NULL,"etcdevice%d",i);  
    if(IS_ERR(demo_device)){  
      ret = PTR_ERR(demo_device);  
      goto device_err;  
    }  
  }      

  /* 为设备描述结构分配内存*/
  mem_devp = kmalloc(slaves_num * sizeof(struct mem_dev), GFP_KERNEL);
  
  if (!mem_devp)    /*申请失败*/
  {
    ret =  - ENOMEM;
    goto fail_malloc;
  }
  memset(mem_devp, 0, sizeof(struct mem_dev));

  /*为设备分配内存*/
  for (i=0; i < slaves_num; i++)
  {
    mem_devp[i].size = MEMDEV_SIZE;
    mem_devp[i].data = kmalloc(MEMDEV_SIZE, GFP_KERNEL);
    memset(mem_devp[i].data, 0, MEMDEV_SIZE);
  } 

  strcpy(mem_devp[0].data->str,"ly ans cwd is in e308");
  mem_devp[0].data->powerBusy =1;
  mem_devp[0].data->positionMoving =1;
  mem_devp[0].data->opModeSet = 9;
  strcpy(mem_devp[1].data->str,"cwd isn't in c412");
  printk(KERN_INFO PFX "malloc finished.\n");  
#endif

#if CONFIGURE_PDOS
    
    for (i=0;i<slaves_num;i++){
        if (!(mem_devp[i].data->slave = ecrt_master_slave_config(
                        master, 0,i,MAXSINE))) {
            printk(KERN_ERR PFX "Failed to get slave configuration.\n");
            goto out_release_master;
        }

        printk(KERN_INFO PFX "Configuring PDOs...\n");

        if (ecrt_slave_config_pdos(mem_devp[i].data->slave, EC_END, EP3E_syncs)) {
            printk(KERN_ERR PFX "Failed to configure PDOs.\n");
            goto out_release_master;
        }

        printk(KERN_INFO PFX "Registering PDO entries...\n");
        ec_pdo_entry_reg_t domain_regs[] = {
        {0,i, MAXSINE, CTRL_WORD, 0, &mem_devp[i].data->drive_variables.ctrl_word},
        {0,i, MAXSINE, OPERATION_MODE, 0,&mem_devp[i].data->drive_variables.operation_mode},
        {0,i, MAXSINE, TARGET_VELOCITY, 0,&mem_devp[i].data->drive_variables.target_velocity},
        {0,i, MAXSINE, TARGET_POSITION, 0,&mem_devp[i].data->drive_variables.target_postion},
        {0,i, MAXSINE, STATUS_WORD, 0, &mem_devp[i].data->drive_variables.status_word},
        {0,i, MAXSINE, MODE_DISPLAY, 0, &mem_devp[i].data->drive_variables.mode_display},
        {0,i, MAXSINE, CURRENT_VELOCITY, 0,&mem_devp[i].data->drive_variables.current_velocity},
        {0,i, MAXSINE, CURRENT_POSITION, 0,&mem_devp[i].data->drive_variables.current_postion},
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
device_err:   
    while(i--) //设备节点创建的回滚操作 device_destroy(demo_class,MKDEV(major, i));   
        class_destroy(demo_class); //删除设备类   
class_err:   
    unregister_chrdev(mem_major, "demo_chrdev");  
fail_malloc:
    printk("fail_malloc\n");
    unregister_chrdev_region(devno, 1);
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
    
    int i;
    //printk("%scheck if mmap func runs fine,mem_devp[0].data=%s\n",TAG,mem_devp[0].data->str);
    printk(KERN_INFO PFX "Stopping...\n");
    cdev_del(&cdev);   /*注销设备*/
    kfree(mem_devp);     /*释放设备结构体内存*/
    unregister_chrdev_region(MKDEV(mem_major, 0), slaves_num); /*释放设备号*/
    /*删除设备节点和设备类*/  
    for(i=0; i<slaves_num; i++)  
      device_destroy(demo_class,MKDEV(mem_major, i));  
    class_destroy(demo_class);  
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
