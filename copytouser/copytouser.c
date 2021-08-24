//#define __KERNEL__
#include <linux/module.h>
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

#define DEVICE_COUNT   2  

static int major = 0;
static int minor = 0;
#define TAG "copytouser"

module_param(major, int, S_IRUGO);//insmod sharedmem mem_major=5  这样就可从命令行传参数进来。若没有指定则使用上一行的默认值

struct mem_dev *mem_devp; /*设备结构体指针*/

struct cdev cdev;

static struct class *demo_class; 
/*文件打开函数*/
int mem_open(struct inode *inode, struct file *filp)
{
    printk("%sfile opened\n",TAG);
    return 0;
}

/*文件释放函数*/
int mem_release(struct inode *inode, struct file *filp)
{
  printk("%sfile released\n",TAG);
  return 0;
}

ssize_t mem_read(struct file *file, char __user *buf,size_t count, loff_t *offset)
{
  char alpha[27];
  int i, cnt;
  memset(alpha, 0, 27);
  for(i = 0; i < 26; i++)
        alpha[i] = 'a' + i;
  if(count > 26)
        cnt = 26;
  else
        cnt = count;
  //使用copy_to_user ()函数从driver读数据到user
  if(!copy_to_user((char *)buf, alpha, cnt))
        return cnt;
  else
    return -1;
}

ssize_t mem_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
  char alpha[27];
  int cnt;
  memset(alpha, 0, 27);
  if(count > 26)
        cnt = 26;
  else
      cnt = count;
  //使用copy_from_user()函数从user写数据到driver
  if(!copy_from_user((char *)alpha, buf, cnt))
  {  
    printk(alpha);
    printk("\n");
    return cnt;
  }  
  else
    return -1;
}



/*文件操作结构体*/
static const struct file_operations mem_fops =
{
  .owner = THIS_MODULE,
  .open = mem_open,
  .release = mem_release,
  .read = mem_read,
  .write = mem_write
};


/*设备驱动模块加载函数*/
static int memdev_init(void)
{
  int result;
  int i;
  dev_t devno = MKDEV(major, minor);
  struct device *demo_device;
  printk("%s started\n",TAG);
  /* 静态申请设备号*/
  if (major)
    result = register_chrdev_region(devno, 2, "memdev");
  else  /* 动态分配设备号 */
  {
    result = alloc_chrdev_region(&devno, 0, 2, "memdev");
    major = MAJOR(devno);
    minor = MINOR(devno);
  } 
 
  if (result < 0){
    printk("chrdev failed\n");
    return result;
  }
    
  printk("%smajor = %d; minor = %d\n", TAG,major, minor);
  /*初始化cdev结构*/
  cdev_init(&cdev, &mem_fops);
  cdev.owner = THIS_MODULE;
  cdev.ops = &mem_fops;
 
  /* 注册字符设备 */
  cdev_add(&cdev, MKDEV(major, 0), 1);

  
 /*创建设备类*/  
  demo_class = class_create(THIS_MODULE,"demo_class");  
  if(IS_ERR(demo_class)){  
          result =  PTR_ERR(demo_class);  
          goto class_err;  
  }  

  /*创建设备文件，通知用户在“/dev/”目录下创件名字为demoX的设备文件*/  
  for(i=0; i<2; i++){ //最多可创建255个设备节点(register_chrdev函数会申请0-254范围的从设备号)  
      demo_device = device_create(demo_class,NULL, MKDEV(major, i), NULL,"demo%d",i);  
      if(IS_ERR(demo_device)){  
            result = PTR_ERR(demo_device);  
            goto device_err;  
      }  
  }      
  return 0;   

  device_err:   
    while(i--) //设备节点创建的回滚操作 device_destroy(demo_class,MKDEV(major, i));   
         class_destroy(demo_class); //删除设备类   
  class_err:   
    unregister_chrdev(major, "demo_chrdev");  

  unregister_chrdev_region(devno, 1);
 
  return result;
}

/*模块卸载函数*/
static void memdev_exit(void)
{
  int i;
  printk("%s EXIT\n",TAG);
  cdev_del(&cdev);   /*注销设备*/
  kfree(mem_devp);     /*释放设备结构体内存*/
  unregister_chrdev_region(MKDEV(major, 0), 2); /*释放设备号*/
  /*删除设备节点和设备类*/  
  for(i=0; i<DEVICE_COUNT; i++)  
      device_destroy(demo_class,MKDEV(major, i));  
  class_destroy(demo_class);  
}


MODULE_AUTHOR("cwd");
MODULE_LICENSE("GPL");

module_init(memdev_init);
module_exit(memdev_exit);
