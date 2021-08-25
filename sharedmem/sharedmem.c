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
#include "memdev.h"

#define DEVICE_COUNT   2  

static int mem_major = MEMDEV_MAJOR;

module_param(mem_major, int, S_IRUGO);//insmod sharedmem mem_major=5  这样就可从命令行传参数进来。若没有指定则使用上一行的默认值

struct mem_dev *mem_devp; /*设备结构体指针*/

struct cdev cdev;

static char str[16] = "cwd is in e308";

static struct class *demo_class; 

#define TAG "sharemem:  "

/*文件打开函数*/
int mem_open(struct inode *inode, struct file *filp)
{
    struct mem_dev *dev;
    
    /*获取次设备号*/
    int num = MINOR(inode->i_rdev);
    if (num >= MEMDEV_NR_DEVS)
            return -ENODEV;
    dev = &mem_devp[num];
    /*将设备描述结构指针赋值给文件私有数据指针*/
    filp->private_data = dev;
    printk("%sfile opened,minor=%d,data=%s\n",TAG,num,dev->data);
    return 0;
}

/*文件释放函数*/
int mem_release(struct inode *inode, struct file *filp)
{
  return 0;
}
static int memdev_mmap(struct file* filp, struct vm_area_struct *vma)
{
  struct mem_dev *dev = filp->private_data; /*获得设备结构体指针*/
  printk("%smmap func run\n",TAG);
  vma->vm_flags |= VM_IO;
  vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);
  if (remap_pfn_range(vma,vma->vm_start,virt_to_phys(dev->data)>>PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot))
      return  -EAGAIN;
  printk("%smmap func runs fine,data=%s\n",TAG,dev->data);
  return 0;
}

/*文件操作结构体*/
static const struct file_operations mem_fops =
{
  .owner = THIS_MODULE,
  .open = mem_open,
  .release = mem_release,
  .mmap = memdev_mmap,
};


/*设备驱动模块加载函数*/
static int memdev_init(void)
{
  int result;
  int i;
  dev_t devno = MKDEV(mem_major, 0);
  struct device *demo_device;
  printk("%s started\n",TAG);
  /* 静态申请设备号*/
  if (mem_major)
    result = register_chrdev_region(devno, 2, "memdev");
  else  /* 动态分配设备号 */
  {
    result = alloc_chrdev_region(&devno, 0, 2, "memdev");
    mem_major = MAJOR(devno);
  } 
 
  if (result < 0){
    printk("chrdev failed\n");
    return result;
  }
    

  /*初始化cdev结构*/
  cdev_init(&cdev, &mem_fops);
  cdev.owner = THIS_MODULE;
  cdev.ops = &mem_fops;
 
  
  
  /*创建设备类*/  
  demo_class = class_create(THIS_MODULE,"demo_class");  
  if(IS_ERR(demo_class)){  
    result =  PTR_ERR(demo_class);  
    goto class_err;  
  }  

  /*创建设备文件，通知用户在“/dev/”目录下创件名字为demoX的设备文件*/  
  for(i=0; i<2; i++){ //最多可创建255个设备节点(register_chrdev函数会申请0-254范围的从设备号)  
    /* 注册字符设备 */
    cdev_add(&cdev, MKDEV(mem_major, i), MEMDEV_NR_DEVS);
    demo_device = device_create(demo_class,NULL, MKDEV(mem_major, i), NULL,"demo%d",i);  
    if(IS_ERR(demo_device)){  
      result = PTR_ERR(demo_device);  
      goto device_err;  
    }  
  }      

  /* 为设备描述结构分配内存*/
  mem_devp = kmalloc(MEMDEV_NR_DEVS * sizeof(struct mem_dev), GFP_KERNEL);
  
  if (!mem_devp)    /*申请失败*/
  {
    result =  - ENOMEM;
    goto fail_malloc;
  }
  memset(mem_devp, 0, sizeof(struct mem_dev));

  /*为设备分配内存*/
  for (i=0; i < MEMDEV_NR_DEVS; i++)
  {
    mem_devp[i].size = MEMDEV_SIZE;
    mem_devp[i].data = kmalloc(MEMDEV_SIZE, GFP_KERNEL);
    memset(mem_devp[i].data, 0, MEMDEV_SIZE);
  } 
  strcpy(mem_devp[0].data,"cwd is in e308");
  mem_devp[1].data="imc";
  printk("%smalloc finished;data=%s\n",TAG,mem_devp[0].data);

  return 0;
  device_err:   
    while(i--) //设备节点创建的回滚操作 device_destroy(demo_class,MKDEV(major, i));   
         class_destroy(demo_class); //删除设备类   
  class_err:   
    unregister_chrdev(mem_major, "demo_chrdev");  
  fail_malloc:
    printk("fail_malloc\n");
  unregister_chrdev_region(devno, 1);
 
  return result;
}

/*模块卸载函数*/
static void memdev_exit(void)
{
  int i;
  printk("%scheck if mmap func runs fine,data=%s\n",TAG,mem_devp[0].data);
  printk("%s EXIT\n",TAG);
  cdev_del(&cdev);   /*注销设备*/
  kfree(mem_devp);     /*释放设备结构体内存*/
  unregister_chrdev_region(MKDEV(mem_major, 0), 2); /*释放设备号*/
  /*删除设备节点和设备类*/  
  for(i=0; i<DEVICE_COUNT; i++)  
      device_destroy(demo_class,MKDEV(mem_major, i));  
  class_destroy(demo_class);  
}

MODULE_AUTHOR("cwd");
MODULE_LICENSE("GPL");

module_init(memdev_init);
module_exit(memdev_exit);
