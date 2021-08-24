
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <sys/types.h>
static int minor = 0;
static dev_t devno;
static struct cdev cdev;
#define HELLO_MAJOR 0;
static int hello_major = HELLO_MAJOR;

module_param(hello_major, int, S_IRUGO);//insmod sharedmem mem_major=5  这样就可从命令行传参数进来。若没有指定则使用上一行的默认值

static int hello_open (struct inode *inode, struct file *filep)
{
	printk("hello_open \n");
	return 0;
}
static struct file_operations hello_ops=
{
	.open = hello_open,			
};
 
static int hello_init(void)
{
	int ret;	
	printk("hello_init");
	devno = MKDEV(hello_major,minor);
  if (hello_major)
	  ret = register_chrdev_region(devno, 1, "hello");
  else{
    ret = alloc_chrdev_region(&devno, 0, 1, "hello");
  }

	if(ret < 0)
	{
		printk("register_chrdev_region fail \n");
		return ret;
	}
	cdev_init(&cdev,&hello_ops);
  cdev.owner = THIS_MODULE;
  cdev.ops = &hello_ops;
	ret = cdev_add(&cdev,devno,1);
	if(ret < 0)
	{
		printk("cdev_add fail \n");
		return ret;
	}	
	return 0;
}
static void hello_exit(void)
{
	cdev_del(&cdev);
	unregister_chrdev_region(devno,1);
	printk("hello_exit \n");
}
MODULE_LICENSE("GPL");
module_init(hello_init);
module_exit(hello_exit);