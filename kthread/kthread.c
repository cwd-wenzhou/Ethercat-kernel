#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/err.h>
#define PFX "kthread: "
#define CPU_NUM 2
struct  timeval timer1,timer2;
static struct task_struct *k;
static struct timespec64 req;
static ktime_t kt,cyctime;
int running;
int cyclic_task(void *arg){
    int res;
    while(running){
        do_gettimeofday(&timer1);
        set_current_state(TASK_UNINTERRUPTIBLE);
        kt = ktime_add(kt,cyctime);
        res = schedule_hrtimeout_range(&cyctime,0,HRTIMER_MODE_ABS_PINNED_HARD);
        printk(KERN_INFO PFX"interval: %ld - %ld = %ld us\r\n",timer1.tv_usec,timer2.tv_usec,timer1.tv_usec-timer2.tv_usec);
        printk(KERN_INFO PFX"res=%d\n",res);
        timer2=timer1;
        //udelay(200);

    }
    return 0;	
}

static int hrtime_init(void)
{
    req.tv_nsec = 1000000;
    req.tv_sec =1;
    //hrtimer_nanosleep(&req,HRTIMER_MODE_ABS,CLOCK_REALTIME);
    do_gettimeofday(&timer1);
    cyctime = ktime_set(0,1000000);
    kt = timeval_to_ktime(timer1);
    printk(KERN_INFO PFX "start!\n");
    running =1;
    k = kthread_create(cyclic_task, NULL, "cwd_thread");
    //k = kthread_create_on_node(cyclic_task, NULL, cpu_to_node(CPU_NUM), "cwd_thread");
    if (IS_ERR(k))
		return -1;
    kthread_bind(k, CPU_NUM);
    printk(KERN_INFO PFX "kthread_create!\n"); 
    k->state = 
    wake_up_process(k);
    printk(KERN_INFO PFX "create ktrhead ok! pid=%d\n",k->pid);  
    
	return 0;
}
static void hrtime_exit(void)
{
    running = 0;
    printk(KERN_INFO PFX "create ktrhead ok! pid=%d now closing\n",k->pid);
    if (!IS_ERR(k)){  
        int ret = kthread_stop(k);  
        printk(KERN_INFO PFX "thread function has run %ds\n", ret);  
    }
	
}
MODULE_LICENSE("GPL");
module_init(hrtime_init);
module_exit(hrtime_exit);