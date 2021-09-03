#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/hrtimer.h>
#include <linux/timer.h>

#define CLOCK_TYPE CLOCK_REALTIME
#define NSECS 1000000
struct hrtimer hrtimer_timer;
ktime_t cyctime_kt;
ktime_t nowtime_kt;

static enum hrtimer_restart cyclic_task(struct hrtimer *timer){
	printk("interval1: %lld - %lld = %lld us\r\n",timer->base->get_time()/1000,nowtime_kt/1000,(timer->base->get_time()-nowtime_kt)/1000);
    nowtime_kt =timer->base->get_time();
    hrtimer_forward(timer,timer->base->get_time(), cyctime_kt);
	return HRTIMER_RESTART;
}


static int hrtime_init(void)
{
    printk("hrtimer_begin \n");
    cyctime_kt  = ktime_set(0,NSECS);
    hrtimer_init(&hrtimer_timer,CLOCK_TYPE,HRTIMER_MODE_ABS_PINNED);
    hrtimer_timer.function = cyclic_task;
    hrtimer_start(&hrtimer_timer,cyctime_kt,HRTIMER_MODE_ABS_PINNED);
	return 0;
}
static void hrtime_exit(void)
{
	hrtimer_cancel(&hrtimer_timer);
	printk("hrtimer_exit \n");
}
MODULE_LICENSE("GPL");
module_init(hrtime_init);
module_exit(hrtime_exit);