#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
//#include <linux/types.h>
#include "ecrt.h"
//#include <math.h>
//#include <stdbool.h>
//#include <stdio.h>
//#include <unistd.h>

MODULE_LICENSE("Dual BSD/GPL");

extern ec_master_t *ecrt_request_master(unsigned int master_index);
extern void ecrt_release_master(ec_master_t *master);
extern int ecrt_master(ec_master_t *master, ec_master_info_t *master_info);
extern void ecrt_release_master(ec_master_t *master);

static int hello_init(void) {
  ec_master_t* master = ecrt_request_master(0);
  ec_master_info_t master_info;
  printk(KERN_ALERT "ECT::ecrt_request_master done\n");
  ecrt_master(master,&master_info);//获取主站信息
  int slave_count = master_info.slave_count;//从站个数
  printk(KERN_ALERT "ECT::there are %d slaves\n",slave_count);
  ecrt_release_master(master);
    
  return 0;
}
static void hello_exit(void) {
  printk(KERN_ALERT "ECT::Bye World!\n");
}

module_init(hello_init);
module_exit(hello_exit);