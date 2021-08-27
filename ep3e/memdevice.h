#ifndef _MEMDEV_H_
#define _MEMDEV_H_

#ifndef MEMDEV_MAJOR
#define MEMDEV_MAJOR 0   /*预设的mem的主设备号*/
#endif

#ifndef MEMDEV_MAX_DEVS
#define MEMDEV_MAX_DEVS 255    /*设备数*/
#endif

#ifndef MEMDEV_NR_DEVS
#define MEMDEV_NR_DEVS 2    /*设备数*/
#endif

#ifndef MEMDEV_SIZE
#define MEMDEV_SIZE 4096
#endif

#include "ep3e.h"

//迈信伺服电机结构体
struct TEST {
    int opModeSet;         //电机运行模式的设定值,默认位置模式
    int opmode;            //驱动器当前运行模式
    int currentVelocity;  //电机当前运行速度
    int currentPosition;  //电机当前位置
    int targetPosition;   //电机的目标位置
    char str[20];//for test
    
};
/*mem设备描述结构体*/
struct mem_dev                                    
{                                                       
  struct TEST *data;                     
  unsigned long size;      
};

#endif /* _MEMDEV_H_ */