#include <stdio.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<sys/mman.h>
#include <stdlib.h>
#include <string.h>
#include "memdevice.h"
#define slave_num 4
int main()
{
   int fd[slave_num];
   struct TEST *s;
   //char* c = "cwd codeed today";
   char alpha[30];
   memset(alpha, 0, 27);
   /*打开文件*/

   fd[0]=open("/dev/etcdevice0", O_RDWR|O_SYNC);
   fd[1]=open("/dev/etcdevice1", O_RDWR|O_SYNC);
   fd[2]=open("/dev/etcdevice2", O_RDWR|O_SYNC);
   fd[3]=open("/dev/etcdevice3", O_RDWR|O_SYNC);

   for (int i=0;i<slave_num;i++){
      if (fd[i]==-1){
         printf("there is no file etc_device0%d\n",i);
         return -1;
      }
   }   
  

   for (int i=0;i<slave_num;i++){
      s = mmap(NULL,MEMDEV_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,fd[i],0);
      /* 读出数据 */
      printf("s1->str = %s\n",s->str); 
      printf("s1->targetPosition = %d\n",s->targetPosition); 
   }
   
   for (int i=0;i<slave_num;i++){
      close(fd[i]); 
   }
   
   return 0; 
}
