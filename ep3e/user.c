#include <stdio.h>
//#include <sys/types.h>
//#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <stdbool.h> //for bool type
#include "memdevice.h"

#define slave_num 1
static volatile int keepRunning = 1; 

void sig_handler( int sig )
{
    if ( sig == SIGINT)
    {
        keepRunning = 0;
    }
}

int main()
{
   signal( SIGINT, sig_handler );
   int fd[slave_num];
   struct MOTOR *s[slave_num];
   //char* c = "cwd codeed today";

   char filename[30];
   memset(filename, 0, 27);
   for (int i=0;i<slave_num;i++){
      sprintf(filename,"/dev/etcdevice%d",i);
      fd[i] = open(filename, O_RDWR|O_SYNC);
      if (fd[i] ==-1){
         printf("there is no file %s\n",filename);
         return -1;
      }
      s[i] = mmap(NULL,MEMDEV_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,fd[i],0);
   }

   while (keepRunning){
      sleep(1);
      for (int i=0;i<slave_num;i++){
         // 读出数据 
         printf("-----data from device%d------------\n",i); 
         printf("s1->targetVelocity = %d\n",s[i]->targetVelocity); 
         printf("s1->targetPosition = %d\n",s[i]->targetPosition); 
         printf("s1->currentPosition = %d\n",s[i]->currentPosition); 
         printf("s1->currentVelocity = %d\n",s[i]->currentVelocity); 
      }
   }

   
   for (int i=0;i<slave_num;i++){
      close(fd[i]); 
   }
   return 0; 
}
