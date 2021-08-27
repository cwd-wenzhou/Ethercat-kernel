#include <stdio.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<sys/mman.h>
#include <stdlib.h>
#include <string.h>
#include "memdev.h"
int main()
{
   int fd1,fd2;
   struct MOTOR *start;
   char* c = "cwd codeed today";
   char alpha[30];
   memset(alpha, 0, 27);
   /*打开文件*/
   fd1 = open("/dev/demo0", O_RDWR|O_SYNC);
   if (fd1==-1){
      printf("there is no file demo0\n");
      return -1;
   }   

   fd2 = open("/dev/demo1", O_RDWR|O_SYNC);
   if (fd2==-1){
      printf("there is no file demo1\n");
      return -1;
   }  
  
   start = mmap(NULL,100,PROT_READ|PROT_WRITE,MAP_SHARED,fd1,0);

   
   write(fd1, c, 17);  
   read(fd1, alpha, 26);
   printf("%s\n", alpha);
   /* 读出数据 */
   printf("start->str = %s\n",start->str); 
   printf("start->targetPosition = %d\n",start->targetPosition); 

   /* 写入数据 */
   strcpy(start->str,"cwd in house!");
   start->targetPosition = 1314;

   /* 再次读出数据 */
   printf("start->str = %s\n",start->str); 
   printf("start->targetPosition = %d\n",start->targetPosition);

   close(fd1); 
   close(fd2); 
   return 0; 
}
