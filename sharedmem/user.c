#include <stdio.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<unistd.h>
#include<sys/mman.h>
#include <stdlib.h>
#include <string.h>
int main()
{
 int fd1,fd2;
 char *start;
 //char buf[100];
 char *buf;
 
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


 buf = (char *)malloc(100);
 memset(buf, 0, 100);
 start = mmap(NULL,100,PROT_READ|PROT_WRITE,MAP_SHARED,fd1,0);

 /* 读出数据 */
 strcpy(buf,start);
 //sleep (1);
 printf("buf 1 = %s\n",buf); 
 printf("start 1 = %s\n",start); 

 /* 写入数据 */
 strcpy(start,"cwd in house!");
 
 memset(buf, 0, 100);
 strcpy(buf,start);
 sleep (1);
 printf("buf 2 = %s\n",buf);

      
//  munmap(start,100); /*解除映射*/
 free(buf);
  
 close(fd1); 
 close(fd2); 
 return 0; 
}
