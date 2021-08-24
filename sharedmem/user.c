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
 int fd;
 char *start;
 //char buf[100];
 char *buf;
 
 /*打开文件*/
 fd = open("/dev/demo0", O_RDWR|O_SYNC);
 if (fd==-1){
    printf("there is no file\n");
    return -1;
 }    
 buf = (char *)malloc(100);
 memset(buf, 0, 100);
 start = mmap(NULL,100,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);

 /* 读出数据 */
 //strcpy(buf,start);
 //sleep (1);
 printf("buf 1 = %s\n",buf); 
 printf("start 1 = %s\n",start); 

//  /* 写入数据 */
//  strcpy(start,"Buf Is Not Null!");
 
//  memset(buf, 0, 100);
//  strcpy(buf,start);
//  sleep (1);
//  printf("buf 2 = %s\n",buf);

      
//  munmap(start,100); /*解除映射*/
 free(buf);
  
 close(fd); 
 return 0; 
}
