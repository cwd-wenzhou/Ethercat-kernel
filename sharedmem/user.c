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

 /* 读出数据 */
 printf("buf 1 = %s\n",start); 

 /* 写入数据 */
 strcpy(start,"cwd in house!");

/* 再次读出数据 */
 printf("buf 2 = %s\n",start);

 close(fd1); 
 close(fd2); 
 return 0; 
}
