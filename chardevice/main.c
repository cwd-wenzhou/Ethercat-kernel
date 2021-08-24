#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>

#define DEVNAME "/dev/hello"

int main()
{
    int fd;
    fd = open(DEVNAME, O_RDWR);
    if(fd == -1){
        printf("file %s is opening......failure!", DEVNAME);
        return -1;
    }
    else{
        printf("file %s is opening......successfully!\nits fd is %d\n", DEVNAME, fd);
    }
    getchar();
    close(fd);
    return 0;
}