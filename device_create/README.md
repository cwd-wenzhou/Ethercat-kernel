#sudo insmod chardevice.ko 
此时：
#cat /proc/dev 
有Character devices:
236 hello_dev

imc@imc:~$ lsmod
Module                  Size  Used by
chardevice             16384  0

而#cd /dev
ls 
有demo0，demo1
测试程序编译
#gcc main.c -o main
切换到root用户
#su root
#./user
可以看到文件被成功打开。

与chardevice的区别是文件是自动注册的