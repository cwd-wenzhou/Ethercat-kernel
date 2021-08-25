编译模块
#make
#sudo insmod sharedmem.ko

#cd /dev
ls 
有demo0，demo1

测试程序编译
#gcc user.c -o user
切换到root用户
#su root
#./user
root@imc:/home/imc/Documents/code/kernel-ethercat/sharedmem# ./user 
buf 1 = cwd is in e308
buf 2 = cwd in house!
可以看到文件被成功打开。且共享内存的数据可以被读出与写入

#sudo rmmod sharedmem.ko
#dmesg
[13877.974141] sharemem:  started
[13877.974212] sharemem:  malloc finished;
[13883.140974] sharemem:  file opened,minor=0,data=cwd is in e308
[13883.140983] sharemem:  file opened,minor=1,data=cwd is in c412
[13883.140988] sharemem:  mmap func run
[13883.140999] sharemem:  mmap func runs fine,data=cwd is in e308
[13888.968110] sharemem:  check if mmap func runs fine,mem_devp[0].data=cwd in house!
[13888.968111] sharemem:  EXIT

