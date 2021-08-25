#sudo insmod sharedmem.ko

#cd /dev
ls 
有demo0，demo1


测试程序编译
#gcc user.c -o user
切换到root用户
#su root
#./user
可以看到文件被成功打开。且共享内存的数据可以被读出与写入

#dmesg
[13045.687503] sharemem:   started
[13045.687673] sharemem:  malloc finished;data=cwd is in e308
[13052.400975] sharemem:  file opened,minor=0,data=cwd is in e308
[13052.400982] sharemem:  file opened,minor=1,data=imc
[13052.401100] sharemem:  mmap func run
[13052.401106] sharemem:  mmap func runs fine,data=cwd is in e308
[13152.687327] sharemem:  check if mmap func runs fine,data=cwd in house!
[13152.687328] sharemem:   EXIT
