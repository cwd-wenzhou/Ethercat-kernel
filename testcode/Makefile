CXX = g++
CFLAGS = -O2 -Wall -g 
LIBS = /opt/etherlab/lib/libethercat.a -lrt -lpthread -lm
CLEANFILES = core core.* *.core *.o temp.* *.out typescript* \
		*.lc *.lh *.bsdi *.sparc *.uw

PROGS = nanosleep   usleep cpu_set load50
OBJS =  

all: $(PROGS)

nanosleep:nanosleep.o
	$(CXX) $(CFLAGS) $(OBJS) -o $@ nanosleep.o ${LIBS}
usleep:usleep.o
	$(CXX) $(CFLAGS) $(OBJS) -o $@ usleep.o ${LIBS}
cpu_set:cpu_set.o
	$(CXX) $(CFLAGS) $(OBJS) -o $@ cpu_set.o ${LIBS}

clean:
	rm -rf /$(PROGS) $(CLEANFILES)
