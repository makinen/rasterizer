CC=gcc
CXX=g++
RM=rm -f
CPPFLAGS=-ggdb3
LDFLAGS=-g
LDLIBS=-l SDL2 -l SDL2_image

SRCS=src/main.cpp src/math.cpp src/rasterizer.cpp src/sdl_window.cpp src/model.cpp src/texture.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: main 

main: $(OBJS)
	$(CXX) $(LDFLAGS) -o main $(OBJS) $(LDLIBS)

depend: .depend

.depend: $(SRCS)
	$(RM) ./.depend
	$(CXX) $(CPPFLAGS) -MM $^>>./.depend;

clean:
	$(RM) $(OBJS)

distclean: clean
	$(RM) *~ .depend

include .depend
