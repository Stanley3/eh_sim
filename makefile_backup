#main: utils.o main.o
#	g++ utils.o main.cpp -o main
#
#main.o:
#	g++ -c main.cpp
#
#utils.o: utils/utils.h utils/utils.cpp
#	g++ -c utils/utils.cpp
#
#clean:
#	rm -f *.o

OBJECTS=utils.o visual_template_match.o main.o gc_multi.o
CC=g++

main: $(OBJECTS)
	$(CC) $(OBJECTS) -o main `pkg-config opencv --cflags --libs`

utils.o: utils/utils.h
	$(CC) -std=gnu++11 -c utils/utils.cpp

visual_template_match.o: visual_template_match.h

gc_multi.o: gc_multi.h

clean:
	rm -f $(OBJECTS)
