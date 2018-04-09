CC = g++ 
CFLAGS = -std=c++11 
INCLUDE = -I .
DEPS = Otto.h
OBJ = Otto.o

all: Otto

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(INCLUDE)

/Otto: $(OBJ) 
	$(CC) -o $@ $^ $(CFLAGS)

clean: 
	rm -rf $(OBJ) ../Otto  
