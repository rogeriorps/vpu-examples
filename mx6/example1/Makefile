CC ?=$(CROSS_COMPILE)gcc
LINK ?=$(CROSS_COMPILE)gcc

OBJ = main.o 

TARGET = example1

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) -o $(TARGET) $(OBJ) ${LIBS} -lvpu -lipu -lrt -lpthread -g

%.o: %.c
	$(CC) $(INC) -Wall -O2 -c $^ -o $@ -g

.PHONY: clean
clean:
	rm -f $(OBJDIR)/$(TARGET) $(OBJ)

include ../make.rules
