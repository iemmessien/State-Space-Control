NAME = State
SRC = main.cpp StateSpace.cpp
OBJ = $(SRC:.cpp=.o)

all: $(OBJ)
	g++ $(OBJ) -o $(NAME) -std=c++11 -lsfml-graphics -lsfml-audio -lsfml-window -lsfml-system
	make clean

clean:
	rm -f $(OBJ) $(OUT)
