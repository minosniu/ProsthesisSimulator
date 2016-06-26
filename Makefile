UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S), Linux)
	OTHER_FLAGS = -lGL -lglut
else
	OTHER_FLAGS = -framework OpenGL -framework GLUT
endif
INCLUDE_DIR = -I/usr/include/GL/ -I. 
CCX = clang++
SOURCE_FILES = Vector.cpp cargadorTGA.cpp 3ds.cpp Main.cpp Materiales.cpp

all:
	$(CXX) $(SOURCE_FILES) $(INCLUDE_DIR) $(OTHER_FLAGS) -o demo
