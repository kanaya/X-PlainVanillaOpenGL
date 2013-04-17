%.o: %.c
	cc -c $<

all: quatsample

quatsample: quatsample.o quat.o
	cc -o $@ $^ -framework OpenGL -framework GLUT
