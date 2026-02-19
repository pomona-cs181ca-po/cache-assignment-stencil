hello-world:
	gcc -O0 -g ./src/hello-world.c -o ./bin/hello-world -static

memtest:
	gcc -O0 -g ./src/memtest.c -o ./bin/memtest -static

all:
	make hello-world memtest

clean:
	rm ./bin/*
