all : rx tx

rx : rx.o
	gcc -o rx rx.o -lSoapySDR -lm 

rx.o : rx.c
	gcc -c rx.c

tx : tx.o
	gcc -o tx tx.o -lSoapySDR -lm 

tx.o : tx.c
	gcc -c tx.c

clean :
	rm rx tx *.o
