all : rx tx tx_rx

rx : rx.o
	gcc -o rx rx.o -lSoapySDR -lm

rx.o : rx.c
	gcc -c rx.c

tx : tx.o
	gcc -o tx tx.o -lSoapySDR -lm

tx.o : tx.c
	gcc -c tx.c

tx_rx : tx_rx.o
	gcc -o tx_rx tx_rx.o -lSoapySDR -lm

tx_rx.o : tx_rx.c
	gcc -c tx_rx.c

clean :
	rm rx tx tx_rx *.o
