#include "lib/lego_lib.h"

int run()
{
	// ==== init ====
	SocketServer server;
	
	std::cout << "server - waiting for connection on port: "  
			  << server.getPort() << std::endl;
			  
	server.open(); 
	// same as this => default parameters
	// server.open(12345, server.ServerType::BLOCKING);
	
	// open() as independent thread 
	// server.open(12345, server.ServerType::NONBLOCKING);
	
	std::cout << "server - start" << std::endl;
	
	// ==== send ====
	int a,b,c;
	a = b = c = -1;
	std::string out(" bye\n");

	a = server.write('=');
	b = server.write("good",4);
	c = server.write(out);
	
	std::cout << "server - send bytes write('='): " 
			  << a << std::endl;	
	std::cout << "server - send bytes write(\"good\",4): "  
			  << b << std::endl;
	std::cout << "server - send bytes write(out): "
			  << c << std::endl;
	
	// ==== read ====
	char A;
	char B [5];
	std::string C;

	A =	server.read();
	b = server.read(B, 5);
	C = server.read(5);

	std::cout << "server.read(): " << A << std::endl;
	std::cout << "server.read(B, 5): "<< B 
			  << " (num of bytes: " << b << ")" <<std::endl;
	std::cout << C << std::endl;

	char ch;
	while(true) {
		if(server.getAvailable() != 0) {
			std::cout << "server - read char: " 
					  << char(server.read()) << std::endl;
		}

		std::cout << "server - isConnected(): " 
				  << server.isConnected() << std::endl;
		
		server.write("hello\n", 6);
		delayMs(500);
	}
	
	return 0;
}