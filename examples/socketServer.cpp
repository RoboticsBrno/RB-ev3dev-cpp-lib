#include "lib/lego_lib.h"

#include <vector>

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
	
	// new example for server.read(B, 5)
	for(int i = 0; i << 5; ++i)
		std::cout << B[i];
	std::cout << std::endl;
	
	// old example - problem with missing zero character in array B 
	// -> don't forget on that - B = raw data X "hello" = string
	//std::cout << "server.read(B, 5): "<< B 
	//		  << " (num of bytes: " << b << ")" <<std::endl;
			  
	std::cout << C << std::endl;

	std::vector<char> data_for_procesing;
	
	while(true) {
		std::cout << "server - isConnected(): " 
			  << server.isConnected() << std::endl;
	
		if(server.isConnected()) { 
			server.write("hello\n", 6);
			
			//std::cout << server.read() << endl;
			//std::cout << server.read(5) << endl;
			// if you don't use getAvailable()
			// then function read() waiting on data
		
			// generally is better use getAvailable()
			// -> returns number of actually available bytes 
			if(server.getAvailable() != 0) {
				std::cout << "server - read char: " 
						  << char(server.read()) << std::endl;
			}
			
			// the best effective way - use own reading buffer
			// -> avoid slow system call function e.g. recv()
			char buf[50];	//better placement before while()
			int available;	//better placement before while()
			if((available = server.getAvailable()) != 0) {
				if(available > 50)
					available = 50;
				
				server.read(buf, available);
				
				for(int i = 0; i < available; ++i)
					data_for_procesing.push_back(buf[i]);
			}
		}
		
		delayMs(100);
	}
	
	return 0;
}