#include "lib/lego_lib.h"
#include "lib/atoms/avakar.h"

// More examples: https://github.com/yaqwsx/atoms (https://github.com/yaqwsx/atoms/blob/master/examples/avakar_packet/main.cpp)

int run()
{
	// ==== init ====
	SocketServer server;
	
	std::cout << "server - waiting for connection on port: "  
			  << server.getPort() << std::endl;
		  
	server.open(); 
	
	std::cout << "server - start" << std::endl;
	
	// ==== data ====
	int leftMotorPosition = 1100;
	int rightMotorPosition = 900;
	int leftMotorSpeed = 220;
	int rightMotorSpeed = 180;
	
	// ==== avakar_packet - send ====
	atoms::AvakarPacket out; // Create empty packet
    out.set_command(1);
    out.push<int>(leftMotorPosition); 	// size 4 bytes => sum: 4 bytes
    out.push<int>(rightMotorPosition);	// size 4 bytes => sum: 8 bytes
    out.push<int>(leftMotorSpeed);		// size 4 bytes => sum: 12 bytes
    out.push<int>(rightMotorSpeed);		// size 4 bytes => sum: 16 bytes
	// actual size = 16 bytes; 
	// max size of packet = 16 bytes
		
    server.write(out.raw(), int(out.raw_size()));
    
	// ==== avakar_packet - read ====
	atoms::AvakarPacket in;
	
	int Pconst, Iconst, Dconst; 
	// size = 4 bytes * 3 variables = 12 bytes
	while(true) {
		if(server.getAvailable() != 0) {
			in.push_byte(server.read());
			
			if(in.complete()) {
				if(in.get_command() == 1 && in.size == 12) {
					Pconst = in.get<int>(0);
					Iconst = in.get<int>(4);
					Dconst = in.get<int>(8);
					
					std::cout << "P: " << Pconst
							  << "I: " << Iconst
							  << "D: " << Dconst << std::endl;
							  
					in.clear();
				}
			}
		}
		delayMs(1);
	}
	
	return 0;
}