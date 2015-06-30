#include "lego_lib.h"

int main()
{
	// Init signal handling
	std::signal(SIGABRT, stop_motors);
	std::signal(SIGFPE, stop_motors);
	std::signal(SIGILL, stop_motors);
	std::signal(SIGINT, stop_motors);
	std::signal(SIGSEGV, stop_motors);
	std::signal(SIGTERM, stop_motors);

	// Run "run" function in a safe manner
	int retcode;
	try {
		retcode = run();
	}
	catch(std::exception& e) {
		std::cerr << "Unhandled exception: " << e.what() << std::endl;
		std::cerr << "Terminating" << std::endl;
		retcode = -1;
	}
	catch(...) {
		std::cerr << "Unknown unhandled exception" << std::endl;
		std::cerr << "Terminating" << std::endl;
		retcode = -1;
	}

	stop_motors(-1);
	std::cout << "Program ended" << std::endl;
	return retcode;
}

void stop_motors(int signal)
{
	try {
		ev3dev::motor motor(ev3dev::OUTPUT_A);
		motor.set_command(ev3dev::motor::command_reset);
	}
	catch(...) { }

	try {
		ev3dev::motor motor(ev3dev::OUTPUT_B);
		motor.set_command(ev3dev::motor::command_reset);
	}
	catch(...) { }

	try {
		ev3dev::motor motor(ev3dev::OUTPUT_C);
		motor.set_command(ev3dev::motor::command_reset);
	}
	catch(...) { }

	try {
		ev3dev::motor motor(ev3dev::OUTPUT_D);
		motor.set_command(ev3dev::motor::command_reset);
	}
	catch(...) { }

	if(signal != -1) {
		std::cerr << "Program failed! Signal " << signal << " caught. Terminating" << std::endl;
		exit(1);
	}
}
