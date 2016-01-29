#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "lego_lib.h"

static int g_sock_fd = -1;
static int g_client_fd = -1;
#define PORT 12345

int main()
{
	// Init signal handling
	std::signal(SIGABRT, teardown);
	std::signal(SIGFPE, teardown);
	std::signal(SIGILL, teardown);
	std::signal(SIGINT, teardown);
	std::signal(SIGSEGV, teardown);
	std::signal(SIGTERM, teardown);

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

	teardown(-1);
	std::cout << "Program ended" << std::endl;
	return retcode;
}

void teardown(int signal)
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

	if(g_client_fd != -1) {
		close(g_client_fd);
		g_client_fd = -1;
	}

	if(g_sock_fd != -1) {
		close(g_sock_fd);
		g_sock_fd = -1;
	}

	if(signal != -1 && signal != SIGINT) {
		std::cerr << "Program failed! Signal " << signal << " caught. Terminating" << std::endl;
		exit(1);
	}
	if(signal == SIGINT) {
		std::cerr << "Program stopped (probably CTR+C or StopButton)\n";
		exit(1);
	}
}

void use_stop_button(port_type button_port) {
	pid_t pid = fork();
	if (pid == 0) {
		// Child
		pid_t parent_id = getppid();
		try {
			TouchSensor stop_button(button_port);
			while(true) {
				if(stop_button.isPressed() || kill(parent_id, 0) != 0) {
					kill(parent_id, SIGINT);
					exit(0);
				}
				delayMs(20);
			}
		}
		catch(...) {
			exit(1);
		}
	}
	else {
		// Parent
		// Try to open the button to test if it exists!
		try {
			TouchSensor stop_button(button_port);
			stop_button.isPressed();
		}
		catch(...) {
			throw std::runtime_error("Stop button on port " + button_port + " isn't connected!\n");
		}
	}
}

/*
 * Delay in milliseconds
 * @param us milliseconds
 */
void delayMs(int ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

/*
 * Delay in microseconds
 * @param us microseconds
 */
void delayUs(int us) {
	std::this_thread::sleep_for(std::chrono::microseconds(us));
}

int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (!initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

Keys getPressedKeys() {
	/*int waiting = _kbhit();
	Keys keys;
	for(int i = 0; i != waiting; i++)
		keys.insert(std::cin.get());*/

	char buff[32];
	Keys keys;
	ssize_t res;

	if(g_client_fd == -1) {
		return keys;
	}

	while((res = read(g_client_fd, buff, sizeof(buff))) > 0) {
		keys.insert(buff, buff+res);
	}

	if(res < 0 && errno != EAGAIN) {
		std::cerr << "Failed to read from client socket: " << strerror(errno) << std::endl;
	} else if(res == 0 && recv(g_client_fd, NULL, 0, 0) == 0) {
		std::cerr << "Client disconnected!" << std::endl;
		close(g_client_fd);
		g_client_fd = -1;
	}
	return keys;
}

bool isKeyPressed(const Keys& keys, char key) {
	return keys.find(key) != keys.end();
}

int numberOfPressedKeys(const Keys& keys) {
	return keys.size();
}

bool waitForConnection() {
	struct sockaddr_in serv_addr;

	if(g_client_fd != -1) {
		close(g_client_fd);
		g_client_fd = -1;
	}

	if(g_sock_fd != -1) {
		close(g_sock_fd);
		g_sock_fd = -1;
	}

    g_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	if(g_sock_fd < 0)
	{
		std::cerr << "failed to do socket() " << strerror(errno) << std::endl;
	    return false;
	}

	memset((char *)&serv_addr, 0, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(PORT);

	const int optval = 1;
	setsockopt(g_sock_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

	if (bind(g_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		std::cerr << "failed to bind: " << strerror(errno) << std::endl;
	    close(g_sock_fd);
	    g_sock_fd = -1;
	    return false;
	}

	listen(g_sock_fd, 5);
	//fcntl(g_sock_fd, F_SETFL, O_NDELAY); // non-blocking

	while((g_client_fd = accept(g_sock_fd, NULL, NULL)) < 0) {
		std::cerr << "failed to accept client: " << strerror(errno) << std::endl;
	}

	ioctl(g_client_fd, FIONBIO, &optval); // non-blocking
	return true;
}

bool isClientConnected() {
	return g_client_fd != -1;
}

bool sendString(const std::string& str) {
	return sendData(str.c_str(), str.size());
}

bool sendString(const char *str) {
	return sendData(str, strlen(str));
}

bool sendData(const char *buff, size_t len) {
	if(len == 0)
		return true;

	if(g_client_fd == -1) {
		std::cerr << "Failed to send data, not connected";
		return false;
	}

	ssize_t res = 0;
	while(len) {
		res = write(g_client_fd, buff, len);
		if(res < 0) {
			 if(errno == EAGAIN)
				 continue;
			 std::cerr << "Failed to send data: " << strerror(errno);
			 return false;
		}
		len -= res;
		buff += res;
	}
	return true;
}
