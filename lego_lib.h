#pragma once

/**
 * @author Jan Mrázek (yaqwsx), Jarek Páral (jarekp)
 * This small piece of code includes the ev3dev library and ensures
 * that if your app fails (uncaught exception or bad pointer
 * dereference), all the motors are stopped.
 */

#include "ev3dev.h"
#include <string>
#include <iostream>
#include <csignal>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <cstring>
#include <termios.h>
#include <stropts.h>
#include <set>
#include <cmath>

/**
 * Use their names
 */
using ev3dev::address_type;
using ev3dev::INPUT_AUTO;
using ev3dev::INPUT_1;
using ev3dev::INPUT_2;
using ev3dev::INPUT_3;
using ev3dev::INPUT_4;
using ev3dev::OUTPUT_AUTO;
using ev3dev::OUTPUT_A;
using ev3dev::OUTPUT_B;
using ev3dev::OUTPUT_C;
using ev3dev::OUTPUT_D;
using ev3dev::button;
using ev3dev::led;

/**
 * Safe replacement of the main function. All exceptions are caught,
 * all signals are processed and the motors are always stopped in the
 * end of program.
 */
int run();

/**
 * Signal handler which correctly tears down everything initiated by this library
 */
void teardown(int);

/**
 * Waits for given time in milliseconds
 */
void delayMs(int ms);

/**
 * Waits for given time in microseconds
 */
void delayUs(int us);

/**
 * Wrapper around button class
 */
class BrickButton {
public:
	BrickButton(ev3dev::button& button) : m_button(button) {};

	/**
	 * Returns reference of this object
	 */
	ev3dev::button& backdoor() {
		return m_button;
	}

	/**
	 * Returns true if the button is pressed
	 */
	bool isPressed() {
		return m_button.pressed() == 1;
	}

	/**
	 * Blocks until button is pressed
	 */
	void waitForPress() {
		while(!isPressed())
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	/**
	 * Blocks until button is read
	 */
	void waitForRelease() {
		while(isPressed())
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	/**
	 * Block until button is clicked
	 */
	void waitForClick() {
		waitForPress();
		waitForRelease();
	}

private:
	ev3dev::button& m_button;
};

/**
 * Wrapper around touch_sensor class
 */
class TouchSensor {
public:
	TouchSensor(ev3dev::address_type port)
		try : sensor(port), m_port(port) {}
		catch(std::runtime_error& e) {
			throw std::runtime_error("Cannot open TouchSensor on port " + port + ": " + e.what());
		}

	/**
	 * Returns port of this object
	 */
	ev3dev::address_type getPort() {
		return m_port;
	}
	/**
	 * Returns true if the sensor is pressed
	 */
	bool isPressed() {
		return sensor.value() == 1;
	}

	/**
	 * Blocks until sensor is pressed
	 */
	void waitForPress() {
		while(!isPressed())
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	/**
	 * Blocks until sensor is read
	 */
	void waitForRelease() {
		while(isPressed())
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}

	/**
	 * Block until sensor is clicked
	 */
	void waitForClick() {
		waitForPress();
		waitForRelease();
	}
private:
		ev3dev::touch_sensor sensor;
		ev3dev::address_type m_port;
};

/**
 * Initializes stop button
 */
void use_stop_button(address_type button_port);

/**
 * Wrapper of the motor class
 */
class Motor {
public:
	Motor(address_type motor_pin) : motor(motor_pin) {
		//this->setInverted(invert);
	}

	/**
	 * Direct access to underlying ev3dev class
	 */
	ev3dev::motor& backdoor() {
		return motor;
	}

//	function setInverted doesn't work on older systems,
//	but it work and was tested on ev3-ev3dev-jessie-2015-12-30
//	/**
//	 * Inverted the motor direction
//	 */
//	void setInverted(bool inv = true)
//	{
//		motor.set_polarity(inv ? "inversed" : "normal");
//	}
//
//	/**
//	 * Is inverted motor direction
//	 */
//	bool isInverted()
//	{
//		return motor.polarity() == "inversed";
//	}

	/**
	 * Resets the motor to the default state
	 */
	void reset() {
		motor.set_command(ev3dev::motor::command_reset);
	}

	/**
	 * Returns number of encoder position per revolution
	 */
	int getCountPerRev() {
		return motor.count_per_rot();
	}

	/**
	 * Turns motor into speed mode and sets its speed
	 * @argument speed speed in range <-2000;2000>
	 * 			 speed is in encoder positions per second
	 */
	void setSpeed(int speed) {
		if(speed < -2000 || speed > 2000)
			throw std::runtime_error("Max speed exceeded; "
					+ std::to_string(abs(speed)) + "of 2000");
		if(motor.speed_regulation_enabled() == "off")
			motor.set_speed_regulation_enabled("on");
		motor.set_speed_sp(speed);
		motor.run_forever();
	}

	/**
	 * Returns the current speed of the motor
	 * @return Speed in encoder positions per second
	 */
	int getSpeed() {
		return motor.speed();
	}

	/**
	 * Setups the PID constants for speed regulation
	 * ToDo: Find out the range for these constants!
	 * @param p proportional constant
	 * @param p integration constant
	 * @param p derivative constant
	 */
	void setSpeedPID(int p, int i, int d) {
		motor.set_speed_regulation_p(p);
		motor.set_speed_regulation_i(i);
		motor.set_speed_regulation_d(d);
	}

	/**
	 * Returns proportional constant for speed regulator
	 */
	int getSpeedP() {
		return motor.speed_regulation_p();
	}

	/**
	 * Returns integration constant for speed regulator
	 */
	int getSpeedI() {
		return motor.speed_regulation_i();
	}

	/**
	 * Returns derivative constant for speed regulator
	 */
	int getSpeedD() {
		return motor.speed_regulation_d();
	}

	/**
	 * Setups start/stop ramp times
	 * @param start time in ms to achieve full speed
	 * @param stop time in ms to break down completely
	 */
	void setSpeedRamps(int start, int stop) {
		motor.set_ramp_up_sp(start);
		motor.set_ramp_down_sp(stop);
	}

	/**
	 * Changes value of the encoder position to given value
	 */
	void setEncoderPosition(int position) {
		motor.set_position(position);
	}

	/**
	 * Turns the motor into position regulation and
	 * moves to the given position at given speed
	 * @param position position is in encoder position
	 * @param speed speed is in encoder tics per second
	 */
	void setPositionAbs(int position, int speed) {
		if(motor.speed_regulation_enabled() == "off")
			motor.set_speed_regulation_enabled("on");
		motor.set_speed_sp(speed);
		motor.set_position_sp(position);
		motor.run_to_abs_pos();
	}

	/**
	 * Turns the motor into position regulation and
	 * moves to given relative position.
	 * @param position position is in encoder position
	 * @param speed speed is in encoder tics per second
	 */
	void setPositionRel(int position, int speed) {
		if(motor.speed_regulation_enabled() == "off")
			motor.set_speed_regulation_enabled("on");
		motor.set_speed_sp(speed);
		motor.set_position_sp(position);
		motor.run_to_rel_pos();
	}

	/**
	 * Returns true if the desired position was reached
	 */
	bool isPositionReached() {
		const auto states = motor.state();
		return states.find("running") == states.end();
	}

	/**
	 * Waits until the motor reaches given position
	 */
	void waitForPosition() {
		while(!isPositionReached())
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	/**
	 * Returns position of the encoder
	 */
	int getEncoderPosition() {
		return motor.position();
	}

	// Do not use this function, ev3dev doesn't implement them!
	// And their position PID sucks!
	/**
	 * Setups the PID constants for position regulation
	 * ToDo: Find out the range for these constants!
	 */
	/* void setPositionPID(int p, int i, int d) {
		motor.set_position_p(p);
		motor.set_position_i(i);
		motor.set_position_d(d);
	}*/

	/**
	 * Returns constants for position regulator
	 */
	/* int getPositionP() {
		return motor.position_p();
	}

	int getPositionI() {
		return motor.position_i();
	}

	int getPositionD() {
		return motor.position_d();
	}*/

	/**
	 * Ignores all regulation and sets the motor power
	 * directly
	 * @param power power in percents <-100;100>
	 */
	void setPower(int power) {
		if(power < -100 || power > 100)
			throw std::runtime_error("Power out of range: "
					+ std::to_string(abs(power)) + "/100");
		if(motor.speed_regulation_enabled() == "on")
			motor.set_speed_regulation_enabled("off");
		motor.set_duty_cycle_sp(power);
		motor.run_forever();
	}

	/**
	 * Returns current power to motor in percents
	 */
	int getPower() {
		return motor.duty_cycle();
	}
private:
	ev3dev::motor motor;
};

/**
 * Wrapper for color sensor
 */
class ColorSensor {
public:
	struct Color_t { int r; int g; int b;};
	enum class color { none, black, blue, green, yellow, red, white, brown};
	enum class mode { col_reflect, col_ambient, col_color, ref_raw, rgb_raw};

	ColorSensor(address_type sensor_port, mode sensor_mode = mode::col_reflect)
		: sensor(sensor_port) {
			this->setMode(sensor_mode);
			m_sensor_mode = sensor_mode;
		}

	ev3dev::color_sensor& backdoor() {
		return sensor;
	}

	/**
	 * Returns value of sensor without set mode (quicker)
	 */
	int getValue() {
		return sensor.value();
	}

	/**
	 * Sets sensor mode
	 */
	void setMode(mode sensor_mode = mode::col_reflect) {
		switch(sensor_mode) {
			case mode::col_reflect:
				sensor.set_mode(ev3dev::color_sensor::mode_col_reflect);
				break;
			case mode::col_ambient:
				sensor.set_mode(ev3dev::color_sensor::mode_col_ambient);
				break;
			case mode::col_color:
				sensor.set_mode(ev3dev::color_sensor::mode_col_color);
				break;
			case mode::ref_raw:
				sensor.set_mode(ev3dev::color_sensor::mode_ref_raw);
				break;
			case mode::rgb_raw:
				sensor.set_mode(ev3dev::color_sensor::mode_rgb_raw);
				break;
		}
		m_sensor_mode = sensor_mode;
	}

	/**
	 * Returns sensor mode
	 */
	mode getMode() {
		return m_sensor_mode;
	}

	/**
	 * Reflected light. Red LED on.
	 * Returns reflected intensity
	 */
	int getReflected() {
		sensor.set_mode(ev3dev::color_sensor::mode_col_reflect);
		return sensor.value();
	}

	/**
	 * Raw reflected. Red LED on
	 * Returns reflected intensity in raw values
	 */
	int getReflectedRaw() {
		sensor.set_mode(ev3dev::color_sensor::mode_ref_raw);
		return sensor.value();
	}

	/**
	 * Raw Color Components. All LEDs rapidly cycling, appears white.
	 * Returns reflected intensity in raw values
	 * @return struct Color_t {int r; int g; int b;};
	 */
	Color_t getReflectedRgbRaw() {
		Color_t color;
		sensor.set_mode(ev3dev::color_sensor::mode_rgb_raw);
		color.r = sensor.value(0);
		color.g = sensor.value(1);
		color.b = sensor.value(2);
		return color;
	}

	/**
	 * Ambient light. Red LEDs off.
	 * Returns ambient color
	 */
	int getAmbient() {
		sensor.set_mode(ev3dev::color_sensor::mode_col_ambient);
		return sensor.value();
	}

	/**
	 * Color. All LEDs rapidly cycling, appears white.
	 * Returns sensed color
	 * ToDo: Translate numbers into color constants
	 */
	int getColor() {
		sensor.set_mode(ev3dev::color_sensor::mode_col_color);
		return sensor.value();
	}
private:
	ev3dev::color_sensor sensor;
	mode m_sensor_mode;
};

/**
 * Wrapper for ultrasonic sensor
 */
class UltrasonicSensor {
public:
	UltrasonicSensor(address_type sensor_port) : sensor(sensor_port) {
		startSignal();
	}

	ev3dev::ultrasonic_sensor& backdoor() {
		return sensor;
	}

	/**
	 * Starts sending ultrasonic signal. Signal can be stopped
	 * by stopSignal. Signal can be heard by another sensor
	 */
	void startSignal() {
		sensor.set_mode(ev3dev::ultrasonic_sensor::mode_us_dist_cm);
	}

	/**
	 * Stop sending ultrasonic signal. Signal can be enabled by
	 * startSignal
	 */
	void stopSignal() {
		sensor.set_mode(ev3dev::ultrasonic_sensor::mode_us_listen);
	}

	/**
	 * Returns distance in cm. It has to send signal - be aware of it
	 */
	double getDistance() {
		const auto mode = sensor.mode();
		if(mode != ev3dev::ultrasonic_sensor::mode_us_dist_cm)
			sensor.set_mode(ev3dev::ultrasonic_sensor::mode_us_si_cm);
		double value = (double)sensor.value() / 10;
		sensor.set_mode(mode);
		return value;
	}

	/**
	 * Returns true, if the sensor hears a signal. Changes the sensor
	 * mode!
	 * ToDo: Change this silly name...
	 */
	bool doIHearAnything() {
		sensor.set_mode(ev3dev::ultrasonic_sensor::mode_us_listen);
		return sensor.value() == 1;
	}

private:
	ev3dev::ultrasonic_sensor sensor;
};

/**
 * Wrapper for gyroscopic sensor
 */
class GyroSensor {
public:
	GyroSensor(address_type sensor_port) : sensor(sensor_port), off(0) {
		sensor.set_mode(ev3dev::gyro_sensor::mode_gyro_g_a);
	}

	ev3dev::gyro_sensor& backdoor() {
		return sensor;
	}

	/**
	 * Returns rotation of the sensor
	 */
	int getAngle() {
		return sensor.value(0) - off;
	}

	/**
	 * Returns angle speed of the sensor
	 */
	int getSpeed() {
		return sensor.value(1);
	}

	/**
	 * Resets the sensor - set it to zero
	 */
	void reset() {
		off = sensor.value(0);
	}

private:
	ev3dev::gyro_sensor sensor;
	int off;
};

/**
 * Basic stop watch class
 */
class StopWatch {
public:
	/**
	 * Initializes stop watch and if true is passed, starts the watch
	 */
	StopWatch(bool start = true) {
		reset(start);
	}

	void start() {
		is_running = true;
		start_time = std::chrono::high_resolution_clock::now();
	}

	void stop() {
		is_running = false;
		stop_time = std::chrono::high_resolution_clock::now();
	}

	bool isRunning() {
		return is_running;
	}

	void reset(bool start = true) {
		is_running = start;
		start_time = std::chrono::high_resolution_clock::now();
		stop_time = std::chrono::high_resolution_clock::now();
	}

	int getMs() {
		if(is_running)
			stop_time = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count();
	}

	int getUs() {
		if(is_running)
			stop_time = std::chrono::high_resolution_clock::now();
		return std::chrono::duration_cast<std::chrono::microseconds>(stop_time - start_time).count();
	}

private:
	typedef typename std::chrono::high_resolution_clock::time_point time_point;
	bool is_running;
	time_point start_time;
	time_point stop_time;
};

/**
 * Class for create server socket and sending debug information
 */
class SocketServer {
public:
	/**
	 * Server type could be BLOCKING or NONBLOCKING
	 * BLOCKING:
	 *  - works in same thread as main program
	 *  - stop main program in function open() till somebody connect on socket
	 * NONBLOCKING:
	 *  - works in own thread
	 *  - waiting in own thread till somebody connect on socket (don't stop main program)
	 * BOTH (BLOCKING|NONBLOCKING):
	 *  - no connection on socket: write() => nothing (seems that data were sent)
	 *  - no connection on socket: read() => waiting on data
	 */
	enum class ServerType { BLOCKING, NONBLOCKING};

	SocketServer() : m_server_type(ServerType::BLOCKING), m_server_port(12345), m_sock_fd(-1), m_client_fd(-1),
		m_terminate_flag(false) {}

	~SocketServer() {
		m_terminate_flag = true;
		if (m_sock_fd != -1) {
			close(m_sock_fd);
			m_sock_fd = -1;
		}
		closeClientSocket();
		if (m_connect_thread.joinable())
			m_connect_thread.join();
	}

	/**
	 * Open the socket
	 * @param server_port number of port (default: 12345)
	 * @param type type of socket (ServerType::BLOCKING|ServerType::NONBLOCKING)
	 * @return true if successfully open else false
	 */
	bool open(int server_port = 12345, ServerType type = ServerType::BLOCKING) {
		struct sockaddr_in serv_addr;
		m_server_type = type;

		if(m_client_fd != -1) {
			close(m_client_fd);
			m_client_fd = -1;
		}

		if(m_sock_fd != -1) {
			close(m_sock_fd);
			m_sock_fd = -1;
		}

		m_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
		if(m_sock_fd < 0) {
			std::cerr << "SocketServer::open: Failed to do socket() - " << strerror(errno) << std::endl;
			return false;
		}

		m_server_port = server_port;
		memset((char *)&serv_addr, 0, sizeof(serv_addr));
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = INADDR_ANY;
		serv_addr.sin_port = htons(server_port);

		const int optval = 1;
		setsockopt(m_sock_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);
		// immediate release socket after end of program

		if (bind(m_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
			std::cerr << "SocketServer::open: Failed to bind() - " << strerror(errno) << std::endl;
			close(m_sock_fd);
			m_sock_fd = -1;
			return false;
		}

		listen(m_sock_fd, 1); // max 1 connection

		if (m_server_type == ServerType::BLOCKING) {
			waitForClient();
		}
		else {
			m_connect_thread = std::thread([&]() {
				while (!m_terminate_flag)
					waitForClient();
			});
		}

		return true;
	}

	/**
	 * Stop the program while somebody connect on socket
	 */
	bool waitForClient() {
		m_client_fd = accept(m_sock_fd, NULL, NULL);
		return m_client_fd != -1;
	}

	/**
	 * Returns true or false if is somebody connected on socket
	 */
	bool isConnected() {
		return m_client_fd != -1;
	}

	/**
	 * Get number of byte in tx socket buffer
	 */
	int getAvailable() {
		if(m_client_fd != -1) {
			int available_bytes = 0;
			if(ioctl(m_client_fd, FIONREAD, &available_bytes) == -1)
				std::cerr << "ServerSocket::getAvailable: error ioctl() - " << strerror(errno);
			return available_bytes;
		}
		return 0;
	}

	/**
	 * Returns true if tx buffer will be free
	 */
	bool empty() {
		return getAvailable() == 0;
	}

	/**
	 * Send one char to socket
	 * @param value char to send
	 * @return number of send chars or error code
	 */
	int write(char value) {
		if(m_client_fd != -1) {
			int ret = send(m_client_fd, &value, 1, MSG_NOSIGNAL);
			if(ret == -1)
				closeClientSocket();
			return ret;
		}
		return 0;
	}

	/**
	 * Send data from buffer to socket
	 * @param buf pointer to const buffer with data
	 * @param len size of buf (buffer)
	 * @return number of send chars or error code
	 */
	int write(const char * buf, int len) {
		if(m_client_fd != -1) {
			int ret = send(m_client_fd, buf, len, MSG_NOSIGNAL);
			if(ret == -1)
				closeClientSocket();
			return ret;
		}
		return 0;
	}

	/**
	 * Send string to socket
	 * @param data reference to string with data
	 * @return number of send chars or error code
	 */
	int write(std::string & data) {
		if(m_client_fd != -1) {
			int ret = send(m_client_fd, data.c_str(), sizeof(char)*data.size(), MSG_NOSIGNAL);
			if(ret == -1)
				closeClientSocket();
			return ret;
		}
		return 0;
	}

	/**
	 * Read char from socket
	 */
	int read() {
		char value;
		if(m_client_fd != -1) {
			while(empty())
				delayUs(1);
			recv(m_client_fd, &value, 1, MSG_NOSIGNAL);
			return value;
		}
		return -1;
	}

	/**
	 * Read data to buffer (char buf [len]) from socket
	 * @param buf pointer to array of chars
	 * @param len required number of characters/size of array
	 * @return number of read bytes
	 */
	int read(char * buf, int len) {
		if(m_client_fd != -1) {
			while(getAvailable() != len)
				delayUs(1);
			 int ret = recv(m_client_fd, buf, len, MSG_NOSIGNAL);
			 return ret;
		}
		return 0; // zero characters reads
	}

	/**
	 * Read data to string from socket
	 * @param len required number of characters/size of array
	 * @return string with data
	 */
	std::string read(int len) {
		if(m_client_fd != -1) {
			std::string value(len, ' ');
			while(getAvailable() != len)
				delayUs(1);
			recv(m_client_fd, &value[0], len, MSG_NOSIGNAL);
			return value;
		}
		return std::string();
	}

	void closeClientSocket() {
		if (m_client_fd != -1) {
			close(m_client_fd);
			m_client_fd = -1;
		}
	}

	int getPort() {
		return m_server_port;
	}

	int getClientFd() {
		return m_client_fd;
	}

	int getServerFd() {
		return m_sock_fd;
	}

	ServerType getServerType() {
		return m_server_type;
	}

private:
	ServerType m_server_type;
	int m_server_port;
	int m_sock_fd;
	int m_client_fd;
	std::atomic<bool> m_terminate_flag;
	std::thread m_connect_thread;
};


typedef std::set<char> Keys;

/**
 * Returns set of pressed keys
 */
Keys getPressedKeys();

/**
 * Returns true if the given key is pressed
 */
bool isKeyPressed(const Keys& keys, char key);

/**
 * Returns number of pressed keys
 */
int numberOfPressedKeys(const Keys& keys);

/**
 * Waits for connection on TCP socket
 */
bool waitForConnection();

/**
 * Returns false if client has disconnected and you should call waitForConnection again
 */
bool isClientConnected();

/**
 * Sends string over the socket connection
 */
bool sendString(const std::string& str);
bool sendString(const char *str);
bool sendData(const char *buff, size_t len);
