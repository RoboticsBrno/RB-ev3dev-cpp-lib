#pragma once

/**
 * @author Jan Mrázek
 * This small piece of code includes the ev3 library and ensures
 * that if your app fails (uncaught exception or bad pointer
 * dereference), all the motors are stopped.
 */

#include "ev3dev.h"
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
#include <termios.h>
#include <stropts.h>
#include <set>
#include <cmath>

/**
 * Use their names
 */
using ev3dev::port_type;
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
 * Waits for given time
 */
void delayMs(int ms);

/**
 * Wrapper around touch_sensor class
 */
class TouchSensor {
public:
	TouchSensor(ev3dev::port_type port)
		try : sensor(port), m_port(port) {}
		catch(std::runtime_error& e) {
			throw std::runtime_error("Cannot open TouchSensor on port " + port + ": " + e.what());
		}

	/**
	 * Returns port of this object
	 */
	ev3dev::port_type getPort() {
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
		ev3dev::port_type m_port;
};

/**
 * Initializes stop button
 */
void use_stop_button(port_type button_port);

/**
 * Wrapper of the motor class
 */
class Motor {
public:
	Motor(port_type motor_pin) : motor(motor_pin) {
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
	 * Speed is in encoder positions per second
	 */
	int getSpeed() {
		return motor.speed_sp();
	}

	/**
	 * Setups the PID constants for speed regulation
	 * ToDo: Find out the range for these constants!
	 */
	void setSpeedPID(int p, int i, int d) {
		motor.set_speed_regulation_p(p);
		motor.set_speed_regulation_i(i);
		motor.set_speed_regulation_d(d);
	}

	/**
	 * Returns constants for speed regulator
	 */
	int getSpeedP() {
		return motor.speed_regulation_p();
	}

	int getSpeedI() {
		return motor.speed_regulation_i();
	}

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
	 * Position is in encoder position, speed is in encoder
	 * tics per second
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
	 * moves to given relative position. See setPositionAbs
	 * for details
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

	ColorSensor(port_type sensor_port, mode sensor_mode = mode::col_reflect)
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
	UltrasonicSensor(port_type sensor_port) : sensor(sensor_port) {
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
	GyroSensor(port_type sensor_port) : sensor(sensor_port), off(0) {
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
