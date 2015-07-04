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
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>

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
 * Signal handler which disables all outputs of the lego
 */
void stop_motors(int);

/**
 * Wrapper around touch_sensor class
 */
class TouchSensor {
public:
	TouchSensor(ev3dev::port_type port)
		try : sensor(port) {}
		catch(std::runtime_error& e) {
			throw std::runtime_error("Cannot open TouchSensor on port " + port + ": " + e.what());
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
};

/**
 * This class represents a stop button - fully automatic program
 * killer.
 */
class StopButton {
public:
	StopButton() : terminate(false) {}
	~StopButton() {
		terminate.store(true);
		if(watch_thread.joinable())
			watch_thread.join();
	}

	/**
	 * Initializes stop button and starts watch thread
	 */
	void init(port_type sensor_port) {
		stop_button.reset(new TouchSensor(sensor_port));
		terminate.store(false);
		watch_thread = std::thread([&] {
			while(!terminate.load()) {
				if(stop_button->isPressed()) {
					std::cout << "Stop button pressed!\n";
					stop_motors(-1);
					std::exit(0); // ToDo: Think about proper way to exit
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
		});
	}
private:
	std::atomic_bool terminate;
	std::unique_ptr<TouchSensor> stop_button;
	std::thread watch_thread;
};

// StopButton instance
extern StopButton g_stop_button;

/**
 * Initializes stop button
 */
void use_stop_button(port_type button_port);

/**
 * Wrapper of the motor class
 */
class Motor {
public:
	Motor(port_type motor_pin) : motor(motor_pin) {	}

	/**
	 * Direct access to underlying ev3dev class
	 */
	ev3dev::motor& backdoor() {
		return motor;
	}

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
	 * moves to the given position
	 */
	void setPositionAbs(int position) {
		motor.set_speed_regulation_enabled("on");
		motor.set_position_sp(position);
		motor.run_to_abs_pos();
	}

	/**
	 * Turns the motor into position regulation and
	 * moves to given relative position
	 */
	void setPositionRel(int position) {
		motor.set_speed_regulation_enabled("off");
		motor.set_position_sp(position);
		motor.run_to_rel_pos();
	}

	/**
	 * Returns position of the encoder
	 */
	int getEncoderPosition() {
		return motor.position();
	}

	/**
	 * Setups the PID constants for position regulation
	 * ToDo: Find out the range for these constants!
	 */
	void setPositionPID(int p, int i, int d) {
		motor.set_position_p(p);
		motor.set_position_i(i);
		motor.set_position_d(d);
	}

	/**
	 * Returns constants for position regulator
	 */
	int getPositionP() {
		return motor.position_p();
	}

	int getPositionI() {
		return motor.position_i();
	}

	int getPositionD() {
		return motor.position_d();
	}

	/**
	 * Ignores all regulation and sets the motor power
	 * directly
	 * @param power power in percents <-100;100>
	 */
	void setPower(int power) {
		if(power < -100 || power > 100)
			throw std::runtime_error("Power out of range: "
					+ std::to_string(abs(power)) + "/100");
		motor.set_speed_regulation_enabled("off");
		motor.set_duty_cycle_sp(power);
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
