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
using ev3dev::OUTPUT_A;
using ev3dev::OUTPUT_D;
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
