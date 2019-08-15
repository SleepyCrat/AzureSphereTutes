/*
This application uses the Azure Sphere MT3620 Starter Kit Board.
It utilizes a ULN2003 motor driver to drive a 28BYJ-48 stepper motor.
The driver's input pins (IN1,IN2,IN3,IN4) are mapped to pins 31,32,33,34 of the MT3620.
The driver is connected to a sperate 5-12V power source with the ground of that source split to one of
the MT3620's ground pins and to the ground pin of the driver the positve of the power source 
is directly connected to the driver only. The motor itself is connected to the driver.
*/
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <time.h>


#include <applibs/log.h>
#include <applibs/gpio.h>

#include "epoll_timerfd_utilities.h"
#include "signal.h"
#include "avnet_mt3620_sk.h";


static int buttonPollTimerFd = -1;
static int epollFd = -1;
static int button_A_GpioFd = -1;
static int greenLEDFd = -1;
static int stepperMotorTimerFd = -1;


// Termination state
static volatile sig_atomic_t terminationRequired = false;

static GPIO_Value_Type buttonState = GPIO_Value_High;
static bool isMotorTurning = false;
static int stepNumber = 0;

static int IN4 = -1;
static int IN3 = -1;
static int IN2 = -1;
static int IN1 = -1;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
	// Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
	terminationRequired = true;
}

/// <summary>
///     Handle button timer event: if the button is pressed, light the green LED and set isMotorTurning to true.
/// </summary>
static void ButtonTimerEventHandler(EventData *eventData)
{
	if (ConsumeTimerFdEvent(buttonPollTimerFd) != 0)
	{
		terminationRequired = true;
		return;
	}

	// Check for a button press
	GPIO_Value_Type newButtonState;
	int result = GPIO_GetValue(button_A_GpioFd, &newButtonState);
	if (result != 0)
	{
		Log_Debug("ERROR: Could not read button GPIO: %s (%d).\n", strerror(errno), errno);
		terminationRequired = true;
		return;
	}

	if (newButtonState == GPIO_Value_High)
	{
		isMotorTurning = false;
		stepNumber = 0;
		GPIO_SetValue(greenLEDFd, GPIO_Value_High);
	}
	else
	{
		isMotorTurning = true;
		GPIO_SetValue(greenLEDFd, GPIO_Value_Low);
	}
}

/*
Stepper mototr event will fire every 0.002048 seconds as defined for a full step for the 28byj-48 stepper motor.
if the isMotorTurning bool is true from the button being pressed we will check to see which step we are on.
We send the signal to the correct GPIO while stopping the signal from the others to tell the motor driver which coil to ignite. 
This moves the motor to the next step.
When we reach our fourth step(index 3) we set it back to the first step(index 0) and start the cycle back again.
*/
static void StepperMotorEventHandler(EventData *eventData)
{
	if (ConsumeTimerFdEvent(stepperMotorTimerFd) != 0)
	{
		terminationRequired = true;
		return;
	}

	if (isMotorTurning)
	{
		switch (stepNumber)
		{
		case 0:
			Log_Debug("STEP 0\n");
			GPIO_SetValue(IN1, GPIO_Value_Low);
			GPIO_SetValue(IN2, GPIO_Value_High);
			GPIO_SetValue(IN3, GPIO_Value_High);
			GPIO_SetValue(IN4, GPIO_Value_High);
			break;
		case 1:
			Log_Debug("STEP 1\n");
			GPIO_SetValue(IN1, GPIO_Value_High);
			GPIO_SetValue(IN2, GPIO_Value_Low);
			GPIO_SetValue(IN3, GPIO_Value_High);
			GPIO_SetValue(IN4, GPIO_Value_High);
			break;
		case 2:
			Log_Debug("STEP 2\n");
			GPIO_SetValue(IN1, GPIO_Value_High);
			GPIO_SetValue(IN2, GPIO_Value_High);
			GPIO_SetValue(IN3, GPIO_Value_Low);
			GPIO_SetValue(IN4, GPIO_Value_High);
			break;
		case 3:
			Log_Debug("STEP 3\n");
			GPIO_SetValue(IN1, GPIO_Value_High);
			GPIO_SetValue(IN2, GPIO_Value_High);
			GPIO_SetValue(IN3, GPIO_Value_High);
			GPIO_SetValue(IN4, GPIO_Value_Low);
			break;

		default:
			break;
		}
		stepNumber++;
		if (stepNumber == 4)
			stepNumber = 0;
	}
	else
	{
		GPIO_SetValue(IN1, GPIO_Value_High);
		GPIO_SetValue(IN2, GPIO_Value_High);
		GPIO_SetValue(IN3, GPIO_Value_High);
		GPIO_SetValue(IN4, GPIO_Value_High);
	}
}

//event handler for button press
static EventData buttonEventData = { .eventHandler = &ButtonTimerEventHandler };

//event handler for stepper motor
static EventData motorTurnEventData = { .eventHandler = &StepperMotorEventHandler };

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = TerminationHandler;
	sigaction(SIGTERM, &action, NULL);

	epollFd = CreateEpollFd();
	if (epollFd < 0) {
		return -1;
	}

	// Open button GPIO as input, and set up a timer to poll it
	Log_Debug("Opening SAMPLE_BUTTON_1 as input.\n");
	button_A_GpioFd = GPIO_OpenAsInput(AVNET_MT3620_SK_USER_BUTTON_A);
	if (button_A_GpioFd < 0) {
		Log_Debug("ERROR: Could not open button A GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}
	struct timespec buttonPressCheckPeriod = { 0, 1000000 };
	buttonPollTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &buttonPressCheckPeriod, &buttonEventData, EPOLLIN);
	if (buttonPollTimerFd < 0)
	{
		return -1;
	}

	// Open LED GPIO, set as output with value GPIO_Value_High (off), and set up a timer to blink it
	Log_Debug("Opening SAMPLE_LED as output.\n");
	greenLEDFd = GPIO_OpenAsOutput(AVNET_MT3620_SK_USER_LED_GREEN, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (greenLEDFd < 0)
	{
		Log_Debug("ERROR: Could not open green LED GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	//open GPIO pin for IN4 from driver board
	Log_Debug("Init IN4 GPIO.\n");
	IN4 = GPIO_OpenAsOutput(AVNET_MT3620_SK_GPIO34, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (IN4 < 0)
	{
		Log_Debug("ERROR: Could not open IN4 GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	//open GPIO pin for IN3 from driver board
	Log_Debug("Init IN3 GPIO.\n");
	IN3 = GPIO_OpenAsOutput(AVNET_MT3620_SK_GPIO31, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (IN3 < 0)
	{
		Log_Debug("ERROR: Could not open IN3 GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	//open GPIO pin for IN2 from driver board
	Log_Debug("Init IN2 GPIO.\n");
	IN2 = GPIO_OpenAsOutput(AVNET_MT3620_SK_GPIO33, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (IN2 < 0)
	{
		Log_Debug("ERROR: Could not open IN2 GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	//open GPIO pin for IN2 from driver board
	Log_Debug("Init IN1 GPIO.\n");
	IN1 = GPIO_OpenAsOutput(AVNET_MT3620_SK_GPIO32, GPIO_OutputMode_PushPull, GPIO_Value_High);
	if (IN1 < 0)
	{
		Log_Debug("ERROR: Could not open IN1 GPIO: %s (%d).\n", strerror(errno), errno);
		return -1;
	}

	/*create timer event for stepper motor. This loop fires evey 0.002048 seconds. 
	If the A button is pressed it will move the motor to the next appropiate step*/
	struct timespec motorStepperTimePeriod = { 0,2048000 };
	stepperMotorTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &motorStepperTimePeriod, &motorTurnEventData, EPOLLIN);
	if (stepperMotorTimerFd< 0)
	{
		return -1;
	}

	return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
	Log_Debug("Closing file descriptors.\n");

	CloseFdAndPrintError(epollFd, "Epoll");
	CloseFdAndPrintError(buttonPollTimerFd, "Button A Timer");
	CloseFdAndPrintError(greenLEDFd, "Green LED");

	CloseFdAndPrintError(button_A_GpioFd, "Button A GPIO");
	CloseFdAndPrintError(stepperMotorTimerFd, "Stepper motor timer event");

	CloseFdAndPrintError(IN1, "IN1 GPIO");
	CloseFdAndPrintError(IN2, "IN2 GPIO");
	CloseFdAndPrintError(IN3, "IN3 GPIO");
	CloseFdAndPrintError(IN4, "IN4 GPIO");
}

int main(void)
{
	Log_Debug("GPIO application starting.\n");
	if (InitPeripheralsAndHandlers() != 0) {
		terminationRequired = true;
	}

	// Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
	while (!terminationRequired) {
		if (WaitForEventAndCallHandler(epollFd) != 0) {
			terminationRequired = true;
		}
	}

	ClosePeripheralsAndHandlers();
	Log_Debug("Application exiting.\n");
	return 0;
}