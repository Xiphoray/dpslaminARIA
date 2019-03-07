#include <iostream>
#include "Aria.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <math.h>

#include <string.h>
#include <thread>
#include <stdio.h>
#include <stdlib.h>

#include "high.h"
#include "mt-rand.h"
#include "CImg.h"

using namespace cimg_library;
// The initial seed used for the random number generated can be set here.
#define SEED 1

// Default names for printing map files.
// File types are automatically appended.
#define MAP_PATH_NAME "map"
#define PARTICLES_PATH_NAME "particles"


//
// Globals
//

// The number of iterations between writing out the map for video. 0 is off.
int VIDEO = 10;

// The current commands being given to the robot for movement. 
// Not used when the robot is reading data from a log file.
double RotationSpeed, TranslationSpeed;

// The means by which the slam thread can be told to halt, either by user command or by the end of a playback file.
int continueSlam;
int PLAYBACK_COMPLETE = 0;

//
//
// InitializeRobot
//
// Calls the routines in 'ThisRobot.c' to initialize the necessary hardware and software.
// Each call has an opportunity to return a value of -1, which indicates that the initialization of that
// part of the robot has failed. In that event, Initialize robot itself will return a general error of -1.
//
//
int InitializeRobot(ArArgumentParser *parser, ArRobot *robot, ArRobotConnector *robotConnector, ArLaserConnector *laserConnector, int argc, char *argv[]) {
	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.

	if (!(*robotConnector).connectRobot())
	{
		ArLog::log(ArLog::Terse, "lasersExample: Could not connect to the robot.");
		if ((*parser).checkHelpAndWarnUnparsed())
		{
			fprintf(stderr, "Start up initialization of the robot has failed.\n");
			// -help not given
			Aria::logOptions();
			return -1;
		}
	}
	fprintf(stderr, "\n  ");
	if (!Aria::parseArgs())
	{
		Aria::logOptions();
		return -1;
	}
	fprintf(stderr, "\n  ");
	ArLog::log(ArLog::Normal, "Connected to robot.");
	// Start the robot processing cycle running in the background.
	// True parameter means that if the connection is lost, then the 
	// run loop ends.

	fprintf(stderr, "\nConnecting Laser.\n");
	(*robot).runAsync(true);

	// Connect to laser(s) as defined in parameter files.
	// (Some flags are available as arguments to connectLasers() to control error behavior and to control which lasers are put in the list of lasers stored by ArRobot. See docs for details.)
	if (!(*laserConnector).connectLasers())
	{
		ArLog::log(ArLog::Terse, "Could not connect to configured lasers. Exiting.");
		return -1;
	}

	// Allow some time to read laser data
	ArUtil::sleep(500);
	ArLog::log(ArLog::Normal, "Connected to all lasers.");
	return 0;
}

class SlamThread : public ArASyncTask
{
	ArCondition SlamCondition;
	ArMutex myMutex;
	ArRobot *robot;

public:
	/* Construtor. Initialize counter. */
	SlamThread()
	{
		SlamCondition.setLogName("SlamThreadCondition");
	}
	/* This method is called in the new thread when launched. The void* parameter
	 * and return value are platform implementation-specific and can be ignored.
	 * This method will run in a loop, incrementing the counter each second, but
	 * locking the mutex to prevent conflicting access by other threads.
	 * If it reaches a value divisible by ten, signal our condition variable.
	 */
	void* runThread(void*)
	{

		TPath *path, *trashPath;
		TSenseLog *obs, *trashObs;

		myMutex.lock();
		//InitHighSlam();
		InitLowSlam(robot);
		myMutex.unlock();
		ArLog::log(ArLog::Normal, "InitLowSlam finished");

		// Run until the thread is requested to end by another thread.
		while (this->getRunningWithLock())
		{
			myMutex.lock();

			ArLog::log(ArLog::Normal, "begin lowslam");
			LowSlam(continueSlam, &path, &obs, robot);
			//HighSlam(path, obs);

			// Get rid of the path and log of observations
			while (path != NULL) {
				trashPath = path;
				path = path->next;
				free(trashPath);
			}
			while (obs != NULL) {
				trashObs = obs;
				obs = obs->next;
				free(trashObs);
			}

			myMutex.unlock();
			ArUtil::sleep(500);
		}
		ArLog::log(ArLog::Normal, "Slam thread: requested stop running, ending thread.");
		return NULL;
	}
	/* Other threads can call this to wait for a condition eventually
	 * signalled by this thread. (So note that in this example program, this
	 * function is not executed within "Example thread", but is executed in the main thread.)
	 */
	void waitOnCondition()
	{
		SlamCondition.wait();
		ArLog::log(ArLog::Normal, " %s ArCondition object was signalled, done waiting for it.", SlamCondition.getLogName());
	}
	/* Get the counter. Not threadsafe, you must lock the mutex during access. */
	ArRobot *getRobot() { return robot; }
	/* Set the countner. Not threadsafe, you must lock the mutex during access. */
	void setRobot(ArRobot *ctr) { robot = ctr; }
	/* Lock the mutex object.  */
	void lockMutex() { myMutex.lock(); }
	/* Unlock the mutex object. */
	void unlockMutex() { myMutex.unlock(); }
};


//void *Slam(ArRobot *robot)
//{
//	TPath *path, *trashPath;
//	TSenseLog *obs, *trashObs;
//
//	//InitHighSlam();
//	InitLowSlam(robot);
//	ArLog::log(ArLog::Normal, "InitLowSlam finished");
//	while (continueSlam) {
//
//		ArLog::log(ArLog::Normal, "begin lowslam");
//		LowSlam(continueSlam, &path, &obs, robot);
//		//HighSlam(path, obs);
//
//		// Get rid of the path and log of observations
//		while (path != NULL) {
//			trashPath = path;
//			path = path->next;
//			free(trashPath);
//		}
//		while (obs != NULL) {
//			trashObs = obs;
//			obs = obs->next;
//			free(trashObs);
//		}
//		ArUtil::sleep(800);
//	}
//	return NULL;
//}


bool imgflash = false;
class ImgThread : public ArASyncTask
{
	ArCondition ImgCondition;
	ArMutex myMutex;

public:
	/* Construtor. Initialize counter. */
	ImgThread()
	{
		ImgCondition.setLogName("ImgThreadCondition");
	}
	/* This method is called in the new thread when launched. The void* parameter
	 * and return value are platform implementation-specific and can be ignored.
	 * This method will run in a loop, incrementing the counter each second, but
	 * locking the mutex to prevent conflicting access by other threads.
	 * If it reaches a value divisible by ten, signal our condition variable.
	 */
	void* runThread(void*)
	{

		int argc = 0;
		char **argv = NULL;

		// Run until the thread is requested to end by another thread.
		while (this->getRunningWithLock())
		{

			const char* file_i = cimg_option("-i", "1.ppm", "Input image");
			const double sigma = cimg_option("-blur", 1.0, "Variance of gaussian pre-blurring");
			const CImg<unsigned char> image = CImg<>(file_i).normalize(0, 255);
			CImgDisplay disp(image, "Color image (Try to move mouse pointer over)", 0);
			while (1) {

				cimg::wait(200);
				if (imgflash) {
					imgflash = false;
					break;
				}

			}

			fprintf(stderr, "Map dumped to file\n");

			
		}
		ArLog::log(ArLog::Normal, "Img thread: requested stop running, ending thread.");
		return NULL;
	}
	/* Other threads can call this to wait for a condition eventually
	 * signalled by this thread. (So note that in this example program, this
	 * function is not executed within "Example thread", but is executed in the main thread.)
	 */
	void waitOnCondition()
	{
		ImgCondition.wait();
		ArLog::log(ArLog::Normal, " %s ArCondition object was signalled, done waiting for it.", ImgCondition.getLogName());
	}

	/* Lock the mutex object.  */
	void lockMutex() { myMutex.lock(); }
	/* Unlock the mutex object. */
	void unlockMutex() { myMutex.unlock(); }
};
//void *showmap()
//{
//	int argc = 0;
//	char **argv = NULL;
//	while (1) {
//		const char* file_i = cimg_option("-i", "1.ppm", "Input image");
//		const double sigma = cimg_option("-blur", 1.0, "Variance of gaussian pre-blurring");
//		const CImg<unsigned char> image = CImg<>(file_i).normalize(0, 255).blur((float)sigma).resize(-100, -100, 1, 3);
//		CImgDisplay disp(image, "Color image (Try to move mouse pointer over)", 0);
//		while (1) {
//			cimg::wait(200);
//			if (imgflash) {
//				imgflash = false;
//				break;
//			}
//		}
//		
//		fprintf(stderr, "Map dumped to file\n");
//	}
//
//	return NULL;
//}

int main(int argc, char **argv)
{

	Aria::init();
	ArRobot robot;
	SlamThread slamThread;
	ImgThread imgThread;
	ArArgumentParser parser(&argc, argv);
	parser.loadDefaultArguments();
	ArRobotConnector robotConnector(&parser, &robot);
	ArLaserConnector laserConnector(&parser, &robot, &robotConnector);
	fprintf(stderr, "********** Localization Example *************\n");
	if (InitializeRobot(&parser, &robot, &robotConnector, &laserConnector, argc, argv) == -1) {
		Aria::exit(1);
		return -1;
	}
	PLAYBACK = "";
	RECORDING = "";
	fprintf(stderr, "********** World Initialization ***********\n");

	seedMT(SEED);
	// Spawn off a seperate thread to do SLAM
	//
	// Should use semaphores or similar to prevent reading of the map
	// during updates to the map.
	//
	continueSlam = 1;
	//std::thread slam(Slam, &robot);
	//std::thread showmap(showmap);
	if (robot.isConnected())
	{
		//slam.join();
		slamThread.setRobot(&robot);
	}
	//if (!robotConnector.connectRobot())
	//{
	//	ArLog::log(ArLog::Terse, "Could not connect to the robot.");
	//	if (parser.checkHelpAndWarnUnparsed())
	//	{
	//		// -help not given, just exit.
	//		Aria::logOptions();
	//		Aria::exit(1);
	//		return 1;
	//	}
	//}
	// Trigger argument parsing
	if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
	{
		Aria::logOptions();
		Aria::exit(1);
		return 1;
	}



	robot.runAsync(true);

	// turn on the motors, turn off amigobot sounds
	robot.enableMotors();
	robot.comInt(ArCommands::SOUNDTOG, 0);
	// add a set of actions that combine together to effect the wander behavior
	ArActionStallRecover recover;
	ArActionBumpers bumpers;
	ArActionAvoidFront avoidFrontNear("Avoid Front Near", 112, 0);
	ArActionAvoidFront avoidFrontFar;
	ArActionConstantVelocity constantVelocity("Constant Velocity", 200);
	robot.addAction(&recover, 50);
	robot.addAction(&bumpers, 37);
	robot.addAction(&avoidFrontNear, 25);
	robot.addAction(&avoidFrontFar, 25);
	robot.addAction(&constantVelocity, 12);

	slamThread.runAsync();
	imgThread.runAsync();
	// wait for robot task loop to end before exiting the program
	robot.waitForRunExit();

	Aria::exit(0);
	return 0;
}

