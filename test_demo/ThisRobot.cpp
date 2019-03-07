//
// This Program is provided by Duke University and the authors as a service to the
// research community. It is provided without cost or restrictions, except for the
// User's acknowledgement that the Program is provided on an "As Is" basis and User
// understands that Duke University and the authors make no express or implied
// warranty of any kind.  Duke University and the authors specifically disclaim any
// implied warranty or merchantability or fitness for a particular purpose, and make
// no representations or warranties that the Program will not infringe the
// intellectual property rights of others. The User agrees to indemnify and hold
// harmless Duke University and the authors from and against any and all liability
// arising out of User's use of the Program.
//
// Copyright 2005 Austin Eliazar, Ronald Parr, Duke University
//

/*****************************************************
 *           PLAYBACK ROBOT
 * This is basically an empty file for
 * allowing the code to run offline,
 * with all of the direct calls to the robot
 * doing nothing. This is to be used exclusively
 * in playback mode, and any attempts to do
 * otherwise will probably give very boring results.
 *****************************************************/


 // All base robot files are required to have this exact same definition. DO NOT CHANGE!
 // This is the structure in which the SICK laser data is stored, so that the SLAM program
 // can use it.

#include "ThisRobot.h"

TOdo odometry;


int InitializeThisRobot(int argc, char *argv[]) {

	return 0;
}

int ConnectOdometry(int argc, char *argv[]) {
	return 0;
}

int ConnectLaser(int argc, char *argv[]) {
	return 0;
}

int ConnectDrive(int argc, char *argv[]) {
	return 0;
}

// This is never going to be called if we are truly running this offline. If for some reason this is run
// not in playback mode, this function wont do anything.
void GetSensation(TOdo &odometry, TSense &sense, ArRobot **robot) {
	int max;
	int numLasers = 0;
	ArLog::log(ArLog::Normal, "test");
	(**robot).lock();
	std::map<int, ArLaser*> *lasers = (**robot).getLaserMap();
	for (std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i)
	{
		int laserIndex = (*i).first;
		ArLaser* laser = (*i).second;
		if (!laser)
			continue;
		++numLasers;
		laser->lockDevice();
		odometry.x = laser->getSensorPositionX() * MAP_SCALE / 1000.0;
		odometry.y = laser->getSensorPositionY() * MAP_SCALE / 1000.0;
		// The current readings are a set of obstacle readings (with X,Y positions as well as other attributes) that are the most recent set from teh laser.
		std::list<ArPoseWithTime*> *currentReadings = laser->getCurrentBuffer(); // see ArRangeDevice interface doc
		// The raw readings are just range or other data supplied by the sensor. It may also include some device-specific extra values associated with each reading as well. (e.g. Reflectance for LMS200)
		const std::list<ArSensorReading*> *rawReadings = laser->getRawReadings();
		int thsensor = 0;
		for (std::list<ArPoseWithTime*>::const_iterator j = currentReadings->begin(); j != currentReadings->end(); ++j)
		{
			ArPoseWithTime* sen = *j;
			double X = sen->findDistanceTo((**robot).getPose()) / 5000.0;
			sense[thsensor].distance = X*MAP_SCALE;
			thsensor++;
		}
		laser->unlockDevice();
	}
	(**robot).unlock();
	return;
}


void GetOdometry(TOdo &odometry , ArRobot **robot) {
	(**robot).lock();
	odometry.theta = (**robot).getTh() / 180.0*M_PI;
	//odometry.x = (**robot).getX() / 1000.0;
	//odometry.y = (**robot).getY() / 1000.0;
	
	(**robot).unlock();
	if (odometry.theta > M_PI)
		odometry.theta = odometry.theta - 2.0 * M_PI;
	else if (odometry.theta < -M_PI)
		odometry.theta = odometry.theta + 2.0 * M_PI;


	//odometry.x = odometry.x - (cos(odometry.theta)*TURN_RADIUS / MAP_SCALE);
	//odometry.y = odometry.y - (sin(odometry.theta)*TURN_RADIUS / MAP_SCALE);
	return;
}


void Drive(double speed, double turn) {
	return;
}
