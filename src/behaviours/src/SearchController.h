#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"

using namespace std;
/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();

  //ShortCircuits changes
  void SetInterrupted(bool centerSeen);

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;

  //ShortCircuits changes
  bool hasSearchPoint;    //tells searchcontroller if it currently has generated a valid search point to travel too
  bool wasInterrupted;    //tells searchcontroller if obstaclecontroller has interrupted search from doing its work previously
  bool hasStartedPattern; //tells searchcontroller if it has started performing an octagon or not
  bool returning;         //tells searchcontroller if it is currently in the process of returning home
  int positionInSearch;   //keeps track of current position inside the fixed octagon position
  int numTimesExceeded;   //Holds the number of times searchcontroller has attempted to go to a point and exceeded its trial attempts.

  //Flag for which x-axis or y-axis boarder line restriction
  bool useX = false;
  bool useY = false;

  float ChooseRandomTheta(float roverAngle);  //Chooses Random Theta
  Point ChooseRandomPoint();                  //Creates a random point to search around
  Point GenDeliberatePoint();                 //Creates a point on the octagonal shaped fixed search pattern
  Point InterruptedLogic();                   //Handles ALL logic for if the search controller was interrupted by obstacle
  void ResetSearchState();
  
  Point Turn180();                            //Turns rover around 180 degrees
  void  SetBoarderValues(float initTheta);    //Sets up boarder values.
  bool  IsWithinBoundary(Point searchPoint);  //Checks if the searchPoint. 

};

#endif /* SEARCH_CONTROLLER */
