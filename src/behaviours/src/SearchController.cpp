#include "SearchController.h"
#include <angles/angles.h>

SearchController::SearchController()
{
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;

  hasSearchPoint = false;
  positionInSearch = 0;
  hasStartedPattern = false;
}

void SearchController::Reset()
{
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork()
{

  cout << "SearchController doing work" << endl;

  cout << "Attempt Count:  " << attemptCount << endl;

  if (!result.wpts.waypoints.empty())
  {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15)
    {
      attemptCount = 0;
    }
  }

  result.type = waypoint;

  /*
    //select new position 50 cm from current location
    if (first_waypoint)
    {
      first_waypoint = false;
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }
    else
    {
      //select new heading from Gaussian distribution around current heading
      searchLocation.theta = rng->gaussian(currentLocation.theta, 0.785398); //45 degrees in radians
      searchLocation.x = currentLocation.x + (0.5 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.5 * sin(searchLocation.theta));
    }

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
*/

  //searchLocation declaration in .h
  Point destination;

  if(!hasSearchPoint)  //if we have finished/don't currently have a new search area make one
  {
    cout << "Doesn't have search area, generating one" << endl;
    searchLocation = ChooseRandomPoint();
    destination = searchLocation;
    hasSearchPoint = true;
  }
  else if (wasInterrupted)  //if we are not within a specific distance of the searchLocation go there
  {
    cout << "Was Interrupted, going back to searchLocation" << endl;
    destination = searchLocation;
    wasInterrupted = false;
  }
  else //otherwise search around this area
  {
    cout << "Has Search Area, performing search pattern" << endl;
    destination = GenDeliberatePoint();
  }

  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), destination);

  return result;

}

void SearchController::SetCenterLocation(Point centerLocation)
{

  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;

  if (!result.wpts.waypoints.empty())
  {
    result.wpts.waypoints.back().x -= diffX;
    result.wpts.waypoints.back().y -= diffY;
  }

}

void SearchController::SetCurrentLocation(Point currentLocation)
{
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() { }

bool SearchController::ShouldInterrupt()
{
  ProcessData();

  return false;
}

bool SearchController::HasWork()
{
  return true;
}

void SearchController::SetSuccesfullPickup()
{
  succesfullPickup = true;
}

void SearchController::SetInterrupted()
{
  //if we have gone anywhere in our search pattern.  This can be called multiple times, so check to see if was
  // interupted hasn't been set yet, if yes don't keep subtracting from the search position.
  if(positionInSearch > 0 && !wasInterrupted)
  {
    //go back 1 step in the pattern
    positionInSearch--;
  }

  //tells searchController that it was interrupted
  wasInterrupted = true;
}

float SearchController::ChooseRandomTheta(float roverAngle)
{

  double absCurrentAngle = angles::normalize_angle_positive(roverAngle);

  cout << "Current Theta:  " << absCurrentAngle << endl;

  float newAngle = rng->gaussian(roverAngle, (M_PI/2)); //90 degrees in radians

  cout << "Angle to Achieve" << newAngle << endl;

  return newAngle;
}

Point SearchController::ChooseRandomPoint()
{
  //ARENA IS 12x12
  const int MAX_ARENA_SIZE = 12;
  const int MIN_SEARCH_DIST = 0.5;

  float searchDist = rng->uniformReal(MIN_SEARCH_DIST, MAX_ARENA_SIZE);

  cout << "SEARCH DISTANCE:  " << searchDist << endl;

  Point temp;

  temp.theta = ChooseRandomTheta(currentLocation.theta);

  temp.x = centerLocation.x + (searchDist * cos(temp.theta));
  temp.y = centerLocation.y + (searchDist * sin(temp.theta));

  return temp;
}

Point SearchController::GenDeliberatePoint()
{
  Point temp;

  int patternSize = 1;

  cout << "GOING TO POINT " << positionInSearch << endl;

  //Start Square Pattern
  switch(positionInSearch)
  {
  case 0:
    //POINT 1

    //generate x and y based on searchPosition

    //First Quadrant Point
    temp.x = searchLocation.x + (patternSize);
    temp.y = searchLocation.y + (patternSize);

    positionInSearch++;  //increment
    break;
  case 1:
    //POINT 2

    //generate x and y based on searchPosition

    //Second Quadrant Point
    temp.x = searchLocation.x - (patternSize);
    temp.y = searchLocation.y + (patternSize);

    positionInSearch++;  //increment
    break;
  case 2:
    //POINT 3

    //generate x and y based on searchPosition

    //Third Quadrant Point
    temp.x = searchLocation.x - (patternSize);
    temp.y = searchLocation.y - (patternSize);

    positionInSearch++;  //increment
    break;
  case 3:
    //POINT 4

    //generate x and y based on searchPosition

    //Fourth Quadrant Point
    temp.x = searchLocation.x + (patternSize);
    temp.y = searchLocation.y - (patternSize);

    positionInSearch++;  //increment
    break;
  case 4:
    //POINT 4

    //generate x and y based on searchPosition

    //First Quadrant Point AGAIN
    temp.x = searchLocation.x + (patternSize);
    temp.y = searchLocation.y + (patternSize);

    positionInSearch++;  //increment
    break;
  case 5:
    //GO HOME
    temp.x = centerLocation.x;
    temp.y = centerLocation.y;
    positionInSearch = 0;
    break;
  default:
    //shouldn't get here...
    temp.x = currentLocation.x;
    temp.y = currentLocation.y;
    break;
  }

  temp.theta = atan2((temp.y - currentLocation.y), (temp.x - currentLocation.x));


  return temp;
}
