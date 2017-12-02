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
  returning = false;
}

void SearchController::Reset()
{
  result.reset = false;
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

void SearchController::SetSuccesfullPickup()
{
  succesfullPickup = true;
}


/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork()
{

  cout << "SearchController doing work" << endl;

  if (!result.wpts.waypoints.empty())
  {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.15)
    {
      attemptCount = 0;
    }
  }

  //searchLocation declaration in .h

  result.type = waypoint;
  Point destination;

  //select new position 50 cm from current location
  if (first_waypoint)
  {
    //first_waypoint = false;

    SetBoarderValues();

    if(!useY)
    {
      searchLocation = ChooseRandomPoint();
      useY = true;
    }

    cout << "Current Point X:  " << searchLocation.x << "  Current Point Y:  " << searchLocation.y << endl;

    if(useX)
    {
      cout << "Distance from Center for boundry:  " << boundary_distance << endl;

      float checkX = centerLocation.x + boundary_distance;
      cout << "Can't go past on X: " << checkX << endl;

      if(boundary_distance < 0)
      {
        if(searchLocation.x < checkX)
        {
          cout << "NOT A VALID POINT" << endl;
        }
        else
        {
          cout << "VALID POINT" << endl;
        }
      }
      else if(boundary_distance > 0)
      {
        if(searchLocation.x > checkX)
        {
          cout << "NOT A VALID POINT" << endl;
        }
        else
        {
          cout << "VALID POINT" << endl;
        }

      }

    }
    else
    {
      cout << "Boundry on Y axis after:  " << centerLocation.y + boundary_distance;
    }

    //destination = Turn180();
    destination = currentLocation;

    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), destination);

    return result;
  }


  /*
   * Performs a 2 stage search.
   * (Checks to see if it was interrupted during any process)
   *
   * 1.) It will generate a random point to drive to
   * 2.) Do an octagon pattern around the generated point once it arrives
   * 3.) Return back to the center once it has finished the pattern
   * 4.) Go back to step 1
  */


  //If Rover had picked up a cube during the search
  if(succesfullPickup)
  {
    //THIS ENSURES SOME SITE FIDELITY
    cout << "Rover had picked up a cube!" << endl;

    //if rover was in the octagon after the first point
    if(positionInSearch > 1)
    {
      cout << "Interrupted while doing the octagon search, going back on point" << endl;
      positionInSearch--;                   //go back one position in the octagon
    }

    // Otherwise rover was driving to first point
    else
    {
      cout << "Interrupted going to first point in octagon, heading back to searchLocation to start pattern again" << endl;
      destination = searchLocation;         //go back to the searchLocation
      hasStartedPattern = false;     //reset hasStartedPattern (Just In Case)
      positionInSearch = 0;          //reset positionInSearch back to 0
    }

    succesfullPickup = false;

    //to refrain from duplicate obstacle calls, seeing center or other issues after trying to drop off the cube would effect search controller
    if(wasInterrupted)
    {
      wasInterrupted = false;
    }
  }

  if (wasInterrupted)
  {
    //series of logic checks to see what point needs to be driven too after being interrupted by obstacle controller.  returns appropriate point
    destination = InterruptedLogic();
  }
  
  else if(!hasSearchPoint)  //if we have finished/don't currently have/need a new search area make one
  {
    cout << "Doesn't have search area, generating one" << endl;
    do{
    	searchLocation = ChooseRandomPoint();
    }while(!IsWithinBoundary(searchLocation));
    destination = searchLocation;
  }
  else //otherwise search around the area
  {
    cout << "Has Search Area, performing search pattern" << endl;
    destination = GenDeliberatePoint();
  }

  if(wasInterrupted)
  {
    wasInterrupted = false;
  }
  else
  {
    attemptCount = 0;
  }


  // cout << ">>> DESTINATION: " << destination.x << ", " << destination.y << endl;
  // destination.x = 4.98511;
  // destination.y = -2.71179;

  //find appropriate theta using robots current position and the position recently generated
  // destination.theta = atan2((destination.y - currentLocation.y), (destination.x - currentLocation.x));
  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), destination);

  return result;

}


//SetInterrupted is called whenever obstaclecontroller sees and obstacle. Tells this method if it saw the center or a physical obstacle
void SearchController::SetInterrupted(bool centerSeen)
{
  //If this is the first time it has been interrupted for an obstacle (this time)
  if(!wasInterrupted)
  {
    if(centerSeen)
    {
      returning = false;

      // if we haven't started a generating deliberate points and see the center
      if(!hasStartedPattern)
      {
        //Assume we are trying to drive through the center

        //Force Rover to Generate new Search Point
        hasSearchPoint = false;
      }

    }

    wasInterrupted = true;
  }
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

  //create bool
  //start loop

  //ARENA IS 50x50
  const int MAX_ARENA_SIZE = 7;
  const int MIN_SEARCH_DIST = 1;

  float searchDist = rng->uniformReal(MIN_SEARCH_DIST, MAX_ARENA_SIZE);

  cout << "SEARCH DISTANCE:  " << searchDist << endl;

  Point temp;

  //theta is the angle at which to plot the new point
  temp.theta = ChooseRandomTheta(currentLocation.theta);

  temp.x = centerLocation.x + (searchDist * cos(temp.theta));
  temp.y = centerLocation.y + (searchDist * sin(temp.theta));

  //pass temp to check method
  //check if result is good
  //if not repeat loop
  //if good return temp

  hasSearchPoint = true;    //say we have a search point
  returning = false;        //reset our returning variable

  return temp;
}

/**
  * Generates various search patterns for testing
  * currently experiment with octagons.
  **/


//GENDELIBERATEPOINT is a method that is called when the rover is in the process of doing a fixed pattern search.  Currently implements an octagonal pattern centered around the
//randomly generated searchpoint
Point SearchController::GenDeliberatePoint()
{
  Point temp;

  float patternSize = 0.5;      //sets distance from searchPoint for octagon pattern

  cout << "GOING TO POINT " << positionInSearch << endl;

  //Octagon ninth point will set it next location to the centerLocation.
  if(positionInSearch > 8 || positionInSearch < 0)
  {
    temp.x = centerLocation.x;
    temp.y = centerLocation.y;
    positionInSearch = 0;
    hasSearchPoint = false;
    returning = true;
  }

  //calculates the next x,y location from the searchLocation
  //angles are 45 degrees apart form one another.
  //source for math logic: https://www.redblobgames.com/grids/hexagons/
  else
  {
    hasStartedPattern = true;

    positionInSearch++;

    //if a value below 1 then this implementation does not work currently
    float ang_deg = 45 * positionInSearch;
    float ang_rad = M_PI/180 * ang_deg;

    temp.x = searchLocation.x + patternSize * cos(ang_rad);
    temp.y = searchLocation.y + patternSize * sin(ang_rad);
  }

  //find appropriate theta using robots current position and the position recently generated
  temp.theta = atan2((temp.y - currentLocation.y), (temp.x - currentLocation.x));

  float dist = hypot((temp.y - currentLocation.y), (temp.x - currentLocation.x));

  cout << "CALCULATED DISTANCE IN SEARCH:  " << dist << endl;
  return temp;

}

Point SearchController::InterruptedLogic()
{

  Point temp;

  //---------------------------------------------------------------------------------------

  //CHECK ATTEMPTS
  //DUAL STAGE CHECK:
  // 1.)  Check to see how many times the rover has tried to get to a point
  // 2.)  Check to see how many times the rover has exceeded its tries in step 1

  const int num_of_tries = 3;
  bool exceededAttempts = false;

  attemptCount++;
  cout << "Avoided Obstacle; Attempt #" << attemptCount << endl;

  //if we have exceeded our limits of attempts (1st CHECK)
  if(attemptCount > num_of_tries)
  {
    cout << "Exceeded # Attempts" << endl;
    exceededAttempts = true;        //trigger exceeded behavior
    numTimesExceeded++;
    attemptCount = 0;               //RESET attempt count
  }

  //If rover has failed consistantly 3 times to get anywhere (2nd CHECK)
  if(numTimesExceeded > num_of_tries)
  {
    //JUST GO HOME!  Reset all variables
    cout << "Exceeded Attempts 3 times in a row, going back to center" << endl;
    returning = true;
    hasSearchPoint = false;
    positionInSearch = 0;
    temp = centerLocation;
  }

  //---------------------------------------------------------------------------------------
  //BEGIN LOGIC
  //---------------------------------------------------------------------------------------

  //This is the case if returning back to center after a pattern is complete or trying to drive across the center after being interrupted by obstacle
  if(!hasSearchPoint)
  {
    //Rover was deliberately set to generate a new point by switching hasSearchPoint to false
    if(!returning)
    {
      cout << "Interrupted by seeing center, assuming driving through center.  Forcing a new Search Point to be Generated" << endl;
      temp = ChooseRandomPoint();        //create new point
    }
    //Rover is trying to go back to the center location
    else
    {
      //Rover will set heading to center as it is returning
      cout << "Interrupted on way back to center, going back to center!" << endl;
      temp = centerLocation;         //go back to center
    }
  }

  //---------------------------------------------------------------------------------------

  //This is the case if driving to a position to start a deliberate pattern but never made it there
  else if(!hasStartedPattern && !returning)
  {
    if(!exceededAttempts)
    {
      cout << "Was Interrupted on the way to searchLocation, going back" << endl;
      temp = searchLocation;           //go back to searchLocation
    }
    else if (returning)
    {
      //Rover will set heading to center as it is returning
      cout << "Interrupted on way back to center, going back to center!" << endl;
      temp = centerLocation;         //go back to center
    }
    //assume position is unreachable
    else
    {
      cout << "Abandoning search point, may be unreachable" << endl;
      temp = centerLocation;
    }
  }

  //---------------------------------------------------------------------------------------

  //Otherwise it was in the middle of the deliberate search pattern somewhere...
  else
  {

    //if rover was in the octagon after the first point
    if(positionInSearch > 1)
    {
      if(!exceededAttempts)
      {
        cout << "Interrupted while doing the octagon search, going back on point" << endl;
        positionInSearch--;                   //go back one position in the octagon
        temp = GenDeliberatePoint();          //get that point
      }
      else
      {
        cout << "Abandoning octagon point, may be unreachable" << endl;
        temp = GenDeliberatePoint();          //go to next point
      }
    }

    // Otherwise rover was driving to first point
    else
    {
      if(!exceededAttempts)
      {
        cout << "Interrupted going to first point in octagon, heading back to searchLocation to start pattern again" << endl;
        temp = searchLocation;         //go back to the searchLocation
        hasStartedPattern = false;     //reset hasStartedPattern (Just In Case)
        positionInSearch = 0;          //reset positionInSearch back to 0
      }
      else
      {
        cout << "Abandoning octagon point, may be unreachable" << endl;
        temp = GenDeliberatePoint();          //go to next point
      }
    }

  }

  return temp;
}

Point SearchController::Turn180()
{
  Point temp;

  temp.theta = currentLocation.theta + M_PI;
  temp.x = currentLocation.x + (0.5 * cos(temp.theta));
  temp.y = currentLocation.y + (0.5 * sin(temp.theta));  //Q4:  0 to pi/2
  //Q3:  pi/2 to pi
  //Q2:  pi to 3pi/2
  //Q1:  3pi/2 to 2pi
  return temp;
}

void SearchController::SetBoarderValues()
{

	/* Rover position on odom map
             	   (Pi/2) 
		  			 N
		  			 ^
			(-,+)	 |     (+,+)
	   			 	 R2
	    (pi) E< R1 __center__R3 >W (0)
	     	         |
			(-, -)   R2      (+,-)
					 S
	*/


  //Get Current Rovers Theta
  //---------------------------------------------------------------------------------------

  float initTheta = angles::normalize_angle_positive(currentLocation.theta);
  //cout << "(" << currentLocation.x << "," << currentLocation.y << ")" << endl;
  //cout << "CALLING SetBoarderValues initTheta" << initTheta << endl;

  //---------------------------------------------------------------------------------------

  //Check to see which Quadrant it is in, and assign it that quadrant

  //---------------------------------------------------------------------------------------


  //Conditions for Quadrants

  //Q1: (theta < pi/4  || theta > 7pi/4)
  //Q2: (theta < 3pi/4 || theta > pi/4)
  //Q3: (theta < 5pi/5 || theta > 3pi/4)
  //Q4: (theta < 7pi/4 || theta > 5pi/4)

  const float PI_4 = .785398;
  const float PI3_4 = 2.356194;
  const float PI5_4 = 3.926990;
  const float PI7_4 = 5.497787;

  int quadrant = 0;

  /*

        (3pi/4) \   Q2  /(pi/4)
                 \  |  /
                  \ | /
                   \|/
  ------Q3----------|----------Q1------
                   /|\
                  / | \
                 /  |  \
        (5pi/4) /   Q4  \(7pi/4)
  */


  if(initTheta <= (PI_4))
  {
    cout << "I AM IN QUADRANT 1:  "  << initTheta << endl;
    quadrant = 1;
  }
  else if(initTheta <= (PI3_4))
  {
    cout << "I AM IN QUADRANT 2:  " << initTheta << endl;
    quadrant = 2;
  }
  else if(initTheta <= (PI5_4))
  {
    cout << "I AM IN QUADRANT 3:  " << initTheta << endl;
    quadrant = 3;
  }
  else if (initTheta <= PI7_4)
  {
    cout << "I AM IN QUADRANT 4:  " << initTheta << endl;
    quadrant = 4;
  }
  else
  {
    cout << "I AM IN QUADRANT 1:  "  << initTheta << endl;
    quadrant = 1;
  }

  //---------------------------------------------------------------------------------------

  //Calculate Boundaries relative to the starting quadrant
  // - Determine if the boundry is in the X or the Y
  // - Determine sign

  //---------------------------------------------------------------------------------------

  switch (quadrant)
  {
    case 1:
      useX = true;
      boundary_distance = -1;
      break;
    case 2:
      useX = false;
      boundary_distance = -1;
      break;
    case 3:
      useX = true;
      break;
    case 4:
      useX = false;
    default:
      break;
  }

/*

	//Rover 1: initial theta range (pi/4, 7pi/4)
	//if range is satisfied sets useY boundary value to true
  //cout << "range (" << M_PI/4 << "," << 7*M_PI/4  << ")"<< endl;
	if(initTheta > 0 && initTheta < M_PI/4 || initTheta > 7*M_PI/4  && initTheta < 2*M_PI) //because X = 0 angle coulde be 0 radians or 2pi
	{
		useX = false;
		useY = true;
    //cout << "range (pi/4, 7pi/5)" << endl;
	}

	//Rover 2: initial theta range (pi/4, 3pi/4)
	//if range is satisfied sets useX boundary value to true
  //cout << "range (" << M_PI/4 << "," << 3*M_PI/4 << ")" << endl;
	if(initTheta > M_PI/4 && initTheta < 3*M_PI/4)
		//initTheta > 5*M_PI/4 && initTheta < 3*M_PI/2 || initTheta > 3*M_PI/2 && initTheta < 7*M_PI/4)
	{
		useX = true;
		useY = false;
    //cout << "range (pi/4, 3pi/4)" << endl;
	}

	//Rover 2: initial theta range (5pi/4, 7pi/4)
	//if range is satisfied sets useX boundary value to true
    //cout << "range (" << 5*M_PI/4 << "," << 7*M_PI/4 << ")" << endl;
	if(initTheta > 5*M_PI/4 && initTheta < 7*M_PI/4)
	{
		useX = true;
		useY = false;
    //cout << "range (5pi/4, 7pi/4)" << endl;
	}

	//Rover 3: initial theta range (3pi/4 , 5pi/4)
	//if range is satisfied sets useX and useY value to true
  //cout << "range (" << 3*M_PI/4 << "," << 5*M_PI/4 << ")" << endl;
	//if(initTheta > 3*M_PI/4 && initTheta < M_PI || initTheta > M_PI && initTheta < 5*M_PI/4)
	if(initTheta > 3*M_PI/4 && initTheta < 5*M_PI/4)
	{
		useX = true;
		useY = true;
    //cout << "range (3pi/4 , 5pi/4)" << endl;
	}

  */
}

bool SearchController::IsWithinBoundary(Point searchPoint)
{
	Point centerPoint;

	cout << "CHECKING IsWithinBoundary" << endl;

	//Rover 1 case:
	if(useY && !useX)
	{
		cout << "case 2 Y = -1" << endl;
		centerPoint.y = centerLocation.y - 1;
		cout << " centerPointY: " << centerPoint.y << "searchPointY:" << searchPoint.y << endl;
		if(searchPoint.y < centerPoint.y)
		{
			cout << "Not in bound" << endl;
			return false;
		}
	}

	//Rover 2 case: 
	else if(useX && !useY)
	{
		cout << "case 1 X = 1" << endl;
		centerPoint.x = centerLocation.x + 1;
		cout << " centerPointX: " << centerPoint.x << "searchPointX:" << searchPoint.x << endl;
		if(searchPoint.x > centerPoint.x)
		{
			cout << "Not in bound" << endl;
			return false;
		}
	}

	//Rover 3 case:
	else if(useX && useY)
	{
		cout << "case 3 X = -1, Y = 1" << endl;
		centerPoint.x = centerLocation.x - 1;
		centerPoint.y = centerLocation.y + 1;
		cout << " centerPointX: " << centerPoint.x << "searchPointX:" << searchPoint.x << endl;
		cout << " centerPointY: " << centerPoint.y << "searchPointY:" << searchPoint.y << endl;
		if(searchPoint.x < centerPoint.x || searchPoint.y > centerPoint.y)
		{
			cout << "Not in bound" << endl;
			return false;
		}
	}

	return true;
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

void SearchController::ResetSearchState()
{
  returning = false;
  hasSearchPoint = false;
  positionInSearch = 0;
}
