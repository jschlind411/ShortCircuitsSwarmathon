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

  cout << "In SearchController Do Work" << endl;

  //searchLocation declaration in .h
  result.type = waypoint;
  Point destination;

  //select new position 50 cm from current location
  if (first_waypoint)
  {

    cout << "first time running, doing first time setup, turning 180" << endl;

    first_waypoint = false;
    SetBoarderValues();

    destination = Turn180();
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), destination);

    return result;
  }


  /*
   * Performs a 2 stage search.
   * (Checks to see if it was interrupted during any process)
   *
   * 1.) It will generate a random point to drive to that is within its assigned boundary
   * 2.) Do an octagon pattern around the generated point once it arrives
   * 3.) Return back to the center once it has finished the pattern
   * 4.) Go back to step 1
  */

  //If Rover had picked up a cube during the search
  if(succesfullPickup)
  {
    //THIS ENSURES SOME SITE FIDELITY
    cout << "In FinishedDropOffLogic" << endl;

    //if rover was in the octagon after the first point
    if(positionInSearch > 1)
    {
      cout << "Interrupted while doing the octagon search, going back on point" << endl;
      positionInSearch--;           //go back one position in the octagon
    }

    // Otherwise rover was driving to first point
    else
    {
      cout << "Interrupted going to first point in octagon, heading back to searchLocation to start pattern again" << endl;
      destination = searchLocation;  //go back to the searchLocation
      hasStartedPattern = false;     //reset hasStartedPattern (Just In Case)
      positionInSearch = 0;          //reset positionInSearch back to 0
    }

    succesfullPickup = false;       //reset successfullpickup state
    wasInterrupted = false;         //prevents duplicate obstacle calls
  }

  if (wasInterrupted)
  {
    //series of logic checks to see what point needs to be driven too after being interrupted by obstacle controller.  returns appropriate point
    destination = InterruptedLogic();
  }
  
  else if(!hasSearchPoint)  //if we have finished/don't currently have/need a new search area make one
  {
    cout << "Doesn't have search area, generating one" << endl;
    searchLocation = ChooseRandomPoint();
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

  //find appropriate theta using robots current position and the position recently generated
  // destination.theta = atan2((destination.y - currentLocation.y), (destination.x - currentLocation.x));
  result.wpts.waypoints.clear();
  result.wpts.waypoints.insert(result.wpts.waypoints.begin(), destination);

  return result;

}


//SetInterrupted is called whenever obstaclecontroller sees and obstacle. Tells this method if it saw the center or a physical obstacle
void SearchController::SetInterrupted(bool centerSeen)
{

  cout << "In SetInterrupt" << endl;

  //if te rover sees the center and is returning to the nest
  if(centerSeen && returning)
  {
    cout << "Saw center, triggering returning behavior off" << endl;
    returning = false;
  }

  //if we have been interrupted on the way to a searchpoint by seeing the center
  if(hasSearchPoint && centerSeen)
  {
    if(hasStartedPattern)
    {
      //SAW CENTER DOING PATTERN
      cout << "Saw center while performing pattern, setting new search point" << endl;

      hasSearchPoint = false;       //Tells robot it does not have a search point and to generate a new one
      positionInSearch = 0;         //Reset Position in Search to start from beginning
    }
    else
    {
      //ASSUME DRIVING THROUGH CENTER

      cout << "Interrupted on the way to searchLocation, assuming driving through center" << endl;

      hasSearchPoint = false;       //Tells robot it does not have a search point and to generate a new one
    }


  }

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

/*
 * Generates rover's next random point that is within boundary rover is assigned to at initial start
 * and return it.
 * 1) Will continue to run while loop if the random point generated is not in the boundary 
 *    rover is assigned with boolean isValid.
 * 2) It does boundary checking using IsWithinBoundary.
 * 3) If the rover is within its boundary while loop stops generating new randomPoints and booleans 
      hasSearchPoint and returning is modified to reset that current rover has a new search location and it is not
      returning to the collectionZone.
 */
Point SearchController::ChooseRandomPoint()
{
   cout << "In ChooseRandomPoint" << endl;

  //create bool
  //start loop
  Point temp;
  isValid = false;

  while(!isValid)
  {
    //ARENA IS 50x50
    const int MAX_ARENA_SIZE = 7;
    const int MIN_SEARCH_DIST = 1;

    float searchDist = rng->uniformReal(MIN_SEARCH_DIST, MAX_ARENA_SIZE);

    cout << "SEARCH DISTANCE:  " << searchDist << endl;

    //theta is the angle at which to plot the new point
    temp.theta = ChooseRandomTheta(currentLocation.theta);

    temp.x = centerLocation.x + (searchDist * cos(temp.theta));
    temp.y = centerLocation.y + (searchDist * sin(temp.theta));

    if(IsWithinBoundary(temp))
    {
      isValid = true;
      hasSearchPoint = true;    //say we have a search point
      returning = false;        //reset our returning variable
      //return temp;
    }
  }
  //pass temp to check method
  //check if result is good
  //if not repeat loop
  //if good return temp
  
  return temp;
}

/*
 * Generates various search patterns for testing
 * currently experiment with octagons.
 */

//GENDELIBERATEPOINT is a method that is called when the rover is in the process of doing a fixed pattern search.  Currently implements an octagonal pattern centered around the
//randomly generated searchpoint
Point SearchController::GenDeliberatePoint()
{
  Point temp;

 cout << "In GenDelPoint" << endl;

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

  cout << "In InterruptedLogic" << endl;

  Point temp;

  //Check Attempts
  //---------------------------------------------------------------------------------------

  bool exceededAttempts = CheckAttemptStatus();

  //If rover has failed consistantly 3 times to get anywhere (2nd CHECK)
  if(numTimesExceeded > num_of_tries)
  {
    //JUST GO HOME!               Reset all variables
    cout << "Exceeded Attempts 3 times in a row, going back to center" << endl;


    returning = true;             //trigger go back to center behavior
    hasSearchPoint = false;       //trigger go back to center behavior
    positionInSearch = 0;         //reset position value to 0;
    temp = centerLocation;        //tell rover to go home

    return temp;
  }

  //---------------------------------------------------------------------------------------
  //BEGIN LOGIC
  //---------------------------------------------------------------------------------------

  //if there is no current valid search point
  if(!hasSearchPoint)
  {
    //if not returning from a search pattern rover shouldn't be seeing center
    //if it does, hasSearchPoint will be false and returning will be false
    if(!returning)
    {
      //DRIVING THROUGH CENTER

      cout << "Interrupted by seeing center, assuming driving through center.  Forcing a new Search Point to be Generated" << endl;
      searchLocation = ChooseRandomPoint();        //create new search point
      temp = searchLocation;                       //set as new point to try to acheive
    }

    //Rover is trying to go back to the center location
    else
    {
      //DRIVING HOME

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
  temp.y = currentLocation.y + (0.5 * sin(temp.theta));  

  return temp;
}

/*
 * Sets up the boarder base on the initial theta robot faces after startup.
 * 1) Get the current inital theta as a positive value.
 * 2) Define the float angle constants for radian values reference.
 * 3) Initialize the quadrant rover is placed in based on the initial angle.
 * 4) Establish the direction the border is drawn and boundary offset base
 *    on rover's quadrant by setting useX boolean and boundary_distance float
 */
void SearchController::SetBoarderValues()
{

  //Get Current Rovers Theta
  //---------------------------------------------------------------------------------------

  float initTheta = angles::normalize_angle_positive(currentLocation.theta);

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
//    cout << "I AM IN QUADRANT 1:  "  << initTheta << endl;
    quadrant = 1;
  }
  else if(initTheta <= (PI3_4))
  {
//    cout << "I AM IN QUADRANT 2:  " << initTheta << endl;
    quadrant = 2;
  }
  else if(initTheta <= (PI5_4))
  {
//    cout << "I AM IN QUADRANT 3:  " << initTheta << endl;
    quadrant = 3;
  }
  else if (initTheta <= PI7_4)
  {
//    cout << "I AM IN QUADRANT 4:  " << initTheta << endl;
    quadrant = 4;
  }
  else
  {
//    cout << "I AM IN QUADRANT 1:  "  << initTheta << endl;
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
      boundary_distance = 1;
      break;
    case 4:
      useX = false;
      boundary_distance = 1;
    default:
      break;
  }
}

/*
 * Validates if the searchPoint given is valid in the x and y . 
 * 1) Verify which orientation X or Y is the boundary drawn for given robot with searchPoint.
 * 2) Calculates the boundary limit point base on boolean useX.
 * 3) Checks of the boundary_distance is positve or negative.
 * 4) Checks of the searchPoint.x or y exceeds limit
 * 5) Sets boolean valid if the searchPoint is valid and return boolean.
 */
bool SearchController::IsWithinBoundary(Point searchPoint)
{
    bool valid = false;
//    cout << "Current Point X:  " << searchPoint.x << "  Current Point Y:  " << searchPoint.y << endl;
//    cout << "Boundary DISTANCE: " << boundary_distance << endl;
      
    if(useX)
    {
     
      float checkX = centerLocation.x + boundary_distance;
//      cout << "Can't go past on X: " << checkX << endl;

      if(boundary_distance < 0)
      {
        if(searchPoint.x < checkX)
        {
//          cout << "NOT A VALID POINT" << endl;
          valid = false;
        }
        else
        {
//          cout << "VALID POINT" << endl;
          valid = true;
        }
      }

      else if(boundary_distance > 0)
      {
        if(searchPoint.x > checkX)
        {
//          cout << "NOT A VALID POINT" << endl;
          valid = false;
        }
        else
        {
//          cout << "VALID POINT" << endl;
          valid = true;
        }
      }
    }

    else
    {
      float checkY = centerLocation.y + boundary_distance;
//      cout << "Can't go past on Y: " << checkY << endl;

      if(boundary_distance < 0)
      {
        if(searchPoint.y < checkY)
        {
//          cout << "NOT A VALID POINT" << endl;
          valid = false;
        }
        else
        {
//          cout << "VALID POINT" << endl;
          valid = true;
        }
      }

      else if(boundary_distance > 0)
      {
        if(searchPoint.y > checkY)
        {
//          cout << "NOT A VALID POINT" << endl;
          valid = false;
        }
        else
        {
//          cout << "VALID POINT" << endl;
          valid = true;
        }
      }
    }

  return valid;
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

void SearchController::PrintStatusToLog()
{
  string s;

  if(hasSearchPoint)
  {
    s = "true";
  }
  else
  {
      s = "false";
  }

  cout << "Search Point = " << s <<  endl;

  if(returning)
  {
    s = "true";
  }
  else
  {
      s = "false";
  }

  cout << "returning = " << s <<  endl;

  if(succesfullPickup)
  {
    s = "true";
  }
  else
  {
      s = "false";
  }

  cout << "successfullpickup = " << s <<  endl;

  if(wasInterrupted)
  {
    s = "true";
  }
  else
  {
      s = "false";
  }

  cout << "wasInterrupted = " << s <<  endl;

  if(result.wpts.waypoints.size() == 0)
  {
      s = "is empty ";
  }
  else
  {
      s = "is not empty ";
  }

  cout << "waypoints list " << s << endl;

}

bool SearchController::CheckAttemptStatus()
{
  //CHECK ATTEMPTS

  attemptCount++;
  cout << "Avoided Obstacle; Attempt #" << attemptCount << endl;

  //if we have exceeded our limits of attempts (1st CHECK)
  if(attemptCount > num_of_tries)
  {
    cout << "Exceeded # Attempts" << endl;
    numTimesExceeded++;             //iterate times exceeded
    attemptCount = 0;               //RESET attempt count

    return true;                    //trigger exceeded behavior
  }

  return false;
}

Point SearchController::FinishedDropOffLogic()
{

}
