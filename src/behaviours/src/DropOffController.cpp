#include "DropOffController.h"

DropOffController::DropOffController() {

  reachedCollectionPoint = false;

  result.type = behavior;
  result.b = wait;
  result.wristAngle = 0.7;
  result.reset = false;
  interrupt = false;

  circularCenterSearching = false;
  spinner = 0;
  centerApproach = false;
  seenEnoughCenterTags = false;
  prevCount = 0;

  countLeft = 0;
  countRight = 0;

  isPrecisionDriving = false;
  startWaypoint = false;
  timerTimeElapsed = -1;

}

DropOffController::~DropOffController() {

}


void DropOffController::FlushController() {
  result.wpts.waypoints.clear();
  result.type = behavior;
  result.b = prevProcess;
  result.reset = true;
}


bool DropOffController::IfShouldGoHome() 
{
  double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);

  if ((distanceToCenter > collectionPointVisualDistance) && targetHeld )
  {
    return true;
  }
  return false;
}

void DropOffController::SetDestinationHome() 
{
  result.type = waypoint;
  result.wpts.waypoints.clear();
  result.wpts.waypoints.push_back(this->centerLocation);
  startWaypoint = false;
  isPrecisionDriving = false;
}

bool DropOffController::IsLost()
{
  if (SeenNest()) { return false;}
  else {return true;}
}


void DropOffController::SetSeenNest(bool val) 
{
  seenNest = val;
}

bool DropOffController::SeenNest()
{
  return seenNest;
}

void DropOffController::SearchForCenter()
{
  Point nextSpinPoint;

  //sets a goal that is 60cm from the centerLocation and spinner
  //radians counterclockwise from being purly along the x-axis.
  nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
  nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
  nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);

  result.type = waypoint;
  result.wpts.waypoints.clear();
  result.wpts.waypoints.push_back(nextSpinPoint);

  spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
  if (spinner > 2*M_PI) 
  {
    spinner -= 2*M_PI;
  }
  spinSizeIncrease += spinSizeIncrement/8;
  // circularCenterSearching = true;

  //safety flag to prevent us trying to drive back to the
  //center since we have a block with us and the above point is
  //greater than collectionPointVisualDistance from the center.

  returnTimer = current_time;
  timerTimeElapsed = 0;

  IncrementSearchTimer();
}

void DropOffController::IncrementSearchTimer()
{
  searchTimer ++;
}

bool DropOffController::IsCentered()
{
  return isCentered;
}

void DropOffController::SetIsCentered(bool val)
{
  isCentered = val;
}

void DropOffController::IncrementNestTimer()
{
  nestTimer ++;
}

void DropOffController::SignalDoneDroppingOff()
{
  finalInterrupt = true;
}

void DropOffController::DropAndLeave()
{
  isPrecisionDriving = true;
  result.type = precisionDriving;
  result.pd.cmdAngularError = 0.0;

  IncrementNestTimer();

  // Check if time to drop. If drop then leave. Else keep driving in.
  if (nestTimer > nestWalkThreshold) 
  {
     // Drop block
    result.fingerAngle = M_PI_2; //open fingers
    result.wristAngle = 0; //raise wrist
    // targetHeld = false;
    cout << "Dropped Target" << endl;
    
    // Set robot to leave
    result.pd.cmdVel = -nestVelocity;

    // Check if we have left by now
    if (nestTimer > 2.5*nestWalkThreshold)
    {
      SignalDoneDroppingOff();
    }
  } 

  // Otherwise keep moving forward
  else 
  {
    cout << ">>> Moving into nest" << endl;
    result.pd.cmdVel = nestVelocity;
  }
}

void DropOffController::CenterRover()
{
  float turnDirection = 1;
  isPrecisionDriving = true;
  result.type = precisionDriving;
  centeringTimer ++;
  cout << "centering timer: " << centeringTimer << endl;

  cout << ">>>> Num of blocks. Left: " << countLeft << "\tRight: " << countRight << endl;
  
  // Too many blocks on left
  if (countLeft > (countRight+countThreshold))
  {
    result.pd.cmdVel = -0.1 * turnDirection;
    result.pd.cmdAngularError = centeringTurnRate*turnDirection;
  }

  // Too many blocks on right
  else if (countRight > (countLeft + countThreshold))
  {
    result.pd.cmdVel = -0.1 * turnDirection;
    result.pd.cmdAngularError = -centeringTurnRate*turnDirection;
  }


  // It's okay if there are slightly more blocks on left or rights
  else if ((countLeft <= countRight) && (countRight <= (countLeft + countThreshold)))
  {
    cout << ">>>> centered by block count" << endl;
    SetIsCentered(true);
  }

  // Make sure we don't get stuck trying to center.
  if (centeringTimer > centeringThreshold)
  {
    cout << ">>>>> centered by timer timing out" << endl;
    SetIsCentered(true);
  }

}

void DropOffController::SignalToResetRover() 
{
  // Reset the gripper
  result.fingerAngle = M_PI_2; //open fingers
  result.wristAngle = 0; //raise wrist

  // Set our destination to home
  result.type = waypoint;
  result.wpts.waypoints.clear();
  result.wpts.waypoints.push_back(this->centerLocation);
  startWaypoint = false;
  isPrecisionDriving = false;

  // Signal Final Interrupt
  finalInterrupt = true;
}

/**
 * DropOffController is tasked with setting the waypoint of where to drop off the object
 * tasked to drive onto the collection zone (the nest)
 * drop the block inside of the nest
 * search for the nest if the nest is not found
*/
Result DropOffController::DoWork() 
{
  cout << ">>>>>>>>> DROP OFF CONTROLLER >>>>>>>>>>" << endl;
  
  if (finalInterrupt)
  {
    cout << ">>>> Flushing Controller" << endl;
    FlushController();
    return result;
  }

  
  if (SeenNest()) 
  {
    cout << ">>>> Seen Nest" << endl;
    result.wpts.waypoints.clear();
    startWaypoint = false;
    if (IsCentered())
    {
      cout << ">>>> Dropping and Leaving" << endl;
      DropAndLeave();
      // return result;
    }
    else
    {
      cout << ">>>> Centering Rover" << endl;
      CenterRover();
    }
    cout << "[before] result.type: " << result.type << endl;
    cout << "[before] result.b: " << result.b << endl;
    isPrecisionDriving = true;
    result.type = precisionDriving;
    result.b = nextProcess;
    cout << "[after] result.type: " << result.type << endl;
    cout << "[after] result.b: " << result.b << endl;
    return result;
  }

  else if (IsLost() && !IfShouldGoHome()) 
  {
    cout << ">>>> Is Lost. Searching for nest" << endl;
    SearchForCenter();
    if (searchTimer > searchThreshold) {
      cout << ">>>> Giving up Search" << endl;
      SignalToResetRover();
    }
    return result;
  }

  else if(IfShouldGoHome())
  {
    cout << ">>>> Setting SetDestinationHome" << endl;
    SetDestinationHome();
    return result;
  }

  cout << "***************** uh oh. Should not get here." << endl;
  return result;
}

void DropOffController::Reset() {
  // George Begin
  seenNest = false;
  isCentered = false;

  nestTimer = -1;
  // George End

  result.type = behavior;
  result.b = wait;
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError = 0;
  result.fingerAngle = -1;
  result.wristAngle = 0.7;
  result.reset = false;
  result.wpts.waypoints.clear();
  spinner = 0;
  spinSizeIncrease = 0;
  prevCount = 0;
  timerTimeElapsed = -1;

  countLeft = 0;
  countRight = 0;


  //reset flags
  reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  circularCenterSearching = false;
  isPrecisionDriving = false;
  finalInterrupt = false;
  precisionInterrupt = false;
  targetHeld = false;
  startWaypoint = false;
  first_center = true;
  cout << "Finished reseting DropOffController variables." << endl;

}

void DropOffController::SetTargetData(vector<Tag> tags) {
  countRight = 0;
  countLeft = 0;


  if(targetHeld) {
    // if a target is detected and we are looking for center tags
    if (tags.size() > 0 && !reachedCollectionPoint) {

      // this loop is to get the number of center tags
      for (int i = 0; i < tags.size(); i++) {
        if (tags[i].getID() == 256) {

          // checks if tag is on the right or left side of the image
          if (tags[i].getPositionX() + cameraOffsetCorrection > 0) {
            countRight++;

          } else {
            countLeft++;
          }
        }
      }
    }

    if (countLeft + countRight > nestTagThreshold)
    {
      SetSeenNest(true);
    }
  }

}

void DropOffController::ProcessData() {
  if((countLeft + countRight) > 0) {
    isPrecisionDriving = true;
  } else {
    startWaypoint = true;
  }
}

// ShouldInterrupt should return true if we would like the drop off controller to do something
bool DropOffController::ShouldInterrupt() {
  ProcessData();
  if (startWaypoint && !interrupt) {
    interrupt = true;
    precisionInterrupt = false;
    return true;
  }
  else if (isPrecisionDriving && !precisionInterrupt) {
    precisionInterrupt = true;
    return true;
  }

  if (SeenNest()) {
    return true;
  }

  if (finalInterrupt) {
    return true;
  }
}

bool DropOffController::HasWork() {
  // if(timerTimeElapsed > -1) {
  //   long int elapsed = current_time - returnTimer;
  //   timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  // }

  // if (circularCenterSearching && timerTimeElapsed < 2 && !isPrecisionDriving) {
  //   return false;
  // }


  return ((startWaypoint || isPrecisionDriving));
}

bool DropOffController::IsChangingMode() {
  return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
  centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
  currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
  targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
  targetHeld = targetHeld || blockBlock;
}

void DropOffController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}