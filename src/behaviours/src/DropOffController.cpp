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

bool DropOffController::IsAtNest() {
  if (atNest == true) return true;

  int numBlocks = countLeft + countRight;
  cout << ">>>>>>>>>>>>>>>>> NUMBLOCKS: " << numBlocks << endl;
  if ((numBlocks > centerTagThreshold) || (atNest == true)) 
  {
    cout << ">>>>>>>> STATES THAT WE ARE AT NEST " << endl;
    atNest = true;
    reachedCollectionPoint = true;
  } 
  return atNest;
}

void DropOffController::DropAndLeave() {
  
  // Has the rover moved into the nest enough? 
  //  - If so drop off the box and back out
  //  - Else keep moving into the circle
  isPrecisionDriving = true;
  result.type = precisionDriving;
  result.pd.cmdAngularError = 0.0;
  
  // Check if time to drop. If drop then leave. Else keep driving in.
  if (nestTimer > nestWalkThreshold) 
  {
     // Drop block
    result.fingerAngle = M_PI_2; //open fingers
    result.wristAngle = 0; //raise wrist
    // targetHeld = false;
    cout << "Dropped Target" << endl;
    // Set robot to leave
    result.pd.cmdVel = -0.3;
  } 
  else 
  {
    result.pd.cmdVel = nestVelocity;
  }
}

void DropOffController::CenterRobot() {
  cout << "+++ Centering Robot" << endl;
  // cout << "right: " << right << "\tleft: " << left << endl;
  // result.pd.cmdAngularError = -1;

  isPrecisionDriving = true;
  result.type = precisionDriving;

  if (countLeft > countRight) 
  {
    result.pd.cmdAngular = -K_angular/2;
  }
  else if (countRight > countLeft)
  {
    result.pd.cmdAngular = K_angular/2; 
  }

  result.pd.setPointVel = 0.0;
  result.pd.cmdVel = 0.0;
  result.pd.setPointYaw = 0;
}

// TODO: Logic to check for centering
bool DropOffController::isCentered() {
  // return isCentered;
  return true;
}

void DropOffController::SearchForNest() {
  Point nextSpinPoint;
  isSearching = true;

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
  circularCenterSearching = true;
  //safety flag to prevent us trying to drive back to the
  //center since we have a block with us and the above point is
  //greater than collectionPointVisualDistance from the center.

  returnTimer = current_time;
  timerTimeElapsed = 0;
}

void DropOffController::FlushController() {
  result.wpts.waypoints.clear();
  // isPrecisionDriving = false;
  // startWaypoint = false;
  // finalInterrupt = true;
  // targetHeld = false;
  // reachedCollectionPoint = true;
  result.type = behavior;
  // result.b = nextProcess;
  result.b = prevProcess;
  result.reset = true;
}

bool DropOffController::ShouldGoBackHome() {
  cout << ">>>>>>>>> Should go back?" << endl;
  double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);
  if ((distanceToCenter > collectionPointVisualDistance) && timerTimeElapsed < 0 && targetHeld )
  {
    cout << ">>>>>>>>> yes" << endl;
    return true;
  }
  cout << ">>>>>>>>> no" << endl;
  return false;
}

void DropOffController::SetDestinationNest() {
  result.type = waypoint;
  result.wpts.waypoints.clear();
  result.wpts.waypoints.push_back(this->centerLocation);
  startWaypoint = false;
  isPrecisionDriving = false;

  // Start timer to keep track how long we've been heading home
  timerTimeElapsed = 0;
}

void DropOffController::RunNestTimer() {
  // Start nest timer if not already
  if (nestTimer < 0) 
  {
    nestTimer = 0;
    // nestTimeStamp = timerTimeElapsed;
  }
  else 
  {
    cout << ">>>>>> Increment nest timer" << endl;
    cout << ">>>>>> before nestTimer: " << nestTimer << endl;
    // Otherwise increment the timer
    nestTimer ++;
    // nestTimer = timerTimeElapsed; // Convert from milliseconds to seconds
    cout << ">>>>>> after nestTimer: " << nestTimer << endl;
  } 
}

void DropOffController::CheckIfLeftNest() {
  cout << ">>>>>>>>>>>>>>>>>>. nestTimer: " << nestTimer << endl;
  if (nestTimer > 60) 
  {
    finalInterrupt = true;
  }
}

void DropOffController::ReturnToCenter() {
  reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  centerApproach = false;

  result.type = waypoint;
  result.wpts.waypoints.push_back(this->centerLocation);
  if (isPrecisionDriving) 
  {
    result.type = behavior;
    result.b = prevProcess;
    result.reset = false;
  }
  isPrecisionDriving = false;
  interrupt = false;
  precisionInterrupt = false;
}

/**
 * DropOffController is tasked with setting the waypoint of where to drop off the object
 * tasked to drive onto the collection zone (the nest)
 * drop the block inside of the nest
 * search for the nest if the nest is not found
*/
Result DropOffController::DoWork() 
{
  bool runDefault = false;
  cout << ">>>>>>>>>>>>>>>>>>>> DropOffController" << endl;
  cout << ">>>>>>>>>>>>>>>>>>>> timerTimeElapsed: " << timerTimeElapsed << endl;

  // This if statement is just for debugging and is purposly out of formatting order  
  if (!runDefault)
  {
  // Check if we should still be doing work
  if (finalInterrupt) 
  {
    cout << "Flushing Controller" << endl;
    FlushController();
    return result;
  }

  // LogicController will ask, "where do I go to drop off"
  // DropOffController responds
  //  - Have I started my drop off?
  //    * If I haven't set drop off point
  //    * Else (If I have started my drop off) check to see if we are lost

  if (ShouldGoBackHome()) 
  {
    cout << ">>>>>>>>>>>> setting SetDestinationNest" << endl;
    SetDestinationNest();
    return result;
  } 
  else if (IsAtNest()) 
  {
    cout << ">>>>>>>>>>>> IS AT NEST" << endl;
    if (isCentered()) 
    {
      cout << ">>>>>>>>>>>>> isCentered. Should run, drop, and backout" << endl;
      RunNestTimer();
      DropAndLeave();
      CheckIfLeftNest();
    } 
    else 
    {
      CenterRobot();
    }

    return result;

  } 
  else if ((timerTimeElapsed > maxDropOffTime) || isSearching) 
  {
    SearchForNest();
    // if (timerTimeElapsed > 2*maxDropOffTime) {
    //   finalInterrupt = true;
    //   ReturnToCenter();
    // }
  }

  return result;
  }


  // DEFAULT CODE
  if (runDefault) { 
  cout << "8" << endl;

  int count = countLeft + countRight;

  if(timerTimeElapsed > -1) {

    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
  //to resart our search.
  if(reachedCollectionPoint)
  {
    cout << "2" << endl;
    if (timerTimeElapsed >= 5)
    {
      if (finalInterrupt)
      {
        result.type = behavior;
        result.b = nextProcess;
        result.reset = true;
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>> FINAL INTERRUPT >>>>>>>>>>>>>" << endl;
        return result;
      }
      else
      {
        finalInterrupt = true;
        cout << "1" << endl;
      }
    }
    else if (timerTimeElapsed >= 0.1)
    {
      cout << ">>>>>>>>>>>> OPENING WRISTS TO DROP OFF TARGET >>>>>>>>>" << endl;
      isPrecisionDriving = true;
      result.type = precisionDriving;

      result.fingerAngle = M_PI_2; //open fingers
      result.wristAngle = 0; //raise wrist

      result.pd.cmdVel = -0.3;
      result.pd.cmdAngularError = 0.0;
    }

    return result;
  }

  double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);

  //check to see if we are driving to the center location or if we need to drive in a circle and look.
  if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && (count == 0)) {

    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(this->centerLocation);
    startWaypoint = false;
    isPrecisionDriving = false;

    timerTimeElapsed = 0;

    return result;

  }
  else if (timerTimeElapsed >= 2)//spin search for center
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
    if (spinner > 2*M_PI) {
      spinner -= 2*M_PI;
    }
    spinSizeIncrease += spinSizeIncrement/8;
    circularCenterSearching = true;
    //safety flag to prevent us trying to drive back to the
    //center since we have a block with us and the above point is
    //greater than collectionPointVisualDistance from the center.

    returnTimer = current_time;
    timerTimeElapsed = 0;

  }

  bool left = (countLeft > 0);
  bool right = (countRight > 0);
  bool centerSeen = (right || left);

  //reset lastCenterTagThresholdTime timout timer to current time
  if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {

    lastCenterTagThresholdTime = current_time;

  }

  if (count > 0 || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
  {

    cout << "9" << endl;
    centerSeen = true;

    if (first_center && isPrecisionDriving)
    {
      first_center = false;
      result.type = behavior;
      result.reset = false;
      result.b = nextProcess;
      return result;
    }
    isPrecisionDriving = true;

    if (seenEnoughCenterTags) //if we have seen enough tags
    {
      if ((countLeft-5) > countRight) //and there are too many on the left
      {
        right = false; //then we say none on the right to cause us to turn right
      }
      else if ((countRight-5) > countLeft)
      {
        left = false; //or left in this case
      }
    }

    float turnDirection = 1;
    //reverse tag rejection when we have seen enough tags that we are on a
    //trajectory in to the square we dont want to follow an edge.
    if (seenEnoughCenterTags) turnDirection = -3;

    result.type = precisionDriving;

    //otherwise turn till tags on both sides of image then drive straight
    if (left && right) {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }
    else if (right) {
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = -centeringTurnRate*turnDirection;
    }
    else if (left){
      result.pd.cmdVel = -0.1 * turnDirection;
      result.pd.cmdAngularError = centeringTurnRate*turnDirection;
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }

    //must see greater than this many tags before assuming we are driving into the center and not along an edge.
    if (count > centerTagThreshold)
    {
      seenEnoughCenterTags = true; //we have driven far enough forward to be in and aligned with the circle.
      lastCenterTagThresholdTime = current_time;
    }
    if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
    {
      lastCenterTagThresholdTime = current_time;
    }
    //time since we dropped below countGuard tags
    long int elapsed = current_time - lastCenterTagThresholdTime;
    float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds

    //we have driven far enough forward to have passed over the circle.
    if (count < 1 && seenEnoughCenterTags && timeSinceSeeingEnoughCenterTags > dropDelay) {
      centerSeen = false;
    }
    centerApproach = true;
    prevCount = count;
    count = 0;
    countLeft = 0;
    countRight = 0;
  }

  //was on approach to center and did not seenEnoughCenterTags
  //for lostCenterCutoff seconds so reset.
  else if (centerApproach) {

    long int elapsed = current_time - lastCenterTagThresholdTime;
    float timeSinceSeeingEnoughCenterTags = elapsed/1e3; // Convert from milliseconds to seconds
    if (timeSinceSeeingEnoughCenterTags > lostCenterCutoff)
    {
      cout << "4" << endl;
      //go back to drive to center base location instead of drop off attempt
      reachedCollectionPoint = false;
      seenEnoughCenterTags = false;
      centerApproach = false;

      result.type = waypoint;
      result.wpts.waypoints.push_back(this->centerLocation);
      if (isPrecisionDriving) {
        result.type = behavior;
        result.b = prevProcess;
        result.reset = false;
      }
      isPrecisionDriving = false;
      interrupt = false;
      precisionInterrupt = false;
    }
    else
    {
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0.0;
    }

    return result;

  }

  if (!centerSeen && seenEnoughCenterTags)
  {
    reachedCollectionPoint = true;
    centerApproach = false;
    returnTimer = current_time;
  }
  } //end default
  return result;
}

void DropOffController::Reset() {
  // George Begin
  atNest = false;
  nestTimer = -1;
  nestTimeStamp = -1;
  isSearching = false;
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
        } else {
          cout << "not 256 instead: " << tags[i].getID() << endl;
        }
      }
    }
  }
  cout << ">>>>>> counting tags, left: " << countLeft << "\tright: " << countRight << endl;

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
  if (finalInterrupt) {
    return true;
  }
}

bool DropOffController::HasWork() {
  if(timerTimeElapsed > -1) {
    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  // if (circularCenterSearching && timerTimeElapsed < 2 && !isPrecisionDriving) {
  //   return false;
  // }


  return ((startWaypoint || isPrecisionDriving));
}

bool DropOffController::IsChangingMode() {
  cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> isPrecisionDriving" << endl;
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