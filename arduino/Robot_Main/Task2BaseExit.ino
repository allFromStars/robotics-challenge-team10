// ============================================================
// Task 2 front phase: base exit -> airlock -> ramp entry
// ============================================================

enum Task2BaseExitState {
  T2B_IDLE,
  T2B_LEAVE_DEPLOYMENT_AREA,
  T2B_FOLLOW_DEPLOYMENT_LINE,
  T2B_TURN_RIGHT_TO_EXIT_ROUTE,
  T2B_REACQUIRE_EXIT_ROUTE,
  T2B_FOLLOW_EXIT_ROUTE_STRAIGHT,
  T2B_TURN_LEFT_TO_B_LINE,
  T2B_REACQUIRE_B_LINE,
  T2B_APPROACH_B_TAG,
  T2B_REQUEST_AIRLOCK_A,
  T2B_WAIT_AIRLOCK_A,
  T2B_FOLLOW_AFTER_B_TAG_TO_JUNCTION,
  T2B_TURN_LEFT_AFTER_B_TAG,
  T2B_REACQUIRE_AFTER_B_TAG_LEFT,
  T2B_FOLLOW_TO_SECOND_JUNCTION,
  T2B_TURN_RIGHT_AFTER_SECOND_JUNCTION,
  T2B_REACQUIRE_AFTER_SECOND_RIGHT,
  T2B_FINAL_FOLLOW_TEMP_STOP,
  T2B_RAMP_CLIMB,
  T2B_DONE,
  T2B_FAILSAFE
};

Task2BaseExitState task2BaseExitState = T2B_IDLE;

const int T2B_SPEED_CAREFUL = 280;
const int T2B_SPEED_LINE = 320;
const int T2B_SPEED_FAST = 400;

const float T2B_RIGHT_TURN_DEG = -90.0;
const float T2B_LEFT_TURN_DEG = 90.0;

const unsigned long T2B_LEAVE_DEPLOYMENT_MIN_MS = 200;
const unsigned long T2B_JUNCTION_IGNORE_MS = 800;
const unsigned long T2B_MAX_STRAIGHT_MS = 15000;
const unsigned long T2B_REACQUIRE_TIMEOUT_MS = 2500;
const unsigned long T2B_LOST_LINE_TIMEOUT_MS = 1200;
const unsigned long T2B_TOTAL_TIMEOUT_MS = 100000;
const unsigned long T2B_FINAL_FOLLOW_MS = 3000;
const unsigned long T2B_SERVER_REPLY_TIMEOUT_MS = 5000;
const unsigned long T2B_SERVER_REQUEST_RETRY_MS = 1500;

const int T2B_BASE_JUNCTION_ACTIVE_COUNT = 5;
const unsigned long T2B_BASE_JUNCTION_STABLE_MS = 100;
const int T2B_SECOND_JUNCTION_ACTIVE_COUNT = 5;
const unsigned long T2B_SECOND_JUNCTION_STABLE_MS = 80;

const int T2B_REACQUIRE_MAX_ABS_ERROR = 1800;
const int T2B_REACQUIRE_CENTER_MAX_ABS_ERROR = 1200;
const unsigned long T2B_REACQUIRE_CENTER_STABLE_MS = 250;
const unsigned long T2B_POST_B_REACQUIRE_STRAIGHT_MS = 350;
const unsigned long T2B_POST_B_REACQUIRE_SWEEP_MS = 450;
const int T2B_POST_B_REACQUIRE_INNER_SPEED = 150;
const int T2B_POST_B_REACQUIRE_OUTER_SPEED = 300;
const unsigned long T2B_TURN_IGNORE_LINE_MS = 600;
const float T2B_TURN_MIN_EARLY_STOP_DEG = 90.0;

unsigned long task2BaseStateStartMs = 0;
unsigned long task2BaseRunStartMs = 0;
unsigned long task2BaseJunctionFirstSeenMs = 0;
unsigned long task2SecondJunctionFirstSeenMs = 0;
unsigned long task2LineLostStartMs = 0;
unsigned long task2ReacquireCenteredStartMs = 0;
unsigned long task2ServerRequestStartedMs = 0;
unsigned long task2LastServerRequestMs = 0;
uint32_t task2ActiveAirlockTag = 0;
float task2TurnStartYaw = 0.0;

bool task2BaseJunctionDetected() {
  return lineDetected && activeSensorCount >= T2B_BASE_JUNCTION_ACTIVE_COUNT;
}

bool task2BaseJunctionStable() {
  if (!task2BaseJunctionDetected()) {
    task2BaseJunctionFirstSeenMs = 0;
    return false;
  }

  if (task2BaseJunctionFirstSeenMs == 0) {
    task2BaseJunctionFirstSeenMs = millis();
    return false;
  }

  return millis() - task2BaseJunctionFirstSeenMs >= T2B_BASE_JUNCTION_STABLE_MS;
}

bool task2SecondJunctionStable() {
  bool detected = lineDetected && activeSensorCount >= T2B_SECOND_JUNCTION_ACTIVE_COUNT;

  if (!detected) {
    task2SecondJunctionFirstSeenMs = 0;
    return false;
  }

  if (task2SecondJunctionFirstSeenMs == 0) {
    task2SecondJunctionFirstSeenMs = millis();
    return false;
  }

  return millis() - task2SecondJunctionFirstSeenMs >= T2B_SECOND_JUNCTION_STABLE_MS;
}

void task2TurnInPlace(int direction, int speed) {
  if (direction < 0) {
    driveMotors(speed, -speed);
  } else {
    driveMotors(-speed, speed);
  }
}

void task2FollowLinePD(int baseSpeed) {
  if (!lineDetected) {
    if (lastError < 0) {
      task2TurnInPlace(-1, T2B_SPEED_LINE);
    } else {
      task2TurnInPlace(1, T2B_SPEED_LINE);
    }
    return;
  }

  followGridLinePD(baseSpeed);
}

bool task2LineReacquired() {
  int currentError = (int)lastLinePosition - LINE_CENTER;

  return lineDetected &&
         !task2BaseJunctionDetected() &&
         abs(currentError) <= T2B_REACQUIRE_MAX_ABS_ERROR &&
         activeSensorCount >= 1 &&
         activeSensorCount <= 5;
}

bool task2LineCenteredAfterReacquire() {
  bool centered = lineDetected &&
                  !task2BaseJunctionDetected() &&
                  abs(lastError) <= T2B_REACQUIRE_CENTER_MAX_ABS_ERROR &&
                  activeSensorCount >= 1 &&
                  activeSensorCount <= 5;

  if (!centered) {
    task2ReacquireCenteredStartMs = 0;
    return false;
  }

  if (task2ReacquireCenteredStartMs == 0) {
    task2ReacquireCenteredStartMs = millis();
    return false;
  }

  return millis() - task2ReacquireCenteredStartMs >= T2B_REACQUIRE_CENTER_STABLE_MS;
}

bool task2ValidLineForEarlyTurnStop(unsigned long ignoreLineMs) {
  return millis() - task2BaseStateStartMs > ignoreLineMs &&
         lineDetected &&
         !task2BaseJunctionDetected() &&
         activeSensorCount >= 1 &&
         activeSensorCount <= 5;
}

bool task2TurnPassedAngle(float minAngleDeg) {
  return abs(sensors.yaw - task2TurnStartYaw) >= minAngleDeg;
}

void task2StartTurn(float angleDeg) {
  task2TurnStartYaw = sensors.yaw;
  startTurnAngle(angleDeg);
}

void task2SearchForPostBTurnLine(bool sweepLeftFirst) {
  unsigned long elapsedMs = millis() - task2BaseStateStartMs;

  if (elapsedMs < T2B_POST_B_REACQUIRE_STRAIGHT_MS) {
    driveForwardMagnitudes(T2B_SPEED_CAREFUL, T2B_SPEED_CAREFUL);
    return;
  }

  unsigned long phase = (elapsedMs - T2B_POST_B_REACQUIRE_STRAIGHT_MS) / T2B_POST_B_REACQUIRE_SWEEP_MS;
  bool sweepLeft = ((phase % 2) == 0) == sweepLeftFirst;

  if (sweepLeft) {
    driveForwardMagnitudes(T2B_POST_B_REACQUIRE_INNER_SPEED, T2B_POST_B_REACQUIRE_OUTER_SPEED);
  } else {
    driveForwardMagnitudes(T2B_POST_B_REACQUIRE_OUTER_SPEED, T2B_POST_B_REACQUIRE_INNER_SPEED);
  }
}

void enterTask2BaseState(int nextState) {
  task2BaseExitState = (Task2BaseExitState)nextState;
  task2BaseStateStartMs = millis();
  task2BaseJunctionFirstSeenMs = 0;
  task2SecondJunctionFirstSeenMs = 0;
  task2LineLostStartMs = 0;
  task2ReacquireCenteredStartMs = 0;

  if (nextState == T2B_TURN_RIGHT_TO_EXIT_ROUTE ||
      nextState == T2B_TURN_LEFT_TO_B_LINE ||
      nextState == T2B_TURN_LEFT_AFTER_B_TAG ||
      nextState == T2B_TURN_RIGHT_AFTER_SECOND_JUNCTION) {
    lastError = 0;
  }
}

void task2BaseFailsafe(const char* reason) {
  stopMotors();
//   Serial.print("[TASK2] FAILSAFE: ");
  // Serial.println(reason);
  enterTask2BaseState(T2B_FAILSAFE);
}

void startTask2BaseExit() {
  stopMotors();
  sensors.yaw = 0.0;
  lastTimeMicros = micros();
  lastError = 0;
  lastLinePosition = LINE_CENTER;
  task2ActiveAirlockTag = 0;
  task2ServerRequestStartedMs = 0;
  task2LastServerRequestMs = 0;
  task2BaseRunStartMs = millis();
  resetSegmentEncoders();
  enterTask2BaseState(T2B_LEAVE_DEPLOYMENT_AREA);
  // Serial.println("[TASK2] Base exit phase started.");
}

bool task2BaseTimedOut() {
  return task2BaseRunStartMs > 0 && millis() - task2BaseRunStartMs > T2B_TOTAL_TIMEOUT_MS;
}

void task2UpdateLineLostTimer() {
  if (lineDetected) {
    task2LineLostStartMs = 0;
    return;
  }

  if (task2LineLostStartMs == 0) {
    task2LineLostStartMs = millis();
  }
}

bool task2LineLostTooLong() {
  return task2LineLostStartMs > 0 && millis() - task2LineLostStartMs > T2B_LOST_LINE_TIMEOUT_MS;
}

int updateTask2BaseExit() {
  processLinePosition();
  task2UpdateLineLostTimer();

  if (task2BaseTimedOut() &&
      task2BaseExitState != T2B_DONE &&
      task2BaseExitState != T2B_FAILSAFE) {
    task2BaseFailsafe("phase timeout");
    return -1;
  }

  switch (task2BaseExitState) {
    case T2B_IDLE:
      stopMotors();
      return 0;

    case T2B_LEAVE_DEPLOYMENT_AREA:
      driveForwardMagnitudes(T2B_SPEED_CAREFUL, T2B_SPEED_CAREFUL);
      if (lineDetected && millis() - task2BaseStateStartMs > T2B_LEAVE_DEPLOYMENT_MIN_MS) {
        enterTask2BaseState(T2B_FOLLOW_DEPLOYMENT_LINE);
      }
      return 0;

    case T2B_FOLLOW_DEPLOYMENT_LINE:
      task2FollowLinePD(T2B_SPEED_FAST);
      if (task2BaseJunctionStable()) {
        task2StartTurn(T2B_RIGHT_TURN_DEG);
        enterTask2BaseState(T2B_TURN_RIGHT_TO_EXIT_ROUTE);
      } else if (task2LineLostTooLong()) {
        task2BaseFailsafe("line lost on deployment line");
        return -1;
      }
      return 0;

    case T2B_TURN_RIGHT_TO_EXIT_ROUTE:
      if (updateTurnAngle()) {
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterTask2BaseState(T2B_REACQUIRE_EXIT_ROUTE);
      }
      return 0;

    case T2B_REACQUIRE_EXIT_ROUTE:
      task2FollowLinePD(T2B_SPEED_LINE);
      if (task2LineCenteredAfterReacquire()) {
        enterTask2BaseState(T2B_FOLLOW_EXIT_ROUTE_STRAIGHT);
      } else if (millis() - task2BaseStateStartMs > T2B_REACQUIRE_TIMEOUT_MS) {
        task2BaseFailsafe("could not reacquire exit route");
        return -1;
      }
      return 0;

    case T2B_FOLLOW_EXIT_ROUTE_STRAIGHT:
      task2FollowLinePD(T2B_SPEED_LINE);
      if (millis() - task2BaseStateStartMs > T2B_JUNCTION_IGNORE_MS &&
          task2BaseJunctionStable()) {
        task2StartTurn(T2B_LEFT_TURN_DEG);
        enterTask2BaseState(T2B_TURN_LEFT_TO_B_LINE);
      } else if (millis() - task2BaseStateStartMs > T2B_MAX_STRAIGHT_MS) {
        task2BaseFailsafe("could not find left-turn junction");
        return -1;
      } else if (task2LineLostTooLong()) {
        task2BaseFailsafe("line lost on exit route");
        return -1;
      }
      return 0;

    case T2B_TURN_LEFT_TO_B_LINE:
      if (millis() - task2BaseStateStartMs > T2B_TURN_IGNORE_LINE_MS &&
          task2TurnPassedAngle(T2B_TURN_MIN_EARLY_STOP_DEG) &&
          task2ValidLineForEarlyTurnStop(T2B_TURN_IGNORE_LINE_MS)) {
        cancelTurnAngle();
        lastError = 0;
        task2BaseJunctionFirstSeenMs = 0;
        enterTask2BaseState(T2B_APPROACH_B_TAG);
        return 0;
      }

      if (updateTurnAngle()) {
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterTask2BaseState(T2B_REACQUIRE_B_LINE);
      }
      return 0;

    case T2B_REACQUIRE_B_LINE:
      driveForwardMagnitudes(T2B_SPEED_CAREFUL, T2B_SPEED_CAREFUL);
      if (task2LineReacquired()) {
        lastError = 0;
        enterTask2BaseState(T2B_APPROACH_B_TAG);
      } else if (millis() - task2BaseStateStartMs > T2B_REACQUIRE_TIMEOUT_MS) {
        task2BaseFailsafe("could not reacquire B line");
        return -1;
      }
      return 0;

    case T2B_APPROACH_B_TAG:
      task2FollowLinePD(T2B_SPEED_CAREFUL);
      if (sensors.rfidInfo != 0) {
        task2ActiveAirlockTag = sensors.rfidInfo;
        task2LastServerRequestMs = 0;
        task2ServerRequestStartedMs = 0;
        enterTask2BaseState(T2B_REQUEST_AIRLOCK_A);
      } else if (task2LineLostTooLong()) {
        task2BaseFailsafe("line lost while approaching RFID");
        return -1;
      }
      return 0;

    case T2B_REQUEST_AIRLOCK_A:
      stopMotors();
      if (task2ActiveAirlockTag == 0) {
        task2BaseFailsafe("RFID detected but tag id was empty");
        return -1;
      }
      if (task2LastServerRequestMs != 0 &&
          millis() - task2LastServerRequestMs < T2B_SERVER_REQUEST_RETRY_MS) {
        return 0;
      }
      task2LastServerRequestMs = millis();
      if (requestOpenAirlock('A', task2ActiveAirlockTag)) {
        task2ServerRequestStartedMs = millis();
        task2LastServerRequestMs = task2ServerRequestStartedMs;
        if (WAIT_FOR_TASK2_AIRLOCK_REPLY) {
          enterTask2BaseState(T2B_WAIT_AIRLOCK_A);
        } else {
          enterTask2BaseState(T2B_FOLLOW_AFTER_B_TAG_TO_JUNCTION);
        }
      }
      return 0;

    case T2B_WAIT_AIRLOCK_A:
      stopMotors();
      if (airlockRequestAccepted()) {
        enterTask2BaseState(T2B_FOLLOW_AFTER_B_TAG_TO_JUNCTION);
      } else if (airlockRequestRejected()) {
        enterTask2BaseState(T2B_REQUEST_AIRLOCK_A);
      } else if (millis() - task2ServerRequestStartedMs >= T2B_SERVER_REPLY_TIMEOUT_MS &&
                 millis() - task2LastServerRequestMs >= T2B_SERVER_REQUEST_RETRY_MS) {
        task2LastServerRequestMs = millis();
        task2ServerRequestStartedMs = millis();
        requestOpenAirlock('A', task2ActiveAirlockTag);
      }
      return 0;

    case T2B_FOLLOW_AFTER_B_TAG_TO_JUNCTION:
      task2FollowLinePD(T2B_SPEED_CAREFUL);
      if (millis() - task2BaseStateStartMs > T2B_JUNCTION_IGNORE_MS &&
          task2BaseJunctionStable()) {
        task2StartTurn(T2B_LEFT_TURN_DEG);
        enterTask2BaseState(T2B_TURN_LEFT_AFTER_B_TAG);
      } else if (millis() - task2BaseStateStartMs > T2B_MAX_STRAIGHT_MS) {
        task2BaseFailsafe("could not find first post-B junction");
        return -1;
      } else if (task2LineLostTooLong()) {
        task2BaseFailsafe("line lost after RFID");
        return -1;
      }
      return 0;

    case T2B_TURN_LEFT_AFTER_B_TAG:
      if (updateTurnAngle()) {
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterTask2BaseState(T2B_REACQUIRE_AFTER_B_TAG_LEFT);
      }
      return 0;

    case T2B_REACQUIRE_AFTER_B_TAG_LEFT:
      task2SearchForPostBTurnLine(true);
      if (task2LineReacquired()) {
        lastError = 0;
        task2BaseJunctionFirstSeenMs = 0;
        enterTask2BaseState(T2B_FOLLOW_TO_SECOND_JUNCTION);
      } else if (millis() - task2BaseStateStartMs > T2B_REACQUIRE_TIMEOUT_MS) {
        task2BaseFailsafe("could not reacquire after post-B left turn");
        return -1;
      }
      return 0;

    case T2B_FOLLOW_TO_SECOND_JUNCTION:
      task2FollowLinePD(T2B_SPEED_CAREFUL);
      if (millis() - task2BaseStateStartMs > T2B_JUNCTION_IGNORE_MS &&
          task2SecondJunctionStable()) {
        task2StartTurn(T2B_RIGHT_TURN_DEG);
        enterTask2BaseState(T2B_TURN_RIGHT_AFTER_SECOND_JUNCTION);
      } else if (millis() - task2BaseStateStartMs > T2B_MAX_STRAIGHT_MS) {
        task2BaseFailsafe("could not find second post-B junction");
        return -1;
      } else if (task2LineLostTooLong()) {
        task2BaseFailsafe("line lost before second junction");
        return -1;
      }
      return 0;

    case T2B_TURN_RIGHT_AFTER_SECOND_JUNCTION:
      if (updateTurnAngle()) {
        lastError = 0;
        lastLinePosition = LINE_CENTER;
        enterTask2BaseState(T2B_REACQUIRE_AFTER_SECOND_RIGHT);
      }
      return 0;

    case T2B_REACQUIRE_AFTER_SECOND_RIGHT:
      task2SearchForPostBTurnLine(false);
      if (task2LineReacquired()) {
        lastError = 0;
        enterTask2BaseState(T2B_FINAL_FOLLOW_TEMP_STOP);
      } else if (millis() - task2BaseStateStartMs > T2B_REACQUIRE_TIMEOUT_MS) {
        task2BaseFailsafe("could not reacquire after second right turn");
        return -1;
      }
      return 0;

    case T2B_FINAL_FOLLOW_TEMP_STOP:
      task2FollowLinePD(T2B_SPEED_CAREFUL);
      if (millis() - task2BaseStateStartMs > T2B_FINAL_FOLLOW_MS) {
        stopMotors();
        startRampWallFollowing();
        enterTask2BaseState(T2B_RAMP_CLIMB);
      } else if (task2LineLostTooLong()) {
        task2BaseFailsafe("line lost during final follow");
        return -1;
      }
      return 0;

    case T2B_RAMP_CLIMB: {
      int rampResult = updateRampWallFollowing(true);
      if (rampResult == 1) {
        enterTask2BaseState(T2B_DONE);
        return 1;
      }
      if (rampResult < 0) {
        task2BaseFailsafe("ramp wall following failed");
        return -1;
      }
      return 0;
    }

    case T2B_DONE:
      stopMotors();
      return 1;

    case T2B_FAILSAFE:
      stopMotors();
      return -1;
  }

  return 0;
}
