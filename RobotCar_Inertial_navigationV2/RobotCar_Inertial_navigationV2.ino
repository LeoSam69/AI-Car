#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include "MecanumCar_v2.h"
#include "ICM20600.h"
#include "AK09918.h"

// =====================================================
// Hardware & Navigation Constants
// =====================================================
const int EchoPin = 13;
const int TrigPin = 12;
const int ServoPin = 9;

Servo servoSensor;
AK09918 mag;
ICM20600 icm20600(true);
mecanumCar mecanumCar(3, 2);

// Calibrated open-loop forward speeds at baseSpeed (cm/s)
// Raw mecanum forward: 108 cm in 3 s -> 36.0 cm/s
// Waypoint (0, 100) measured 97 cm -> tuned slightly lower so it reaches closer to 100 cm
const float RAW_CM_PER_SECOND = 36.0f;
const float WAYPOINT_CM_PER_SECOND = 35.0f;

// Chassis dimensions & clearance parameters (cm)
// Looking from the back: 16.0 cm base width + 5.0 cm extra width on the left side, 20.0 cm length
const float CAR_WIDTH = 16.0f;
const float CAR_LENGTH = 20.0f;
const float LEFT_EXTRA_WIDTH_CM = 5.0f;
const float LEFT_HALF_WIDTH_CM = (CAR_WIDTH * 0.5f) + LEFT_EXTRA_WIDTH_CM; // 13.0 cm
const float RIGHT_HALF_WIDTH_CM = CAR_WIDTH * 0.5f;                        //  8.0 cm
const float PASSING_MARGIN_CM = 6.0f;
const float CHASSIS_BUFFER_CM = 4.0f;

// Extra turn & clearance for the +5 cm left-side protrusion
const float LEFT_EXTRA_TURN_DEG = 12.0f;
const float LEFT_TURN_CLEARANCE_PAD_CM = 6.0f;
const float MAX_TURN_ANGLE_DEG = 115.0f;
const float EXTRA_NUDGE_TURN_DEG = 14.0f;
const int MAX_PREDRIVE_NUDGES = 3;

// Quick pre-drive clearance check (further left for the +5 cm extra width)
const float PRE_DRIVE_CLEARANCE_CM = 18.0f;
const int SERVO_SLIGHT_LEFT_ANGLE = 132;
const int SERVO_SLIGHT_RIGHT_ANGLE = 72;

// Distance & leg limits (cm) - ultrasonic sets drive distance dynamically
const float FRONT_STOP_DISTANCE_CM = 26.0f;
const float MIN_AVOID_STEP_CM = 12.0f;
const float MAX_MEASURED_DRIVE_CM = 85.0f;
const float MAX_SENSOR_RANGE_CM = 300.0f;

const float WAYPOINT_TOLERANCE_CM = 5.0f;
const int MAX_AVOID_DEPTH = 3;

// Mid-course heading error threshold that triggers a course correction
const float BAD_TURN_RECOVER_DEG = 16.0f;
const unsigned long HEADING_RECALC_MS = 200;

// Servo sweep angles (180 = left from behind, 90 = center, 0 = right from behind)
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 180;
const int SERVO_CENTER_ANGLE = 90;
const int SERVO_LEFT_DIAG_ANGLE = 148;
const int SERVO_RIGHT_DIAG_ANGLE = 45;

// Smooth servo stepping to avoid jitter and echo bounce
const int SERVO_STEP_DEG = 5;
const unsigned long SERVO_STEP_DELAY_MS = 6;
const unsigned long SERVO_POST_SETTLE_MS = 120;

// =====================================================
// Tunable Motion & Control Parameters (Torque-safe >= 40)
// =====================================================
float headingKp = 1.5f;
float maxCorrection = 12.0f;
float baseSpeed = 50.0f;

int turnFastSpeed = 50;
int turnMediumSpeed = 42;
int turnSlowSpeed = 40;

float turnMediumZone = 25.0f;
float turnSlowZone = 10.0f;
float turnTolerance = 1.5f;

// =====================================================
// Runtime State (Heading, Gyro, Coordinates)
// =====================================================
float heading = 0.0f;      // Committed global heading (0 deg = +Y, clockwise positive)
float gyroAngle = 0.0f;    // Integrated relative angle since last reset/commit
float gyroBias = 0.0f;
unsigned long lastTime = 0;

float posX = 0.0f;
float posY = 0.0f;
float currentTargetX = 0.0f;
float currentTargetY = 0.0f;

int currentServoAngle = SERVO_CENTER_ANGLE;
unsigned long lastServoMoveMs = 0;
bool hasRun = false;

enum PreDriveBlockSide
{
    BLOCK_NONE = 0,
    BLOCK_CENTER,
    BLOCK_SLIGHT_LEFT,
    BLOCK_SLIGHT_RIGHT
};

struct AvoidancePlan
{
    bool valid;
    bool turnLeft;
    float turnAngleDeg;
    float safeDriveCm;
};

void avoidObstacle(int depth);

// =====================================================
// Angle & Coordinate Helpers
// =====================================================
void normalizeHeadingValue(float &angleDeg)
{
    while (angleDeg < 0.0f)
        angleDeg += 360.0f;
    while (angleDeg >= 360.0f)
        angleDeg -= 360.0f;
}

void normalizeHeading()
{
    normalizeHeadingValue(heading);
}

float shortestAngleDiffDeg(float targetDeg, float currentDeg)
{
    float diff = targetDeg - currentDeg;
    while (diff > 180.0f)
        diff -= 360.0f;
    while (diff < -180.0f)
        diff += 360.0f;
    return diff;
}

float computeHeadingToPoint(float fromX, float fromY, float toX, float toY)
{
    float dx = toX - fromX;
    float dy = toY - fromY;
    float angleDeg = atan2(dx, dy) * 180.0f / PI;
    normalizeHeadingValue(angleDeg);
    return angleDeg;
}

float distanceToPointCm(float fromX, float fromY, float toX, float toY)
{
    return hypot(toX - fromX, toY - fromY);
}

float getCurrentHeading()
{
    float h = heading + gyroAngle;
    normalizeHeadingValue(h);
    return h;
}

void resetTurn()
{
    gyroAngle = 0.0f;
    lastTime = micros();
}

void updateGyro()
{
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    if (dt <= 0.0f || dt > 0.5f)
        return;

    float gz = icm20600.getGyroscopeZ() - gyroBias;
    if (fabs(gz) < 0.5f)
        gz = 0.0f;

    gyroAngle += gz * dt;
}

void settleAfterTurn()
{
    unsigned long start = millis();
    while (millis() - start < 160)
    {
        updateGyro();
        delay(5);
    }
}

void commitGyroToHeading()
{
    updateGyro();
    heading += gyroAngle;
    normalizeHeading();
    resetTurn();
}

void calibrateGyroFast()
{
    gyroBias = 0.0f;
    delay(300);

    const int samples = 200;
    for (int i = 0; i < samples; i++)
    {
        gyroBias += icm20600.getGyroscopeZ();
        delay(3);
    }

    gyroBias /= (float)samples;
    lastTime = micros();
}

void updatePositionFromMotion(float signedDistanceCm)
{
    float radians = getCurrentHeading() * PI / 180.0f;
    posX += signedDistanceCm * sin(radians);
    posY += signedDistanceCm * cos(radians);
}

// =====================================================
// Motor & Heading Control
// =====================================================
void setAllWheelSpeeds(uint8_t speed)
{
    uint8_t safe = (uint8_t)constrain((int)speed, 40, 100);
    speed_Upper_L = safe;
    speed_Lower_L = safe;
    speed_Upper_R = safe;
    speed_Lower_R = safe;
}

void correctDrivingToHeading(float desiredHeadingDeg)
{
    float error = shortestAngleDiffDeg(desiredHeadingDeg, getCurrentHeading());
    float correction = constrain(headingKp * error, -maxCorrection, maxCorrection);

    uint8_t leftSpeed = (uint8_t)constrain((int)(baseSpeed - correction), 40, 100);
    uint8_t rightSpeed = (uint8_t)constrain((int)(baseSpeed + correction), 40, 100);

    speed_Upper_L = leftSpeed;
    speed_Lower_L = leftSpeed;
    speed_Upper_R = rightSpeed;
    speed_Lower_R = rightSpeed;
}

void turnByDegrees(float targetAngle)
{
    mecanumCar.Stop();
    commitGyroToHeading();

    unsigned long startTime = millis();
    const int MAX_CORRECTIONS = 6;

    for (int attempt = 0; attempt < MAX_CORRECTIONS; attempt++)
    {
        float previousError = targetAngle - gyroAngle;

        while (true)
        {
            updateGyro();
            float error = targetAngle - gyroAngle;

            if (fabs(error) <= turnTolerance)
                break;

            if ((previousError > 0.0f && error < 0.0f) ||
                (previousError < 0.0f && error > 0.0f))
                break;

            float remaining = fabs(error);
            int turnSpeed;
            if (remaining > turnMediumZone)
                turnSpeed = turnFastSpeed;
            else if (remaining > turnSlowZone)
                turnSpeed = turnMediumSpeed;
            else
                turnSpeed = turnSlowSpeed;

            setAllWheelSpeeds((uint8_t)turnSpeed);

            if (error > 0.0f)
                mecanumCar.Turn_Left();
            else
                mecanumCar.Turn_Right();

            if (millis() - startTime > 8000)
                break;

            previousError = error;
            delay(5);
        }

        mecanumCar.Stop();
        settleAfterTurn();

        float finalError = targetAngle - gyroAngle;
        if (fabs(finalError) <= turnTolerance || millis() - startTime > 8000)
            break;
    }

    mecanumCar.Stop();
    settleAfterTurn();
    commitGyroToHeading();
}

void turnLeftDegrees(float deg)
{
    float safeDeg = constrain(fabs(deg), 0.0f, MAX_TURN_ANGLE_DEG);
    turnByDegrees(safeDeg);
}

void turnRightDegrees(float deg)
{
    float safeDeg = constrain(fabs(deg), 0.0f, MAX_TURN_ANGLE_DEG);
    turnByDegrees(-safeDeg);
}

void alignToHeading(float desiredHeadingDeg, float deadbandDeg = 3.0f)
{
    updateGyro();
    float turnNeeded = shortestAngleDiffDeg(desiredHeadingDeg, getCurrentHeading());
    if (fabs(turnNeeded) > deadbandDeg)
    {
        turnByDegrees(turnNeeded);
    }
}

// =====================================================
// Smooth Servo & Noise-Filtered Ultrasonic Sensing
// =====================================================
void moveServoSmoothly(int targetAngleDeg, unsigned long extraSettleMs = SERVO_POST_SETTLE_MS)
{
    int safeTarget = constrain(targetAngleDeg, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    if (safeTarget != currentServoAngle)
    {
        int step = (safeTarget > currentServoAngle) ? SERVO_STEP_DEG : -SERVO_STEP_DEG;
        while (currentServoAngle != safeTarget)
        {
            if (abs(safeTarget - currentServoAngle) <= SERVO_STEP_DEG)
            {
                currentServoAngle = safeTarget;
            }
            else
            {
                currentServoAngle += step;
            }

            servoSensor.write(currentServoAngle);
            unsigned long stepStart = millis();
            while (millis() - stepStart < SERVO_STEP_DELAY_MS)
            {
                updateGyro();
                delay(2);
            }
        }
    }

    unsigned long settleStart = millis();
    while (millis() - settleStart < extraSettleMs)
    {
        updateGyro();
        delay(5);
    }

    lastServoMoveMs = millis();
}

float getSingleRawDistanceCm()
{
    updateGyro();
    digitalWrite(TrigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(TrigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(TrigPin, LOW);

    long duration = pulseIn(EchoPin, HIGH, 25000);
    updateGyro();

    if (duration == 0)
        return -1.0f;

    float cm = duration * 0.0343f / 2.0f;
    if (cm < 4.0f || cm > MAX_SENSOR_RANGE_CM)
        return -1.0f;

    return cm;
}

float readFilteredDistanceCm()
{
    float s[3];
    int validCount = 0;

    for (int i = 0; i < 3; i++)
    {
        float d = getSingleRawDistanceCm();
        if (d > 0.0f)
        {
            s[validCount++] = d;
        }
        if (i < 2)
        {
            delay(12);
            updateGyro();
        }
    }

    if (validCount == 0)
        return -1.0f;
    if (validCount == 1)
        return s[0];
    if (validCount == 2)
        return 0.5f * (s[0] + s[1]);

    for (int i = 0; i < 2; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            if (s[j] < s[i])
            {
                float tmp = s[i];
                s[i] = s[j];
                s[j] = tmp;
            }
        }
    }
    return s[1];
}

float scanAngleCm(int angleDeg, unsigned long settleMs = SERVO_POST_SETTLE_MS)
{
    moveServoSmoothly(angleDeg, settleMs);
    float d = readFilteredDistanceCm();
    return (d < 0.0f) ? MAX_SENSOR_RANGE_CM : d;
}

float getEffectiveHalfWidthCm(bool leftSideExposed)
{
    return leftSideExposed ? LEFT_HALF_WIDTH_CM : RIGHT_HALF_WIDTH_CM;
}

// Measures how far the robot can safely drive straight ahead using the ultrasonic sensor
float measureSafeForwardDriveCm(bool leftSideExposed = true)
{
    moveServoSmoothly(SERVO_CENTER_ANGLE, 110);
    float measuredCm = readFilteredDistanceCm();
    float openDistCm = (measuredCm < 0.0f) ? MAX_SENSOR_RANGE_CM : measuredCm;

    float extraLeftPad = leftSideExposed ? (LEFT_EXTRA_WIDTH_CM + LEFT_TURN_CLEARANCE_PAD_CM) : 0.0f;
    float clearanceNeeded = getEffectiveHalfWidthCm(leftSideExposed) + CHASSIS_BUFFER_CM + PASSING_MARGIN_CM + extraLeftPad;

    float safeCm = openDistCm - clearanceNeeded;
    if (safeCm < 0.0f)
        safeCm = 0.0f;
    if (safeCm > MAX_MEASURED_DRIVE_CM)
        safeCm = MAX_MEASURED_DRIVE_CM;

    float goalDistCm = distanceToPointCm(posX, posY, currentTargetX, currentTargetY);
    if (goalDistCm > 0.0f && safeCm > goalDistCm)
    {
        safeCm = goalDistCm;
    }

    return safeCm;
}

// Checks straight ahead (90 deg), wide left (132 deg) for +5 cm extra width, and slight right (72 deg)
PreDriveBlockSide checkPreDriveBlockSide()
{
    mecanumCar.Stop();
    settleAfterTurn();

    moveServoSmoothly(SERVO_CENTER_ANGLE, 80);
    float dCenter = readFilteredDistanceCm();
    if (dCenter > 0.0f && dCenter < PRE_DRIVE_CLEARANCE_CM)
    {
        return BLOCK_CENTER;
    }

    moveServoSmoothly(SERVO_SLIGHT_LEFT_ANGLE, 85);
    float dLeft = readFilteredDistanceCm();
    float minLeftClearance = PRE_DRIVE_CLEARANCE_CM + LEFT_EXTRA_WIDTH_CM;
    if (dLeft > 0.0f && dLeft < minLeftClearance)
    {
        moveServoSmoothly(SERVO_CENTER_ANGLE, 80);
        return BLOCK_SLIGHT_LEFT;
    }

    moveServoSmoothly(SERVO_SLIGHT_RIGHT_ANGLE, 70);
    float dRight = readFilteredDistanceCm();
    if (dRight > 0.0f && dRight < PRE_DRIVE_CLEARANCE_CM)
    {
        moveServoSmoothly(SERVO_CENTER_ANGLE, 80);
        return BLOCK_SLIGHT_RIGHT;
    }

    moveServoSmoothly(SERVO_CENTER_ANGLE, 80);
    return BLOCK_NONE;
}

// When the pre-drive check sees an obstacle, turn a little more away instead of turning back
bool nudgeClearOfPreDriveObstacle()
{
    for (int i = 0; i < MAX_PREDRIVE_NUDGES; i++)
    {
        PreDriveBlockSide blockSide = checkPreDriveBlockSide();
        if (blockSide == BLOCK_NONE)
        {
            return true;
        }

        if (blockSide == BLOCK_SLIGHT_LEFT)
        {
            turnRightDegrees(EXTRA_NUDGE_TURN_DEG + (LEFT_EXTRA_TURN_DEG * 0.5f));
        }
        else if (blockSide == BLOCK_SLIGHT_RIGHT)
        {
            turnLeftDegrees(EXTRA_NUDGE_TURN_DEG + LEFT_EXTRA_TURN_DEG);
        }
        else
        {
            float dL = scanAngleCm(SERVO_SLIGHT_LEFT_ANGLE, 85) - LEFT_EXTRA_WIDTH_CM;
            float dR = scanAngleCm(SERVO_SLIGHT_RIGHT_ANGLE, 80);
            moveServoSmoothly(SERVO_CENTER_ANGLE, 80);

            if (dR > dL)
                turnRightDegrees(EXTRA_NUDGE_TURN_DEG);
            else
                turnLeftDegrees(EXTRA_NUDGE_TURN_DEG + LEFT_EXTRA_TURN_DEG);
        }
    }

    return (checkPreDriveBlockSide() == BLOCK_NONE);
}

// =====================================================
// Dimension-Aware & Coordinate-Guided Avoidance Planner
// =====================================================
AvoidancePlan planAvoidanceManeuver()
{
    AvoidancePlan plan;
    plan.valid = false;
    plan.turnLeft = true;
    plan.turnAngleDeg = 90.0f;
    plan.safeDriveCm = 0.0f;

    mecanumCar.Stop();
    settleAfterTurn();

    float dLeftFull = scanAngleCm(SERVO_MAX_ANGLE, 130);       // 180 deg (left)
    float dLeftDiag = scanAngleCm(SERVO_LEFT_DIAG_ANGLE, 115); // 148 deg (further left for +5 cm side)
    float dFront    = scanAngleCm(SERVO_CENTER_ANGLE, 110);    //  90 deg (center)
    float dRightDiag= scanAngleCm(SERVO_RIGHT_DIAG_ANGLE, 110);//  45 deg (front-right)
    float dRightFull= scanAngleCm(SERVO_MIN_ANGLE, 130);       //   0 deg (right)

    moveServoSmoothly(SERVO_CENTER_ANGLE, 120);

    float leftClearance90  = max(0.0f, dLeftFull - (LEFT_HALF_WIDTH_CM + CHASSIS_BUFFER_CM + PASSING_MARGIN_CM + LEFT_EXTRA_WIDTH_CM + LEFT_TURN_CLEARANCE_PAD_CM));
    float leftClearance45  = max(0.0f, dLeftDiag - (LEFT_HALF_WIDTH_CM + CHASSIS_BUFFER_CM + PASSING_MARGIN_CM + LEFT_EXTRA_WIDTH_CM + LEFT_TURN_CLEARANCE_PAD_CM));
    float rightClearance90 = max(0.0f, dRightFull - (RIGHT_HALF_WIDTH_CM + CHASSIS_BUFFER_CM + PASSING_MARGIN_CM));
    float rightClearance45 = max(0.0f, dRightDiag - (RIGHT_HALF_WIDTH_CM + CHASSIS_BUFFER_CM + PASSING_MARGIN_CM));

    float currentHdg = getCurrentHeading();
    float goalHdg = computeHeadingToPoint(posX, posY, currentTargetX, currentTargetY);
    float goalDistCm = distanceToPointCm(posX, posY, currentTargetX, currentTargetY);

    float leftAngle45 = constrain(45.0f + LEFT_EXTRA_TURN_DEG, 45.0f, MAX_TURN_ANGLE_DEG);
    float leftAngle90 = constrain(90.0f + LEFT_EXTRA_TURN_DEG, 90.0f, MAX_TURN_ANGLE_DEG);

    float hLeft45  = currentHdg + leftAngle45; normalizeHeadingValue(hLeft45);
    float hLeft90  = currentHdg + leftAngle90; normalizeHeadingValue(hLeft90);
    float hRight45 = currentHdg - 45.0f;      normalizeHeadingValue(hRight45);
    float hRight90 = currentHdg - 90.0f;      normalizeHeadingValue(hRight90);

    float errLeft45  = fabs(shortestAngleDiffDeg(goalHdg, hLeft45));
    float errLeft90  = fabs(shortestAngleDiffDeg(goalHdg, hLeft90));
    float errRight45 = fabs(shortestAngleDiffDeg(goalHdg, hRight45));
    float errRight90 = fabs(shortestAngleDiffDeg(goalHdg, hRight90));

    float scoreLeft45  = (leftClearance45 * 1.15f)  - (errLeft45 * 0.65f);
    float scoreLeft90  = (leftClearance90 * 1.00f)  - (errLeft90 * 0.65f);
    float scoreRight45 = (rightClearance45 * 1.15f) - (errRight45 * 0.65f);
    float scoreRight90 = (rightClearance90 * 1.00f) - (errRight90 * 0.65f);

    float bestLeftScore  = (scoreLeft45 >= scoreLeft90) ? scoreLeft45 : scoreLeft90;
    float bestRightScore = (scoreRight45 >= scoreRight90) ? scoreRight45 : scoreRight90;

    bool hasLeftRoom  = (leftClearance45 >= MIN_AVOID_STEP_CM || leftClearance90 >= MIN_AVOID_STEP_CM);
    bool hasRightRoom = (rightClearance45 >= MIN_AVOID_STEP_CM || rightClearance90 >= MIN_AVOID_STEP_CM);

    if (hasLeftRoom && !hasRightRoom)
        plan.turnLeft = true;
    else if (hasRightRoom && !hasLeftRoom)
        plan.turnLeft = false;
    else
        plan.turnLeft = (bestLeftScore >= bestRightScore);

    if (plan.turnLeft)
    {
        if (leftClearance45 >= MIN_AVOID_STEP_CM && scoreLeft45 >= scoreLeft90)
        {
            plan.turnAngleDeg = leftAngle45;
            plan.safeDriveCm = leftClearance45;
        }
        else
        {
            plan.turnAngleDeg = leftAngle90;
            plan.safeDriveCm = leftClearance90;
        }
    }
    else
    {
        if (rightClearance45 >= MIN_AVOID_STEP_CM && scoreRight45 >= scoreRight90)
        {
            plan.turnAngleDeg = 45.0f;
            plan.safeDriveCm = rightClearance45;
        }
        else
        {
            plan.turnAngleDeg = 90.0f;
            plan.safeDriveCm = rightClearance90;
        }
    }

    if (plan.safeDriveCm > MAX_MEASURED_DRIVE_CM)
    {
        plan.safeDriveCm = MAX_MEASURED_DRIVE_CM;
    }

    if (goalDistCm > 0.0f && plan.safeDriveCm > goalDistCm)
    {
        plan.safeDriveCm = goalDistCm;
    }

    (void)dFront;
    plan.valid = (plan.safeDriveCm >= MIN_AVOID_STEP_CM);
    return plan;
}

// Measures open distance along `desiredHeadingDeg` with ultrasonic and drives that distance using RAW_CM_PER_SECOND
void driveUltrasonicMeasuredLeg(float fallbackDistanceCm, float desiredHeadingDeg)
{
    alignToHeading(desiredHeadingDeg, 3.0f);

    if (!nudgeClearOfPreDriveObstacle())
    {
        return;
    }

    float measuredSafeCm = measureSafeForwardDriveCm(true);
    float targetDistanceCm = (measuredSafeCm >= MIN_AVOID_STEP_CM) ? measuredSafeCm : fallbackDistanceCm;
    if (targetDistanceCm <= 0.0f)
        return;

    unsigned long maxDurationMs = (unsigned long)((targetDistanceCm / RAW_CM_PER_SECOND) * 1000.0f);
    unsigned long startTime = millis();
    unsigned long lastUpdate = millis();
    unsigned long lastFrontCheck = 0;

    float drivenCm = 0.0f;
    commitGyroToHeading();
    setAllWheelSpeeds((uint8_t)baseSpeed);
    mecanumCar.Advance();

    while (millis() - startTime < maxDurationMs && drivenCm < targetDistanceCm)
    {
        updateGyro();

        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0f;
        lastUpdate = now;

        float stepCm = RAW_CM_PER_SECOND * dt;
        if (drivenCm + stepCm > targetDistanceCm)
        {
            stepCm = targetDistanceCm - drivenCm;
        }

        updatePositionFromMotion(stepCm);
        drivenCm += stepCm;

        correctDrivingToHeading(desiredHeadingDeg);
        mecanumCar.Advance();

        if (drivenCm >= 6.0f && currentServoAngle == SERVO_CENTER_ANGLE && (now - lastFrontCheck >= 110))
        {
            lastFrontCheck = now;
            float frontDist = readFilteredDistanceCm();
            if (frontDist > 0.0f && frontDist < FRONT_STOP_DISTANCE_CM)
            {
                mecanumCar.Stop();
                settleAfterTurn();
                commitGyroToHeading();
                return;
            }
        }

        delay(12);
    }

    mecanumCar.Stop();
    settleAfterTurn();
    commitGyroToHeading();
}

void avoidObstacle(int depth)
{
    if (depth > MAX_AVOID_DEPTH)
        return;

    AvoidancePlan plan = planAvoidanceManeuver();
    if (!plan.valid)
    {
        float currentHdg = getCurrentHeading();
        float goalHdg = computeHeadingToPoint(posX, posY, currentTargetX, currentTargetY);
        float hL = currentHdg + (90.0f + LEFT_EXTRA_TURN_DEG); normalizeHeadingValue(hL);
        float hR = currentHdg - 90.0f;                         normalizeHeadingValue(hR);

        if (fabs(shortestAngleDiffDeg(goalHdg, hL)) <= fabs(shortestAngleDiffDeg(goalHdg, hR)))
            turnLeftDegrees(90.0f + LEFT_EXTRA_TURN_DEG);
        else
            turnRightDegrees(90.0f);

        driveUltrasonicMeasuredLeg(MIN_AVOID_STEP_CM, getCurrentHeading());
    }
    else
    {
        if (plan.turnLeft)
            turnLeftDegrees(plan.turnAngleDeg);
        else
            turnRightDegrees(plan.turnAngleDeg);

        driveUltrasonicMeasuredLeg(plan.safeDriveCm, getCurrentHeading());
    }

    // Check straight ahead after the leg: avoid again if still blocked, otherwise re-aim to waypoint
    moveServoSmoothly(SERVO_CENTER_ANGLE, 110);
    float aheadDist = readFilteredDistanceCm();
    if (aheadDist > 0.0f && aheadDist < FRONT_STOP_DISTANCE_CM && depth < MAX_AVOID_DEPTH)
    {
        avoidObstacle(depth + 1);
        return;
    }

    float nextGoalHeading = computeHeadingToPoint(posX, posY, currentTargetX, currentTargetY);
    alignToHeading(nextGoalHeading, 4.0f);
    moveServoSmoothly(SERVO_CENTER_ANGLE, 110);
}

// =====================================================
// Coordinate-Driven Waypoint Navigation
// =====================================================
void driveToWaypoint(float targetX, float targetY)
{
    currentTargetX = targetX;
    currentTargetY = targetY;

    while (true)
    {
        updateGyro();
        float remainingCm = distanceToPointCm(posX, posY, targetX, targetY);
        if (remainingCm <= WAYPOINT_TOLERANCE_CM)
            break;

        float desiredHeading = computeHeadingToPoint(posX, posY, targetX, targetY);
        float turnNeeded = shortestAngleDiffDeg(desiredHeading, getCurrentHeading());

        if (fabs(turnNeeded) > 4.0f)
        {
            turnByDegrees(turnNeeded);
        }

        if (!nudgeClearOfPreDriveObstacle())
        {
            avoidObstacle(0);
            continue;
        }

        commitGyroToHeading();

        unsigned long lastUpdate = millis();
        unsigned long lastRecalc = millis();
        unsigned long lastFrontCheck = 0;
        bool needOuterRecalc = false;

        setAllWheelSpeeds((uint8_t)baseSpeed);
        mecanumCar.Advance();

        while (true)
        {
            updateGyro();

            unsigned long now = millis();
            float dt = (now - lastUpdate) / 1000.0f;
            lastUpdate = now;

            float stepCm = WAYPOINT_CM_PER_SECOND * dt;
            updatePositionFromMotion(stepCm);

            remainingCm = distanceToPointCm(posX, posY, targetX, targetY);
            if (remainingCm <= WAYPOINT_TOLERANCE_CM)
            {
                mecanumCar.Stop();
                settleAfterTurn();
                commitGyroToHeading();
                return;
            }

            if (now - lastRecalc >= HEADING_RECALC_MS)
            {
                lastRecalc = now;
                desiredHeading = computeHeadingToPoint(posX, posY, targetX, targetY);
                float err = shortestAngleDiffDeg(desiredHeading, getCurrentHeading());
                if (fabs(err) > BAD_TURN_RECOVER_DEG)
                {
                    mecanumCar.Stop();
                    settleAfterTurn();
                    commitGyroToHeading();
                    needOuterRecalc = true;
                    break;
                }
            }

            correctDrivingToHeading(desiredHeading);
            mecanumCar.Advance();

            if (currentServoAngle == SERVO_CENTER_ANGLE && (now - lastFrontCheck >= 110))
            {
                lastFrontCheck = now;
                float frontDistCm = readFilteredDistanceCm();
                if (frontDistCm > 0.0f && frontDistCm < FRONT_STOP_DISTANCE_CM)
                {
                    mecanumCar.Stop();
                    settleAfterTurn();
                    commitGyroToHeading();

                    avoidObstacle(0);
                    needOuterRecalc = true;
                    break;
                }
            }

            delay(12);
        }

        if (needOuterRecalc)
        {
            commitGyroToHeading();
        }
    }

    mecanumCar.Stop();
}

// =====================================================
// Setup & Main Loop
// =====================================================
void setup()
{
    pinMode(TrigPin, OUTPUT);
    pinMode(EchoPin, INPUT);

    mecanumCar.Init();
    setAllWheelSpeeds((uint8_t)baseSpeed);

    Wire.begin();
    icm20600.initialize();
    mag.initialize();

    calibrateGyroFast();

    servoSensor.attach(ServoPin);
    servoSensor.write(SERVO_CENTER_ANGLE);
    currentServoAngle = SERVO_CENTER_ANGLE;
    lastServoMoveMs = millis();
    delay(250);
}

void loop()
{
    if (hasRun)
        return;

    driveToWaypoint(0.0f, 150.0f);
    driveToWaypoint(0.0f, 0.0f);

    hasRun = true;
}