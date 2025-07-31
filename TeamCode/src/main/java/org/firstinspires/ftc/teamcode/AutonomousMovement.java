package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.Robot;


public class AutonomousMovement {

    private LinearOpMode opMode;
    private Robot robot;

    // Constructor
    public AutonomousMovement(LinearOpMode opMode) {
        this.opMode = opMode;
        this.robot = new Robot(opMode);
    }

    // Movement parameters
    private double maxSpeed = 1.0;
    private double acceleration = 1.5;
    private double deceleration = 2.0;
    private double positionTolerance = 1.0;
    private double headingTolerance = 2.0;
    private double P = 0.058;
    private double minPower = 0.11;

    // Motion profiling
    private double currentVelocity = 0.0;
    private double baseLookaheadDistance = 6.0; // Base lookahead distance

    /**
     * Enhanced movement with proper path following and motion profiling
     */
    public void enhancedMovement(double startX, double startY, double endX, double endY,
                                 double startHeading, double endHeading, double maxSpeed,
                                 String pathType, double positionTolerance) {

        this.maxSpeed = maxSpeed;
        this.positionTolerance = positionTolerance;
        this.currentVelocity = 0.0;

        // Handle special case for "linear_turn_start"
        if (pathType.toLowerCase().equals("linear_turn_start")) {
            // First turn to target heading
            turnToHeading(endHeading);
            // Then move linearly with constant heading
            linearMovementWithHeading(startX, startY, endX, endY, endHeading, maxSpeed, positionTolerance);
            return;
        }

        // For other path types, use path following
        PathGenerator path = new PathGenerator(startX, startY, endX, endY,
                startHeading, endHeading, pathType);

        ElapsedTime totalTimer = new ElapsedTime();
        ElapsedTime deltaTimer = new ElapsedTime();
        deltaTimer.reset();

        while (opMode.opModeIsActive() && totalTimer.milliseconds() < 10000) {

            SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
            pos.x *= -1;
            pos.y *= -1;

            double deltaTime = deltaTimer.seconds();
            deltaTimer.reset();
            if (deltaTime <= 0) deltaTime = 0.02;

            // Find closest point on path to robot
            PathResult pathResult = path.getClosestPointOnPath(pos.x, pos.y);

            // Calculate remaining distance along path
            double remainingDistance = path.getRemainingDistance(pathResult.closestIndex);

            // Calculate target velocity based on remaining distance
            double targetVelocity = calculateTargetVelocity(remainingDistance, deltaTime);

            // Get adaptive lookahead point for smooth following
            double adaptiveLookahead = Math.max(baseLookaheadDistance, targetVelocity * 1.5);
            PathPoint lookaheadPoint = path.getLookaheadPoint(pathResult.closestIndex, adaptiveLookahead);

            // Check if we've reached the end
            double distanceToEnd = Math.sqrt(Math.pow(endX - pos.x, 2) + Math.pow(endY - pos.y, 2));
            double headingError = normalizeAngle(lookaheadPoint.heading - pos.h);

            if (distanceToEnd < positionTolerance && Math.abs(headingError) < headingTolerance) {
                robot.drive.drivePower(0, 0, 0);
                break;
            }

            // Calculate movement to lookahead point
            double dx = lookaheadPoint.x - pos.x;
            double dy = lookaheadPoint.y - pos.y;
            double distanceToLookahead = Math.sqrt(dx * dx + dy * dy);

            // Scale movement by target velocity
            double strafe = 0, forward = 0;
            if (distanceToLookahead >= 0.1) {
                strafe = (dx / distanceToLookahead) * targetVelocity;
                forward = (dy / distanceToLookahead) * targetVelocity;
            }

            double turn = headingError;

            // Apply control gains
            strafe *= P;
            forward *= P;
            turn *= P * (2.0 / 3.0);

            // Add minimum power
            if (Math.abs(strafe) > 0.05) strafe += Math.copySign(minPower, strafe);
            if (Math.abs(forward) > 0.05) forward += Math.copySign(minPower, forward);

            // Clip to max speed
            strafe = Range.clip(strafe, -maxSpeed, maxSpeed);
            forward = Range.clip(forward, -maxSpeed, maxSpeed);

            // Convert to robot frame
            double botHeading = Math.toRadians(pos.h);
            double rotStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            // Telemetry
            double progress = 1.0 - (remainingDistance / path.getTotalDistance());
            opMode.telemetry.addData("Path Type", pathType);
            opMode.telemetry.addData("Progress", "%.1f%%", progress * 100);
            opMode.telemetry.addData("Target Velocity", "%.2f", targetVelocity);
            opMode.telemetry.addData("Remaining Dist", "%.1f", remainingDistance);
            opMode.telemetry.addData("Cross Track Error", "%.1f", pathResult.crossTrackError);
            opMode.telemetry.addData("Current Pos", "%.1f, %.1f", pos.x, pos.y);
            opMode.telemetry.addData("Lookahead", "%.1f, %.1f", lookaheadPoint.x, lookaheadPoint.y);
            opMode.telemetry.addData("Heading Error", "%.1f°", headingError);
            opMode.telemetry.update();

            robot.drive.drivePower(rotForward, rotStrafe, turn);
        }
    }

    /**
     * Turn to specific heading (for linear_turn_start)
     */
    private void turnToHeading(double targetHeading) {
        ElapsedTime turnTimer = new ElapsedTime();

        while (opMode.opModeIsActive() && turnTimer.milliseconds() < 3000) {
            SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
            pos.x *= -1;
            pos.y *= -1;

            double headingError = normalizeAngle(targetHeading - pos.h);

            if (Math.abs(headingError) < headingTolerance) {
                robot.drive.drivePower(0, 0, 0);
                break;
            }

            double turn = headingError * P * (2.0 / 3.0);
            turn = Range.clip(turn, -0.5, 0.5); // Limit turn speed

            robot.drive.drivePower(0, 0, turn);
        }
    }

    /**
     * Linear movement with constant heading
     */
    private void linearMovementWithHeading(double startX, double startY, double endX, double endY,
                                           double constantHeading, double maxSpeed, double tolerance) {

        this.currentVelocity = 0.0;
        double totalDistance = Math.sqrt(Math.pow(endX - startX, 2) + Math.pow(endY - startY, 2));

        ElapsedTime moveTimer = new ElapsedTime();
        ElapsedTime deltaTimer = new ElapsedTime();
        deltaTimer.reset();

        while (opMode.opModeIsActive() && moveTimer.milliseconds() < 8000) {
            SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
            pos.x *= -1;
            pos.y *= -1;

            double deltaTime = deltaTimer.seconds();
            deltaTimer.reset();
            if (deltaTime <= 0) deltaTime = 0.02;

            double distanceToEnd = Math.sqrt(Math.pow(endX - pos.x, 2) + Math.pow(endY - pos.y, 2));
            double headingError = normalizeAngle(constantHeading - pos.h);

            if (distanceToEnd < tolerance && Math.abs(headingError) < headingTolerance) {
                robot.drive.drivePower(0, 0, 0);
                break;
            }

            // Calculate target velocity
            double targetVelocity = calculateTargetVelocity(distanceToEnd, deltaTime);

            // Movement towards target
            double dx = endX - pos.x;
            double dy = endY - pos.y;
            double distance = Math.sqrt(dx * dx + dy * dy);

            double strafe = 0, forward = 0;
            if (distance > 0.1) {
                strafe = (dx / distance) * targetVelocity;
                forward = (dy / distance) * targetVelocity;
            }

            double turn = headingError;

            // Apply gains
            strafe *= P;
            forward *= P;
            turn *= P * (2.0 / 3.0);

            // Add minimum power
            if (Math.abs(strafe) > 0.05) strafe += Math.copySign(minPower, strafe);
            if (Math.abs(forward) > 0.05) forward += Math.copySign(minPower, forward);

            // Clip speeds
            strafe = Range.clip(strafe, -maxSpeed, maxSpeed);
            forward = Range.clip(forward, -maxSpeed, maxSpeed);

            // Robot frame conversion
            double botHeading = Math.toRadians(pos.h);
            double rotStrafe = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotForward = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            robot.drive.drivePower(rotForward, rotStrafe, turn);
        }
    }

    /**
     * Calculate target velocity with trapezoidal profile
     */
    private double calculateTargetVelocity(double remainingDistance, double deltaTime) {
        double decelerationDistance = (currentVelocity * currentVelocity) / (2 * deceleration);
        double targetVelocity;

        if (remainingDistance <= decelerationDistance + 2.0) {
            // Deceleration phase - prevent sqrt of negative numbers
            double safeDistance = Math.max(0.1, remainingDistance - 1.0);
            targetVelocity = Math.max(0.15, Math.sqrt(2 * deceleration * safeDistance));
        } else if (currentVelocity < maxSpeed) {
            // Acceleration phase
            targetVelocity = Math.min(maxSpeed, currentVelocity + acceleration * deltaTime);
        } else {
            // Cruise phase
            targetVelocity = maxSpeed;
        }

        currentVelocity = targetVelocity;
        return currentVelocity;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private double interpolateAngle(double startAngle, double endAngle, double progress) {
        double diff = normalizeAngle(endAngle - startAngle);
        return normalizeAngle(startAngle + diff * progress);
    }

    // Setters
    public void setAcceleration(double acceleration) { this.acceleration = acceleration; }
    public void setDeceleration(double deceleration) { this.deceleration = deceleration; }
    public void setPIDConstants(double P, double minPower) { this.P = P; this.minPower = minPower; }
    public void setTolerances(double positionTolerance, double headingTolerance) {
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
    }
    public void setLookaheadDistance(double distance) { this.baseLookaheadDistance = distance; }

    /**
     * Path generator for curves and linear paths
     */
    private class PathGenerator {
        private PathPoint[] pathPoints;
        private double totalDistance;
        private String pathType;
        private double startX, startY, endX, endY, startHeading, endHeading;
        private final int RESOLUTION = 200; // Higher resolution for smoother curves

        public PathGenerator(double startX, double startY, double endX, double endY,
                             double startHeading, double endHeading, String pathType) {
            this.startX = startX;
            this.startY = startY;
            this.endX = endX;
            this.endY = endY;
            this.startHeading = startHeading;
            this.endHeading = endHeading;
            this.pathType = pathType.toLowerCase();

            generatePath();
        }

        private void generatePath() {
            pathPoints = new PathPoint[RESOLUTION + 1];
            totalDistance = 0;

            // Generate path points
            for (int i = 0; i <= RESOLUTION; i++) {
                double t = (double)i / RESOLUTION;
                pathPoints[i] = calculatePathPoint(t);

                if (i > 0) {
                    double dx = pathPoints[i].x - pathPoints[i-1].x;
                    double dy = pathPoints[i].y - pathPoints[i-1].y;
                    totalDistance += Math.sqrt(dx * dx + dy * dy);
                }
            }
        }

        private PathPoint calculatePathPoint(double t) {
            PathPoint point = new PathPoint();

            switch (pathType) {
                case "linear_turn_distributed":
                    point.x = startX + (endX - startX) * t;
                    point.y = startY + (endY - startY) * t;
                    point.heading = interpolateAngle(startHeading, endHeading, t);
                    break;

                case "bezier":
                    point = calculateBezierPoint(t);
                    point.heading = interpolateAngle(startHeading, endHeading, t);
                    break;

                default:
                    point.x = startX + (endX - startX) * t;
                    point.y = startY + (endY - startY) * t;
                    point.heading = interpolateAngle(startHeading, endHeading, t);
                    break;
            }

            return point;
        }

        private PathPoint calculateBezierPoint(double t) {
            double distance = Math.sqrt(Math.pow(endX - startX, 2) + Math.pow(endY - startY, 2));
            double controlDistance = Math.min(distance * 0.25, 10.0); // Limit control distance

            // FTC convention: 0° = North (+Y), 90° = East (+X)
            // Convert to standard math: 0° = East (+X), 90° = North (+Y)
            // FTC North (0°) = Math North (90°), so: Math angle = 90 - FTC angle
            double startAngleRad = Math.toRadians(90 - startHeading);
            double endAngleRad = Math.toRadians(90 - endHeading);

            double cp1X = startX + controlDistance * Math.cos(startAngleRad);
            double cp1Y = startY + controlDistance * Math.sin(startAngleRad);

            double cp2X = endX - controlDistance * Math.cos(endAngleRad);
            double cp2Y = endY - controlDistance * Math.sin(endAngleRad);

            // Cubic Bezier
            double u = 1 - t;
            PathPoint point = new PathPoint();
            point.x = u*u*u * startX + 3*u*u*t * cp1X + 3*u*t*t * cp2X + t*t*t * endX;
            point.y = u*u*u * startY + 3*u*u*t * cp1Y + 3*u*t*t * cp2Y + t*t*t * endY;

            return point;
        }

        public PathResult getClosestPointOnPath(double robotX, double robotY) {
            double minDistance = Double.MAX_VALUE;
            int closestIndex = 0;

            // Find closest discrete point
            for (int i = 0; i < pathPoints.length; i++) {
                double dx = pathPoints[i].x - robotX;
                double dy = pathPoints[i].y - robotY;
                double distance = Math.sqrt(dx * dx + dy * dy);

                if (distance < minDistance) {
                    minDistance = distance;
                    closestIndex = i;
                }
            }

            // Refine by checking adjacent segments for closer point
            if (closestIndex > 0 && closestIndex < pathPoints.length - 1) {
                // Check if we're closer to the line segment between adjacent points
                double betterDistance = pointToSegmentDistance(robotX, robotY,
                        pathPoints[closestIndex-1], pathPoints[closestIndex]);
                if (betterDistance < minDistance) {
                    minDistance = betterDistance;
                }

                betterDistance = pointToSegmentDistance(robotX, robotY,
                        pathPoints[closestIndex], pathPoints[closestIndex+1]);
                if (betterDistance < minDistance) {
                    minDistance = betterDistance;
                }
            }

            PathResult result = new PathResult();
            result.closestIndex = closestIndex;
            result.crossTrackError = minDistance;
            return result;
        }

        private double pointToSegmentDistance(double px, double py, PathPoint p1, PathPoint p2) {
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double length = Math.sqrt(dx * dx + dy * dy);

            if (length < 0.001) return Math.sqrt((px - p1.x) * (px - p1.x) + (py - p1.y) * (py - p1.y));

            double t = Math.max(0, Math.min(1, ((px - p1.x) * dx + (py - p1.y) * dy) / (length * length)));
            double projX = p1.x + t * dx;
            double projY = p1.y + t * dy;

            return Math.sqrt((px - projX) * (px - projX) + (py - projY) * (py - projY));
        }

        public PathPoint getLookaheadPoint(int startIndex, double lookaheadDistance) {
            double accumulatedDistance = 0;

            // Ensure we don't go backward - start from at least current index
            int searchStart = Math.max(0, startIndex);

            for (int i = searchStart; i < pathPoints.length - 1; i++) {
                double dx = pathPoints[i+1].x - pathPoints[i].x;
                double dy = pathPoints[i+1].y - pathPoints[i].y;
                double segmentDistance = Math.sqrt(dx * dx + dy * dy);

                if (accumulatedDistance + segmentDistance >= lookaheadDistance) {
                    return pathPoints[i+1];
                }

                accumulatedDistance += segmentDistance;
            }

            // Return end point if lookahead goes beyond path
            return pathPoints[pathPoints.length - 1];
        }

        public double getRemainingDistance(int currentIndex) {
            double remaining = 0;

            for (int i = currentIndex; i < pathPoints.length - 1; i++) {
                double dx = pathPoints[i+1].x - pathPoints[i].x;
                double dy = pathPoints[i+1].y - pathPoints[i].y;
                remaining += Math.sqrt(dx * dx + dy * dy);
            }

            return remaining;
        }

        public double getTotalDistance() {
            return totalDistance;
        }
    }

    private class PathPoint {
        public double x, y, heading;
        public PathPoint() { x = 0; y = 0; heading = 0; }
    }

    private class PathResult {
        public int closestIndex;
        public double crossTrackError;
    }
}