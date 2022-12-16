
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.helper.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.helper.Robot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoRightCone extends LinearOpMode
{
    /* Declare OpMode members. */
    private BNO055IMU imu = null;
    private DcMotor topLeftMotor = null;
    private DcMotor topRightMotor = null;
    private DcMotor bottomLeftMotor = null;
    private DcMotor bottomRightMotor = null;
    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;
    private boolean detected = false;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftSpeed = 0;
    private double rightSpeed = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 766.106508876 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 2.95276 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants dfine the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.9;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.8;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.15;     // Larger is more responsive, but also less stable

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    Robot robot = new Robot(); // Instantiate Robot Class to Access Drive Motors

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    final int TAG_1 = 163; // Tag ID 63 from the 36h11 family
    final int TAG_2 = 164; // Tag ID 63 from the 36h11 family
    final int TAG_3 = 165; // Tag ID 63 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        topLeftMotor = robot.topLeftMotor;
        topRightMotor = robot.topRightMotor;
        bottomLeftMotor = robot.bottomLeftMotor;
        bottomRightMotor = robot.bottomRightMotor;

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        robot.init(hardwareMap);
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                tagOfInterest = currentDetections.get(0);
                detected = true;
                telemetry.addLine(String.valueOf(tagOfInterest.id));
            }
            else
            {
                if (detected) {
                    telemetry.addLine("Tag Not in Immediate Line of Sight, but Detected in Past" + "(" + tagOfInterest.id + ")");
                } else {
                    telemetry.addLine("Tag Not in Immediate Line of Sight");
                }
            }
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        camera.closeCameraDevice();

        // Set the encoders for closed loop speed control, and reset the heading.
        topLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        /* Actually do something useful */
        if (tagOfInterest != null) {
            telemetry.addData("Tag Found", tagOfInterest.id);
        } else {
            telemetry.addLine("No Tag Found, Defaulting to Position 2 (Middle/id:164)");
        }
        telemetry.update();


        turnToHeading(TURN_SPEED + 0.1, -90);
        holdHeading(TURN_SPEED, -90, 0.5);

        driveStraight(DRIVE_SPEED + 0.1, 27, -90);
        holdHeading(TURN_SPEED, -90, 0.5);

        driveStraight(DRIVE_SPEED, -37, -90);
        holdHeading(TURN_SPEED, -90, 0.5);

        turnToHeading(TURN_SPEED + 0.1, 0);
        holdHeading(TURN_SPEED, 0, 0.5);

        driveStraight(DRIVE_SPEED, 45, 0);
        holdHeading(TURN_SPEED, 0, 0.5);


        turnToHeading(TURN_SPEED, -90);
        holdHeading(TURN_SPEED, -90, 0.5);

        robot.jointAMotor.setTargetPosition(537);
        robot.jointAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jointAMotor.setPower(0.5);

        while (opModeIsActive() && robot.jointAMotor.isBusy()) {}

        robot.jointAMotor.setTargetPosition(320);
        robot.jointAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jointAMotor.setPower(-0.3);

        robot.jointBMotor.setTargetPosition(-1083);
        robot.jointBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jointBMotor.setPower(-0.5);

        while (opModeIsActive() && robot.jointAMotor.isBusy() && robot.jointBMotor.isBusy()) {
            telemetry.addLine("Joints Moving");
            telemetry.update();
        }

        robot.clawMotor.setTargetPosition(12);
        robot.clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.clawMotor.setPower(0.7);

        while (opModeIsActive() && robot.clawMotor.isBusy()) {}

        robot.jointAMotor.setTargetPosition(220);
        robot.jointAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jointAMotor.setPower(-0.3);

        while (opModeIsActive() && robot.jointAMotor.isBusy()) {}

        robot.clawServo.setPower(0);
        sleep(1000);

        directDrive(DRIVE_SPEED, 11);
        sleep(700);

        robot.clawServo.setPower(1);
        sleep(1000);

        directDrive(DRIVE_SPEED, -4);

        robot.jointBMotor.setTargetPosition(-1183);
        robot.jointBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.jointBMotor.setPower(-0.5);

        while (opModeIsActive() && robot.jointBMotor.isBusy()) {}

        directDrive(DRIVE_SPEED, -7);

        sleep(15000);


//        switch (tagOfInterest.id) {
//            case TAG_1:
//                turnToHeading(TURN_SPEED, 90);
//                holdHeading(TURN_SPEED, 90, 0.5);
//
//                driveStraight(DRIVE_SPEED, 20, 90);
//                holdHeading(TURN_SPEED, 90, 0.5);
//                break;
//            case TAG_3:
//                turnToHeading(TURN_SPEED, -90);
//                holdHeading(TURN_SPEED, -90, 0.5);
//
//                driveStraight(DRIVE_SPEED, 20, -90);
//                holdHeading(TURN_SPEED, -90, 0.5);
//                break;
//            default:
//                break;
//        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    public void directDrive(double driveSpeed, int inches) {
        if (inches < 0) {
            driveSpeed = -driveSpeed;
        }

        robot.topLeftMotor.setTargetPosition(robot.topLeftMotor.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches));
        robot.topRightMotor.setTargetPosition(robot.topRightMotor.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches));
        robot.bottomLeftMotor.setTargetPosition(robot.bottomLeftMotor.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches));
        robot.bottomRightMotor.setTargetPosition(robot.bottomRightMotor.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches));

        robot.topLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bottomLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bottomRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.topLeftMotor.setPower(driveSpeed);
        robot.topRightMotor.setPower(driveSpeed);
        robot.bottomLeftMotor.setPower(driveSpeed);
        robot.bottomRightMotor.setPower(driveSpeed);

        while (opModeIsActive() && robot.topLeftMotor.isBusy() && robot.topRightMotor.isBusy() && robot.bottomLeftMotor.isBusy() && robot.bottomRightMotor.isBusy()) {}

        robot.topLeftMotor.setPower(0);
        robot.topRightMotor.setPower(0);
        robot.bottomLeftMotor.setPower(0);
        robot.bottomRightMotor.setPower(0);

        robot.topLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveByRatio(double maxDriveSpeed, double y, int trInchUnit) {
        double scale = 1 / (y + 1);
        double yscale = y * scale;
        int startPos = topRightMotor.getCurrentPosition();

        topLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeftMotor.setPower((yscale - scale) * maxDriveSpeed);
        topRightMotor.setPower((yscale + scale) * maxDriveSpeed);
        bottomLeftMotor.setPower((yscale + scale) * maxDriveSpeed);
        bottomRightMotor.setPower((yscale - scale) * maxDriveSpeed);

        while (opModeIsActive() && topRightMotor.getCurrentPosition() < startPos + trInchUnit * COUNTS_PER_INCH) {
            telemetry.addData("Moving by Scale", y);
            telemetry.update();
        }

        topLeftMotor.setPower(0);
        topRightMotor.setPower(0);
        bottomLeftMotor.setPower(0);
        bottomRightMotor.setPower(0);
    }

    /**
     *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = (int) (topLeftMotor.getCurrentPosition() + bottomLeftMotor.getCurrentPosition() / 2) + moveCounts;
            rightTarget = (int) (topRightMotor.getCurrentPosition() + bottomRightMotor.getCurrentPosition() / 2) + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            topLeftMotor.setTargetPosition(leftTarget);
            bottomLeftMotor.setTargetPosition(leftTarget);
            topRightMotor.setTargetPosition(rightTarget);
            bottomRightMotor.setTargetPosition(rightTarget);

            topLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            topRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (topLeftMotor.isBusy() && bottomLeftMotor.isBusy() && topRightMotor.isBusy() && bottomRightMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            topLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            topRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        topLeftMotor.setPower(leftSpeed);
        bottomLeftMotor.setPower(leftSpeed);
        topRightMotor.setPower(rightSpeed);
        bottomRightMotor.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      (int) (topLeftMotor.getCurrentPosition() + bottomLeftMotor.getCurrentPosition() / 2),
                    (int) (topRightMotor.getCurrentPosition() + bottomRightMotor.getCurrentPosition() / 2));
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}
