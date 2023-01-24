package org.firstinspires.ftc.teamcode.autorewritten;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.helper.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.helper.Constants.*;
import org.firstinspires.ftc.teamcode.helper.Robot;
import org.firstinspires.ftc.teamcode.helper.Utils;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


@Autonomous(name="AniAuto", group="_Rewritten")
public class AniAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Robot robot = new Robot(); // Instantiate Robot Class to Access Drive Motors

    // Vision Members
    OpenCvCamera camera; // Instantiate Camera
    AprilTagDetectionPipeline aprilTagDetectionPipeline; // Instantiate AprilTagDetection Pipeline

    // Lens Intrinsics (In Pixels)
    double fx = 1430; // Focal Length X
    double fy = 1430; // Focal Length Y
    double cx = 480; // Center X
    double cy = 620; // Center Y

    // AprilTag Size
    double tagSize = 0.166; // Meters

    // AprilTag Detection Storage
    AprilTagDetection tagOfInterest = null;
    boolean detected = false;

    boolean cameraOpen = false;

    @Override
    public void runOpMode() {
        // Set Telemetry Display Format for Enhanced Display
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Report Initialization
        telemetry.addData("Status", Utils.fontText("Initializing...", "yellow"));
        telemetry.update();

        // Init Robot w/ Defaults
        robot.init(hardwareMap);
        // Update Drive Motors to Run with Encoders
        for (DcMotorEx motor : robot.driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280,960, OpenCvCameraRotation.UPRIGHT);
                cameraOpen = true;
            }
            @Override
            public void onError(int errorCode) {}
        });

        // Report Completed Initialization, Detect AprilTag, and Wait for Start
        while (!isStarted() && !isStopRequested()) {
            if (cameraOpen) {telemetry.addData("Status", Utils.fontText("Initialized!", "green"));}
            else {telemetry.addData("Status", Utils.fontText("Initialized, but Camera not Yet Open.", "#AAAA00"));}
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                tagOfInterest = currentDetections.get(0);
                detected = true;
                telemetry.addData("Detected", Utils.fontText(tagToPosition(tagOfInterest.id), "lime"));
            } else {
                if (detected) {
                    telemetry.addData("Detected", Utils.fontText(tagToPosition(tagOfInterest.id) + " (In Past)", "olive"));
                } else {
                    telemetry.addData("Detected", Utils.fontText("NONE", "red"));
                }
            }
            telemetry.addData("Heading", "%4.0f", getRawHeading());
            telemetry.addLine(Utils.fontText("Waiting for Start...", "navy", "+2"));

            telemetry.update();
            sleep(20);
        }

        // START
        runtime.reset();

        // Report Final Tag Detection
        if (tagOfInterest != null) {
            telemetry.addLine("Parking " + tagToPosition(tagOfInterest.id));
        } else {
            telemetry.addLine("No Tag Found, Defaulting to MIDDLE");
        }
        telemetry.update();

        // Close Camera Asynchronously
        camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                cameraOpen = false;
            }
        });


        // Begin Movement ---------------------------------

        // Push Cone into Terminal
        moveStraight(6, 0.8);
        turnLeft(90, 1.0);
        moveStraight(-6, 0.8);
        moveStraight(6, 0.8);
        turnRight(0, 0.8);
        // Navigate into Parking Area
        moveStraight(30, 0.8);
        moveStraight(-6, 0.8);
        turnLeft(90, 1.0);

        // Navigate into Appropriate Area
        switch (tagOfInterest.id) {
            // Left
            case 163:
                moveStraight(24, 1.0);
                break;
            // Right
            case 165:
                moveStraight(-24, 1.0);
                break;
            // Middle/Default
            default:
                break;
        }


//        // Push Cone into Terminal
//        turnRight(-90, 0.8);
//        moveStraight(48, 0.8);
//        moveStraight(-48, 0.8);
//        turnLeft(0, 0.8);
//        // Navigate into Parking Area
//        moveStraight(48, 0.8);
//        // Turn to Stack
//        turnRight(-90, 0.8);
//        // Open Arm and Move to Stack
//        moveStraight(48, 0.6);
//        // Retreat from Stack
//        moveStraight(-48, 0.6);
//        // Turn to Pole and Score
//        turnLeft(48, 0.6);
//        // Move Arm to Initial and Park
//        turnRight(0, 0.8);
//
//        // Navigate into Appropriate Area
//        switch (tagOfInterest.id) {
//            // Left
//            case 163:
//                turnLeft(90, 1.0);
//                moveStraight(48, 1.0);
//                break;
//            // Right
//            case 165:
//                turnRight(-90, 1.0);
//                moveStraight(48, 1.0);
//                break;
//            // Middle/Default
//            default:
//                break;
//        }

        // Safety Sleep
        sleep(1000);
    }

    private void moveStraight(int inches, double power) {
        int relative_target_position = (int) (Drivetrain.COUNTS_PER_INCH * inches);

        // Move Robot Backwards if Target is Behind
        if (inches < 0 && power < 0) {
            power = -power;
        }

        // Set Motors to Run
        for (DcMotorEx motor : robot.driveMotors) {
            motor.setTargetPosition(motor.getCurrentPosition() + relative_target_position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        // Run Motors
        for (DcMotorEx motor : robot.driveMotors) {
            motor.setPower(power);
        }
        // Wait While Motors are Moving
        while (opModeIsActive() && driveIsBusy()) {
            // TODO: Heading Correction
            telemetry.addData("Status", "Moving Straight...");
            telemetry.update();
        }
        // Stop Motors
        for (DcMotorEx motor : robot.driveMotors) {
            motor.setPower(0.0);
        }
    }

    // TODO: Add Overshoot Correction
    private void turnLeft(double heading, double power) {
        drivetrainToEncoders();

        double driveError = Math.abs(heading - getRawHeading());
        double drivePower = power / heading * driveError;

        while (opModeIsActive() && driveError > Drivetrain.TURNING_TOLERANCE) {
            robot.topLeftMotor.setPower(-drivePower);
            robot.topRightMotor.setPower(drivePower);
            robot.bottomLeftMotor.setPower(-drivePower);
            robot.bottomRightMotor.setPower(drivePower);

            driveError = Math.abs(heading - getRawHeading());
            drivePower = (drivePower > Drivetrain.MINIMUM_TURNING_SPEED) ? (power / heading * driveError) : Drivetrain.MINIMUM_TURNING_SPEED;

            telemetry.addData("Status", "Turning Left...");
            telemetry.addData("Error", driveError);
            telemetry.addData("Power", drivePower);
            telemetry.update();
        }
    }

    // TODO: Add Overshoot Correction
    private void turnRight(double heading, double power) {
        drivetrainToEncoders();

        double driveError = Math.abs(heading - getRawHeading());
        double drivePower = power / Math.abs(heading) * driveError;

        while (opModeIsActive() && driveError > Drivetrain.TURNING_TOLERANCE) {
            robot.topLeftMotor.setPower(drivePower);
            robot.topRightMotor.setPower(-drivePower);
            robot.bottomLeftMotor.setPower(drivePower);
            robot.bottomRightMotor.setPower(-drivePower);

            driveError = Math.abs(heading - getRawHeading());
            drivePower = (drivePower > Drivetrain.MINIMUM_TURNING_SPEED) ? (power / Math.abs(heading) * driveError) : Drivetrain.MINIMUM_TURNING_SPEED;

            telemetry.addData("Status", "Turning Right...");
            telemetry.addData("Error", driveError);
            telemetry.addData("Power", drivePower);
            telemetry.addData("Heading", getRawHeading());

            telemetry.update();
        }
    }

    /**
     * Returns true if Any Drive Motor is Busy
     */
    private boolean driveIsBusy() {
        return (robot.topLeftMotor.isBusy() || robot.topRightMotor.isBusy() || robot.bottomLeftMotor.isBusy() || robot.bottomRightMotor.isBusy());
    }

    /**
     * Returns the Current Robot Heading Along the Z Axis
     */
    private double getRawHeading() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Returns a String Representation of a given AprilTag ID
     */
    private String tagToPosition(int tag) {
        switch (tag) {
            case 163:
                return "LEFT";
            case 164:
                return "MIDDLE";
            case 165:
                return "RIGHT";
            default:
                return "UNKNOWN TAG";
        }
    }

    private void drivetrainToEncoders() {
        for (DcMotorEx motor : robot.driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
