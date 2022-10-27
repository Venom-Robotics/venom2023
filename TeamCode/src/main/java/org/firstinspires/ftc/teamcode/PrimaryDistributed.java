package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helper.Robot;

@TeleOp(name="PrimaryDistributed")
public class PrimaryDistributed extends OpMode
{
    // Declare OpMode members
    Robot robot = new Robot(); // Instantiate Robot Class to Access Drive Motors
    private ElapsedTime runtime = new ElapsedTime(); // Keep Track of Time

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Initialize Robot
        robot.init(hardwareMap);

        // Update Telemetry to Reflect that Initialization is Complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        // Inform the User that The Arm is Initializing
        telemetry.addData("Status", "Initializing Arms...");
        telemetry.update();

        // Move the Robot to It's "0" Position
//        robot.jointAMotor.setTargetPosition(robot.JOINT_A_SETUP_TICKS);
//        robot.jointAMotor.setTargetPosition(robot.JOINT_B_SETUP_TICKS);
//        robot.jointAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.jointBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.jointAMotor.setPower(0.6);
//        robot.jointBMotor.setPower(0.6);
//
//        // Inform the User that the Arm is Moving
//        while (robot.jointAMotor.isBusy() || robot.jointBMotor.isBusy()) {
//            telemetry.addData("Status", "Setting Arms...");
//            telemetry.update();
//        }
//
//        // Reset Encoders and and Hold The Robot at The 0 Position
//        // REPLACE WITH "LynxResetMotorEncoderCommand"?
//        robot.jointAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.jointBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.jointAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.jointBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.jointAMotor.setTargetPosition(0);
//        robot.jointBMotor.setTargetPosition(0);
//        robot.jointAMotor.setPower(0.6);
//        robot.jointBMotor.setPower(0.6);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        int tl = robot.topLeftMotor.getCurrentPosition();
        int tr = robot.topRightMotor.getCurrentPosition();
        int bl = robot.bottomLeftMotor.getCurrentPosition();
        int br = robot.bottomRightMotor.getCurrentPosition();
        int Apos = robot.jointAMotor.getCurrentPosition();
        int Bpos = robot.jointBMotor.getCurrentPosition();
        //  ------Controller 1------

        // Drivetrain
        // Create Variables for Joystick Y Axes
        final double leftStickY = -gamepad1.left_stick_y;
        final double rightStickY = -gamepad1.right_stick_y;
        // Create Variables for Joystick X Axes / 2
        final double halfLeftStickX = gamepad1.left_stick_x/2;
        final double halfRightStickX = gamepad1.right_stick_x/2;
        // Calculate Movement for Each Wheel (and Discreetly Clamp Each Value to a Range of (-1.0, 1.0))
        double topLeftPower = leftStickY + halfLeftStickX + halfRightStickX;
        double topRightPower = rightStickY - halfLeftStickX - halfRightStickX;
        double bottomLeftPower = leftStickY - halfLeftStickX - halfRightStickX;
        double bottomRightPower = rightStickY + halfLeftStickX + halfRightStickX;
        // Dpad Inputs
//        topLeftPower += ((gamepad1.dpad_up) ? 1: 0) + ((gamepad1.dpad_down) ? -1: 0) + ((gamepad1.dpad_right) ? 1: 0) + ((gamepad1.dpad_left) ? -1: 0);
//        topRightPower += ((gamepad1.dpad_up) ? 1: 0) + ((gamepad1.dpad_down) ? -1: 0) + ((gamepad1.dpad_left) ? 1: 0) + ((gamepad1.dpad_right) ? -1: 0);
//        bottomLeftPower += ((gamepad1.dpad_up) ? 1: 0) + ((gamepad1.dpad_down) ? -1: 0) + ((gamepad1.dpad_left) ? 1: 0) + ((gamepad1.dpad_right) ? -1: 0);
//        bottomRightPower += ((gamepad1.dpad_up) ? 1: 0) + ((gamepad1.dpad_down) ? -1: 0) + ((gamepad1.dpad_right) ? 1: 0) + ((gamepad1.dpad_left) ? -1: 0);
        // Trigger Rotation
//        if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
//            topLeftPower += -0.2 - gamepad1.left_trigger + gamepad1.right_trigger;
//            topRightPower += -0.2 + gamepad1.left_trigger - gamepad1.right_trigger;
//            bottomLeftPower += -0.2 - gamepad1.left_trigger + gamepad1.right_trigger;
//            topRightPower += -0.2 + gamepad1.left_trigger - gamepad1.right_trigger;
//        }

        //  ------Controller 2------
        // Arm
        robot.jointAMotor.setPower(-gamepad2.left_stick_y/3);
        robot.jointBMotor.setPower(gamepad2.right_stick_y/3);

        if (gamepad1.a) {
            if (Bpos > -1800) {
                robot.jointBMotor.setPower(-0.5);
            } else if (Bpos < -1900) {
                robot.jointBMotor.setPower(0.5);
            }
        }

        // Claw Motor
        robot.clawMotor.setPower(gamepad1.left_trigger/3 + -gamepad1.right_trigger/3);

        // Claw Servo
        if (gamepad1.left_bumper) {
            robot.clawServo.setPower(1);
        } else if (gamepad1.right_bumper) {
            robot.clawServo.setPower(0);
        } else {
            robot.clawServo.setPower(1);
        }

        // Rotation Servo
        if (gamepad1.dpad_left) {
            robot.rotationServo.setPwmEnable();
            robot.rotationServo.setPower(0);
        } else if (gamepad1.dpad_right) {
            robot.rotationServo.setPwmEnable();
            robot.rotationServo.setPower(-1);
        } else {
            robot.rotationServo.setPwmDisable();
        }

        // Assign Calculated Values to Motors
        robot.topLeftMotor.setPower(topLeftPower);
        robot.topRightMotor.setPower(topRightPower);
        robot.bottomLeftMotor.setPower(bottomLeftPower);
        robot.bottomRightMotor.setPower(bottomRightPower);

        // Driver Station Telemetry
        telemetry.addData("Status", "Running...");
        telemetry.addLine(String.format("Top Left (%.2f) Top Right (%.2f)\nBottom Left (%.2f) Bottom Right (%.2f)",
                topLeftPower, topRightPower, bottomLeftPower, bottomRightPower));
        telemetry.addData("0", tl);
        telemetry.addData("1", tr);
        telemetry.addData("2", bl);
        telemetry.addData("3", br);
        telemetry.addData("a", Apos);
        telemetry.addData("b", Bpos);

        telemetry.addData("Run Time", runtime.toString());
//        telemetry.update(); // Automatically Updates Once Per Loop? https://gm0.org/en/latest/docs/software/tutorials/using-telemetry.html#updating-telemetry
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        // Updates Robot Status
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

//    private int[] determineAngles(int x, int y, int z) {
//        double convert = 180 / Math.PI;
////        double b = Math.atan2(y, x) * convert;
//        double l = Math.sqrt(x * x + y * y);
//        double h = Math.sqrt(l * l + z * z);
//        double phi = Math.atan(z/l) * convert;
//        double theta = Math.acos((h * (robot.ARM_LENGTH / robot.BONE1_LENGTH)) / robot.BONE1_LENGTH);
//
//        double a1 = phi + theta;
//        double a2 = phi - theta;
//
////        int x_ticks = (int) Math.round((double) drive.DRIVETRAIN_ENCODER_TICKS / 360 * b);
//        int y_ticks = (int) Math.round((double) robot.JOINT_ENCODER_TICKS / 360 * a1);
//        int z_ticks = (int) Math.round((double) robot.JOINT_ENCODER_TICKS / 360 * a2);
//
////        return new double[] {x_ticks, y_ticks, z_ticks};
//        return new int[] {y_ticks, z_ticks};
//
//    }

}