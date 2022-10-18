package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helper.Robot;

@TeleOp(name="Primary")
public class Primary extends OpMode
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
        double topRightPower = rightStickY + -halfLeftStickX + -halfRightStickX;
        double bottomLeftPower = leftStickY + -halfLeftStickX + -halfRightStickX;
        double bottomRightPower = rightStickY + halfLeftStickX + halfRightStickX;
        // Dpad Inputs
        topLeftPower += ((gamepad1.dpad_up) ? 1: 0) + ((gamepad1.dpad_down) ? -1: 0) + ((gamepad1.dpad_right) ? 1: 0) + ((gamepad1.dpad_left) ? -1: 0);
        topRightPower += ((gamepad1.dpad_up) ? 1: 0) + ((gamepad1.dpad_down) ? -1: 0) + ((gamepad1.dpad_left) ? 1: 0) + ((gamepad1.dpad_right) ? -1: 0);
        bottomLeftPower += ((gamepad1.dpad_up) ? 1: 0) + ((gamepad1.dpad_down) ? -1: 0) + ((gamepad1.dpad_left) ? 1: 0) + ((gamepad1.dpad_right) ? -1: 0);
        bottomRightPower += ((gamepad1.dpad_up) ? 1: 0) + ((gamepad1.dpad_down) ? -1: 0) + ((gamepad1.dpad_right) ? 1: 0) + ((gamepad1.dpad_left) ? -1: 0);
        // Trigger Rotation
        if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) {
            topLeftPower += -0.2 + -gamepad1.left_trigger + gamepad1.right_trigger;
            topRightPower += -0.2 + gamepad1.left_trigger + -gamepad1.right_trigger;
            bottomLeftPower += -0.2 + -gamepad1.left_trigger + gamepad1.right_trigger;
            topRightPower += -0.2 + gamepad1.left_trigger + -gamepad1.right_trigger;
        }

        //  ------Controller 2------
        // Arm
        int Apos = robot.jointAMotor.getCurrentPosition();
        int Bpos = robot.jointBMotor.getCurrentPosition();

        if ((Apos != (int) (Apos + -gamepad2.left_stick_y * 10)) || (Bpos != (int) (Bpos + -gamepad2.right_stick_y * 10))) {
            telemetry.addLine("armIsMoving");
            robot.jointAMotor.setTargetPosition((int) (Apos + -gamepad2.left_stick_y * 10));
            robot.jointBMotor.setTargetPosition((int) (Bpos + -gamepad2.right_stick_y * 10));

            robot.jointAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.jointBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.jointAMotor.setVelocity(200);
            robot.jointBMotor.setVelocity(200);
        }

        // Claw Motor
        robot.clawMotor.setPower(gamepad2.left_trigger + -gamepad2.right_trigger);

        // Claw Servo
        if (gamepad2.left_bumper) {
            robot.clawServo.setPosition(0);
        } else if (gamepad2.right_bumper) {
            robot.clawServo.setPosition(1);
        }

        // Rotation Servo
        // 🤷

        // Assign Calculated Values to Motors
        robot.topLeftMotor.setPower(topLeftPower);
        robot.topRightMotor.setPower(topRightPower);
        robot.bottomLeftMotor.setPower(bottomLeftPower);
        robot.bottomRightMotor.setPower(bottomRightPower);

        // Driver Station Telemetry
        telemetry.addData("Status", "Running...");
        telemetry.addLine(String.format("Top Left (%.2f) Top Right (%.2f)\nBottom Left (%.2f) Bottom Right (%.2f)",
                topLeftPower, topRightPower, bottomLeftPower, bottomRightPower));
        telemetry.addData("0", robot.topLeftMotor.getCurrentPosition());
        telemetry.addData("1", robot.topRightMotor.getCurrentPosition());
        telemetry.addData("2", robot.bottomLeftMotor.getCurrentPosition());
        telemetry.addData("3", robot.bottomRightMotor.getCurrentPosition());
        telemetry.addData("range", robot.clawServo.getPwmRange());

        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.update(); // Automatically Updates Once Per Loop? https://gm0.org/en/latest/docs/software/tutorials/using-telemetry.html#updating-telemetry
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        // Updates Robot Status
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }

    private int[] determineAngles(int x, int y, int z) {
        double convert = 180 / Math.PI;
//        double b = Math.atan2(y, x) * convert;
        double l = Math.sqrt(x * x + y * y);
        double h = Math.sqrt(l * l + z * z);
        double phi = Math.atan(z/l) * convert;
        double theta = Math.acos((h * (robot.ARM_LENGTH / robot.BONE1_LENGTH)) / robot.BONE1_LENGTH);

        double a1 = phi + theta;
        double a2 = phi - theta;

//        int x_ticks = (int) Math.round((double) drive.DRIVETRAIN_ENCODER_TICKS / 360 * b);
        int y_ticks = (int) Math.round((double) robot.JOINT_ENCODER_TICKS / 360 * a1);
        int z_ticks = (int) Math.round((double) robot.JOINT_ENCODER_TICKS / 360 * a2);

//        return new double[] {x_ticks, y_ticks, z_ticks};
        return new int[] {y_ticks, z_ticks};

    }

}