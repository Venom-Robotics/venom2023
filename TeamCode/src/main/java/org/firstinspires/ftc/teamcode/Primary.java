package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.Robot;

import java.util.Random;

@TeleOp(name="Primary")
public class Primary extends OpMode
{
    // Declare OpMode members
    Robot robot = new Robot(); // Instantiate Robot Class to Access Drive Motors
    private ElapsedTime runtime = new ElapsedTime();

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Initialize Robot
        robot.init(hardwareMap);
        runtime.reset();

        // Initialize Telemetry
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        robot.hubColor(0x0000FF);

        // Update Telemetry to Reflect that Initialization is Complete
        telemetry.addData("Status", "<font color='green' font-weight=\"bold\">Initialized!</font>");
        telemetry.addLine("<font color='blue' font-weight=\"bold\">Waiting for Start...</font>");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        robot.randomHubColor();
    }

    @Override
    public void start() {
        runtime.reset();
        robot.hubColor(0xFF00FF);
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Grab Necessary Encoder Values
        int A_pos = this.robot.jointAMotor.getCurrentPosition();
//        int B_pos = this.robot.jointBMotor.getCurrentPosition();
        int Claw_pos = this.robot.clawMotor.getCurrentPosition();

        // ------------------------
        // ----- Controller 1 -----
        // ------------------------

        // Drivetrain
        double[] drivetrainMotors = calculateDrivetrain();
        robot.topLeftMotor.setPower(drivetrainMotors[0]);
        robot.topRightMotor.setPower(drivetrainMotors[1]);
        robot.bottomLeftMotor.setPower(drivetrainMotors[2]);
        robot.bottomRightMotor.setPower(drivetrainMotors[3]);

        // Claw Servo
        if (gamepad1.a) {
            robot.clawServo.setPower(0);
        } else {
            robot.clawServo.setPower(1);
        }

        // Rotation Servo
        if (gamepad1.left_bumper) {
            robot.rotationServo.setPwmEnable();
            robot.rotationServo.setPower(1);
        } else if (gamepad1.right_bumper) {
            robot.rotationServo.setPwmEnable();
            robot.rotationServo.setPower(-1);
        } else {
            robot.rotationServo.setPwmDisable();
        }

        // ------------------------
        // ----- Controller 2 -----
        // ------------------------

        // Arm
        robot.jointAMotor.setPower(-gamepad2.left_stick_y/2 + (A_pos < 900 ? 0.1 : -0.2));
        robot.jointBMotor.setPower(gamepad2.right_stick_y/2 - 0.1);

        // Claw Motor
        robot.clawMotor.setPower(-gamepad2.left_trigger/2.2 + gamepad2.right_trigger/2 + (Claw_pos > 50 ? -0.05 : 0.1));

        // Driver Station Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "<font color='purple' font-weight=\"bold\">Running...</font>");
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        robot.hubColor(0xFF0000);
        // Updates Robot Status
        telemetry.addData("Status", "<font color='red' font-weight=\"bold\">Stopped</font>");
        telemetry.update();
    }

    private double[] calculateDrivetrain() {
        // Pre-Calculated Stick and Button Values
        double negative_left_stick_y = -gamepad1.left_stick_y/1.25;
        double negative_right_stick_y = -gamepad1.right_stick_y/1.25;

        double half_left_stick_x = gamepad1.left_stick_x/2;
        double half_right_stick_x = gamepad1.right_stick_x/2;

        double dpad_up_down = ((gamepad1.dpad_up) ? robot.DPAD_SPEED: 0) + ((gamepad1.dpad_down) ? -robot.DPAD_SPEED: 0);
        double dpad_right = ((gamepad1.dpad_right) ? robot.DPAD_SPEED: 0);
        double dpad_left = ((gamepad1.dpad_left) ? robot.DPAD_SPEED: 0);

        // Calculate Movement for Each Wheel (and Discreetly Clamp Each Value to a Range of (-1.0, 1.0))
        double topLeftPower = negative_left_stick_y + half_left_stick_x + half_right_stick_x;
        double topRightPower = negative_right_stick_y - half_left_stick_x - half_right_stick_x;
        double bottomLeftPower = negative_left_stick_y - half_left_stick_x - half_right_stick_x;
        double bottomRightPower = negative_right_stick_y + half_left_stick_x + half_right_stick_x;

        // Dpad Inputs
        topLeftPower += dpad_up_down + dpad_right - dpad_left;
        topRightPower += dpad_up_down - dpad_right + dpad_left;
        bottomLeftPower += dpad_up_down - dpad_right + dpad_left;
        bottomRightPower += dpad_up_down + dpad_right - dpad_left;

        // Trigger Rotation
        topLeftPower += -gamepad1.left_trigger + gamepad1.right_trigger;
        topRightPower += gamepad1.left_trigger - gamepad1.right_trigger;
        bottomLeftPower += -gamepad1.left_trigger + gamepad1.right_trigger;
        bottomRightPower += gamepad1.left_trigger - gamepad1.right_trigger;

        return new double[]{topLeftPower, topRightPower, bottomLeftPower, bottomRightPower};
    }

}