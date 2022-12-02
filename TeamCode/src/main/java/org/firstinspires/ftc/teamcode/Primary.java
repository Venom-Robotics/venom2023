package org.firstinspires.ftc.teamcode;


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
        // Inform the User that The Arm is Initializing
        telemetry.addData("Status", "Initializing Arms...");
        telemetry.update();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        int A_pos = this.robot.jointAMotor.getCurrentPosition();
        int B_pos = this.robot.jointBMotor.getCurrentPosition();
        int Claw_pos = this.robot.clawMotor.getCurrentPosition();
        // ------------------------
        // ----- Controller 1 -----
        // ------------------------

        // Drivetrain --------------------------------------
        // Calculate Movement for Each Wheel (and Discreetly Clamp Each Value to a Range of (-1.0, 1.0))
        double topLeftPower = -gamepad1.left_stick_y/1.25 + gamepad1.left_stick_x/2 + gamepad1.right_stick_x/2;
        double topRightPower = -gamepad1.right_stick_y/1.25 - gamepad1.left_stick_x/2 - gamepad1.right_stick_x/2;
        double bottomLeftPower = -gamepad1.left_stick_y/1.25 - gamepad1.left_stick_x/2 - gamepad1.right_stick_x/2;
        double bottomRightPower = -gamepad1.right_stick_y/1.25 + gamepad1.left_stick_x/2 + gamepad1.right_stick_x/2;

        // Dpad Inputs
        topLeftPower += ((gamepad1.dpad_up) ? 0.7: 0) + ((gamepad1.dpad_down) ? -0.7: 0) + ((gamepad1.dpad_right) ? 0.7: 0) + ((gamepad1.dpad_left) ? -0.7: 0);
        topRightPower += ((gamepad1.dpad_up) ? 0.7: 0) + ((gamepad1.dpad_down) ? -0.7: 0) + ((gamepad1.dpad_left) ? 0.7: 0) + ((gamepad1.dpad_right) ? -0.7: 0);
        bottomLeftPower += ((gamepad1.dpad_up) ? 0.7: 0) + ((gamepad1.dpad_down) ? -0.7: 0) + ((gamepad1.dpad_left) ? 0.7: 0) + ((gamepad1.dpad_right) ? -0.7: 0);
        bottomRightPower += ((gamepad1.dpad_up) ? 0.7: 0) + ((gamepad1.dpad_down) ? -0.7: 0) + ((gamepad1.dpad_right) ? 0.7: 0) + ((gamepad1.dpad_left) ? -0.7: 0);

        // Trigger Rotation
        topLeftPower += -gamepad1.left_trigger + gamepad1.right_trigger;
        topRightPower += gamepad1.left_trigger + -gamepad1.right_trigger;
        bottomLeftPower += -gamepad1.left_trigger + gamepad1.right_trigger;
        bottomRightPower += gamepad1.left_trigger + -gamepad1.right_trigger;

        // Assign Calculated Values to Motors
        robot.topLeftMotor.setPower(topLeftPower);
        robot.topRightMotor.setPower(topRightPower);
        robot.bottomLeftMotor.setPower(bottomLeftPower);
        robot.bottomRightMotor.setPower(bottomRightPower);
        // -------------------------------------------------

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
        telemetry.addData("A", A_pos);
        telemetry.addData("B", B_pos);
        telemetry.addData("C", Claw_pos);
        telemetry.addData("Status", "Running...");
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        // Updates Robot Status
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }


}