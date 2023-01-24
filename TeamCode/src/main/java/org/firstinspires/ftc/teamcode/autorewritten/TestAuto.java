package org.firstinspires.ftc.teamcode.autorewritten;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helper.Robot;

@TeleOp(name="TestAuto", group="_Rewritten")
public class TestAuto extends LinearOpMode {

    // Declare OpMode members.
    Robot robot = new Robot(); // Instantiate Robot Class to Access Drive Motors
    double power = 0.5;
    boolean last_a = false;
    boolean last_y = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Init Robot Defaults
        robot.init(hardwareMap);

        for (DcMotorEx motor : robot.driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                power += 0.01;
            } else if (gamepad1.dpad_down) {
                power -= 0.01;
            }

            if (last_y && !gamepad1.y) {power += 0.01;};
            if (last_a && !gamepad1.a) {power -= 0.01;};
            last_y = gamepad1.y;
            last_a = gamepad1.a;


            robot.topLeftMotor.setPower(power);
            robot.topRightMotor.setPower(-power);
            robot.bottomLeftMotor.setPower(power);
            robot.bottomRightMotor.setPower(-power);

            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
