package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helper.Constants.Drive;
import org.firstinspires.ftc.teamcode.helper.Constants.Presets;
import org.firstinspires.ftc.teamcode.helper.Robot;


@TeleOp(name = "Primary")
public class Primary extends OpMode {
    // Declare OpMode members
    Robot robot = new Robot(); // Instantiate Robot Class to Access Drive Motors
    private ElapsedTime runtime = new ElapsedTime();

    private int A_pos;
    private int B_pos;
    private int Claw_pos;

    private boolean moving_to_preset = false;
    private boolean dpad_move_to_preset = false;

    private boolean last_up = false;
    private boolean last_down = false;
    private int stack_position = 5;

    double[] drivetrainMotors;

    double negative_left_stick_y;
    double negative_right_stick_y;
    double half_left_stick_x;
    double half_right_stick_x;
    double dpad_up_down;
    double dpad_right;
    double dpad_left;
    double topLeftPower;
    double topRightPower;
    double bottomLeftPower;
    double bottomRightPower;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        // Initialize Robot
        robot.init(hardwareMap);
        runtime.reset();

//        for (DcMotorEx motor : robot.driveMotors) {
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        }

        // Initialize Telemetry
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // Update Telemetry to Reflect that Initialization is Complete
        telemetry.addData("Status", "<font color='green' font-weight=\"bold\">Initialized!</font>");
        telemetry.addLine("<font color='blue' font-weight=\"bold\">Waiting for Start...</font>");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        // Pre-Loop Logic
//         Reset Manipulator if Moving is Complete or 'Start' is Pressed
        if ((moving_to_preset && !manipulatorIsBusy()) || gamepad2.start) {
            resetManipulator();
            moving_to_preset = false;
        }
        // Update Stack Position Based on Dpad Up/Down
        if (gamepad2.dpad_up && !last_up && (stack_position < 5)) {
            stack_position++;
            dpad_move_to_preset = true;
        }
        if (gamepad2.dpad_down && !last_down && (stack_position > 1)) {
            stack_position--;
            dpad_move_to_preset = true;
        }
        last_up = gamepad2.dpad_up;
        last_down = gamepad2.dpad_down;
        // Redundantly Clamp stack_position
        stack_position = Math.max(Math.min(stack_position, 5), 1);

        // Cache Necessary Encoder Values
        A_pos = robot.jointAMotor.getCurrentPosition();
        B_pos = robot.jointBMotor.getCurrentPosition();
        Claw_pos = robot.clawMotor.getCurrentPosition();

        // ----- Controller 1 -----

        // Drivetrain
        drivetrainMotors = calculateDrivetrain();
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

        // ----- Controller 2 -----

        if (!moving_to_preset) {
            if (gamepad2.b) {
                runToPreset(Presets.Junctions.HIGH_A, Presets.Junctions.HIGH_B, Presets.Junctions.HIGH_C);
            } else if (gamepad2.y) {
                runToPreset(Presets.Junctions.MID_A, Presets.Junctions.MID_B, Presets.Junctions.MID_C);
            } else if (gamepad2.x) {
                runToPreset(Presets.Junctions.LOW_A, Presets.Junctions.LOW_B, Presets.Junctions.LOW_C);
            } else if (gamepad2.a) {
                runToPreset(Presets.Common.GROUND_A, Presets.Common.GROUND_B, Presets.Common.GROUND_C);
            }
            if (dpad_move_to_preset) {
                switch (stack_position) {
                    case 5:
                        runToPreset(Presets.Stack.STACK5_A, Presets.Stack.STACK5_B, Presets.Stack.STACK5_C);
                        dpad_move_to_preset = false;
                        break;
                    case 4:
                        runToPreset(Presets.Stack.STACK4_A, Presets.Stack.STACK4_B, Presets.Stack.STACK4_C);
                        dpad_move_to_preset = false;
                        break;
                    case 3:
                        runToPreset(Presets.Stack.STACK3_A, Presets.Stack.STACK3_B, Presets.Stack.STACK3_C);
                        dpad_move_to_preset = false;
                        break;
                    case 2:
                        runToPreset(Presets.Stack.STACK2_A, Presets.Stack.STACK2_B, Presets.Stack.STACK2_C);
                        dpad_move_to_preset = false;
                        break;
                    case 1:
                        runToPreset(Presets.Common.GROUND_A, Presets.Common.GROUND_B, Presets.Common.GROUND_C);
                        dpad_move_to_preset = false;
                        break;
                }
            }
        }

        if (!moving_to_preset) {
            // Arm
            robot.jointAMotor.setPower(-gamepad2.left_stick_y / 2 + (A_pos < 900 ? 0.1 : -0.2));
            robot.jointBMotor.setPower(gamepad2.right_stick_y / 2 - 0.1);
            // Claw Motor
            robot.clawMotor.setPower(-gamepad2.left_trigger / 2.2 + gamepad2.right_trigger / 2 + (Claw_pos > 50 ? -0.05 : 0.1));
        }

        // Driver Station Telemetry
        telemetry.addData("Status", "<font color='purple' font-weight=\"bold\">Running...</font>");
        telemetry.addData("Manipulator", moving_to_preset ? "<font color='red' font-weight=\"bold\">Running to Preset</font>" : "<font color='green' font-weight=\"bold\">Running with Joysticks</font>");
        telemetry.addLine("Stack Position: " + new String(new char[stack_position]).replace("\0", "▲"));
        telemetry.addLine("\tPositions:" +
                "\n\t\tA: " + A_pos +
                "\n\t\tB: " + B_pos +
                "\n\t\tC: " + Claw_pos);
        for (DcMotorEx motor : robot.driveMotors) {
            telemetry.addData(motor.getDeviceName(), motor.getCurrentPosition());
        }
    }
    //  telemetry.addLine("Run Time: " + runtime.toString());

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
        // Updates Robot Status
        telemetry.addData("Status", "<font color='red' font-weight=\"bold\">Stopped</font>");
        telemetry.update();
    }

    private double[] calculateDrivetrain() {
        // Pre-Calculated Stick and Button Values
        negative_left_stick_y = -gamepad1.left_stick_y;
        negative_right_stick_y = -gamepad1.right_stick_y;

        half_left_stick_x = gamepad1.left_stick_x / 2;
        half_right_stick_x = gamepad1.right_stick_x / 2;

        dpad_up_down = ((gamepad1.dpad_up) ? Drive.DPAD_SPEED : 0) + ((gamepad1.dpad_down) ? -Drive.DPAD_SPEED : 0);
        dpad_right = ((gamepad1.dpad_right) ? Drive.DPAD_SPEED : 0);
        dpad_left = ((gamepad1.dpad_left) ? Drive.DPAD_SPEED : 0);

        // Calculate Joystick Movement for Each Wheel (and Discreetly Clamp Each Value to a Range of (-1.0, 1.0))
        topLeftPower = negative_left_stick_y + half_left_stick_x + half_right_stick_x;
        topRightPower = negative_right_stick_y - half_left_stick_x - half_right_stick_x;
        bottomLeftPower = negative_left_stick_y - half_left_stick_x - half_right_stick_x;
        bottomRightPower = negative_right_stick_y + half_left_stick_x + half_right_stick_x;

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

        // Common Multiplier
        topLeftPower *= Drive.TOTAL_MULTIPLIER;
        topRightPower *= Drive.TOTAL_MULTIPLIER;
        bottomLeftPower *= Drive.TOTAL_MULTIPLIER;
        bottomRightPower *= Drive.TOTAL_MULTIPLIER;

        return new double[]{topLeftPower, topRightPower, bottomLeftPower, bottomRightPower};
    }

    private void runToPosition(DcMotor motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    private void runToPreset(int A, int B, int C) {
        moving_to_preset = true;
        runToPosition(robot.jointAMotor, A, (A > A_pos) ? 0.5 : -0.5);
        runToPosition(robot.jointBMotor, B, (B > B_pos) ? 0.5 : -0.5);
        runToPosition(robot.clawMotor, C, (C > Claw_pos) ? 0.5 : -0.5);
    }

    private void resetManipulator() {
        robot.jointAMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.jointBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private boolean manipulatorIsBusy() {
        return (robot.jointAMotor.isBusy() || robot.jointBMotor.isBusy() || robot.clawMotor.isBusy());
    }

}