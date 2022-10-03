package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Class that Declares and Initializes Robot Motors to Simplify Access */
public class Robot
{
    // Constants
    public final int DRIVETRAIN_ENCODER_TICKS = 700; // UNTESTED
    public final int JOINT_ENCODER_TICKS = 4000;
    public final double BONE1_LENGTH = 312;
    public final double BONE2_LENGTH = 406.4;
    public final double ARM_LENGTH = BONE1_LENGTH + BONE2_LENGTH;
    public final int JOINT_A_SETUP_TICKS = 0;
    public final int JOINT_B_SETUP_TICKS = 0;

    // Declare Drivetrain members
    public DcMotor topLeftMotor = null;
    public DcMotor topRightMotor = null;
    public DcMotor bottomLeftMotor = null;
    public DcMotor bottomRightMotor = null;

    // Declare Manipulator Motors
    public DcMotor jointAMotor = null;
    public DcMotor jointBMotor = null;
    public DcMotor clawMotor = null;

    // Declare Manipulator Servos
    public Servo clawServo = null;
    public Servo rotationServo = null;

    // Create HardwareMap
    HardwareMap hwMap = null;

    // Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save Reference to Hardware Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        topLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        topRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        bottomLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        bottomRightMotor = hwMap.get(DcMotor.class, "back_right_motor");
        jointAMotor = hwMap.get(DcMotor.class, "joint_a_motor");
        jointBMotor = hwMap.get(DcMotor.class, "joint_b_motor");
        clawMotor = hwMap.get(DcMotor.class, "claw_motor");

        // Define and Initialize Servos
        clawServo = hwMap.get(Servo.class, "claw_servo");
        rotationServo = hwMap.get(Servo.class, "rotation_servo");

        // Set Motor Directions
        topLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        topRightMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        bottomRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset all Motor Powers
        topLeftMotor.setPower(0);
        topRightMotor.setPower(0);
        bottomLeftMotor.setPower(0);
        bottomRightMotor.setPower(0);
        jointAMotor.setPower(0);
        jointBMotor.setPower(0);
        clawMotor.setPower(0);

        // Set Motors to Brake
        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointAMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        jointBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Motor Encoders
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointAMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        jointBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Motors' Encoder Status
        topLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jointAMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        jointBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}