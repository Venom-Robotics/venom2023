package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


import java.util.List;
import java.util.Random;

/** Class that Declares and Initializes Robot Motors to Simplify Access */
public class Robot
{

    Random rand = new Random();
    // Constants
    public final double DRIVETRAIN_ENCODER_TICKS = 766.106508876;
    public final double JOINT_ENCODER_TICKS = 4005.55;
    public final double BONE1_LENGTH = 312;
    public final double BONE2_LENGTH = 406.4;
    public final double DPAD_SPEED = 0.7;

    // Declare Drivetrain members
    public DcMotor topLeftMotor = null;
    public DcMotor topRightMotor = null;
    public DcMotor bottomLeftMotor = null;
    public DcMotor bottomRightMotor = null;

    // Declare Manipulator Motors
    public DcMotorEx jointAMotor = null;
    public DcMotorEx jointBMotor = null;
    public DcMotorEx clawMotor = null;

    // Declare Manipulator Servos
    public CRServoImplEx clawServo = null;
    public CRServoImplEx rotationServo = null;

    public List<LynxModule> allHubs = null;

    // Create HardwareMap
    HardwareMap hwMap = null;

    public enum Direction {
        LEFT, RIGHT, NONE
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap _hardwareMap) {
        // Save Reference to Hardware Map
        hwMap = _hardwareMap;

        // Define and Initialize Motors
        topLeftMotor = hwMap.get(DcMotor.class, "front_left_motor");
        topRightMotor = hwMap.get(DcMotor.class, "front_right_motor");
        bottomLeftMotor = hwMap.get(DcMotor.class, "back_left_motor");
        bottomRightMotor = hwMap.get(DcMotor.class, "back_right_motor");
        jointAMotor = hwMap.get(DcMotorEx.class, "joint_a_motor");
        jointBMotor = hwMap.get(DcMotorEx.class, "joint_b_motor");
        clawMotor = hwMap.get(DcMotorEx.class, "claw_motor");

        // Set Hubs to Bulk Read
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//             Set Hub Color to White
            hub.setConstant(0xFFFFFF);
        }

        // Define and Initialize Servos
        clawServo = hwMap.get(CRServoImplEx.class, "claw_servo");
        rotationServo = hwMap.get(CRServoImplEx.class, "rotation_servo");

        // Widen Servo Ranges
//        clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        rotationServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

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
        jointAMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        jointBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void hubColor(int color) {
        for (LynxModule hub : allHubs) {
            hub.setConstant(color);
        }
    }

    public void randomHubColor() {
//        hubColor(rand.nextInt(16777215));
    }
}