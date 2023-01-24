package org.firstinspires.ftc.teamcode.helper;

public final class Constants {
    public final class Drivetrain {
        public final static double WHEEL_DIAMETER = 2.95275590552; // 75 mm in Inches (REV Mecanums)
        public final static double COUNTS_PER_MOTOR_REVOLUTION = 766.106508876; // Ticks (Ultraplanetary HD Hex)
        public final static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public final static double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REVOLUTION / WHEEL_CIRCUMFERENCE;

        public static final double MINIMUM_TURNING_SPEED = 0.02; // Motor Power (RUN_USING_ENCODER)
        public static final double TURNING_TOLERANCE = 1.0; // Angle
    }

    public final static class Presets {
        public final static class Junctions {
            public static final int HIGH_A = 50;
            public static final int HIGH_B = -2710;
            public static final int HIGH_C = 107;

            public static final int MID_A = 56;
            public static final int MID_B = -2146;
            public static final int MID_C = 38;

            public static final int LOW_A = 61;
            public static final int LOW_B = -1725;
            public static final int LOW_C = 50;
        }

        public static class Stack {
            public static final int STACK5_A = -24;
            public static final int STACK5_B = -1524;
            public static final int STACK5_C = 26;

            public static final int STACK4_A = -24;
            public static final int STACK4_B = -1524;
            public static final int STACK4_C = 26;

            public static final int STACK3_A = -24;
            public static final int STACK3_B = -1524;
            public static final int STACK3_C = 26;

            public static final int STACK2_A = -52;
            public static final int STACK2_B = -1143;
            public static final int STACK2_C = 26;
        }

        public static final class Common {
            public static final int GROUND_A = 20;
            public static final int GROUND_B = -1126;
            public static final int GROUND_C = 30;
        }
    }

    public static final class Drive {
        public static double DPAD_SPEED = 0.7;
        public static double TOTAL_MULTIPLIER = 1.0;
    }
}
