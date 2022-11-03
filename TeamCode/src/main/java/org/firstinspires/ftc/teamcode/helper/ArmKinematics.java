package org.firstinspires.ftc.teamcode.helper;


public class ArmKinematics {
    static final double SCALE = 4005.55 / 360;

    public static double[] forwardKinematics(double L1, double L2, double a1, double a2) {
        a1 *= Math.PI/180;
        a2 *= Math.PI/180;
        double cosa1 = Math.cos(a1);
        double sina1 = Math.sin(a1);
        double cosa2 = Math.cos(a2);
        double sina2 = Math.sin(a2);
        return new double[] {L1 * cosa1 + L2 * cosa1 * cosa2 + L2 * sina1 * sina2, sina1 + L2 * sina1 * cosa2 + L2 * cosa1 * sina2}; // x,y
    }

    public static double[] inverseKinematics(double L1, double L2, double x, double y) {
        double a2 = Math.acos((Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(L1, 2) - Math.pow(L2, 2))/(2 * L1 * L2));
        double L1L2cosa2 = L1 + L2 * Math.cos(a2);
        double L2sina2 = L2 * Math.sin(a2);
        double a1 = Math.atan((y * (L1L2cosa2) - x * L2sina2)/(x * (L1L2cosa2) + y * L2sina2));
        return new double[] {Math.round(a1 * 180/Math.PI) * SCALE, Math.round(a2 * 180/Math.PI) * SCALE};
    }
}
