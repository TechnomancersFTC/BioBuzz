package org.firstinspires.ftc.teamcode.constants;


public class autoshoot {
    public static double flywheeloffset = 0;
    public static double distoffset = 0;
    public static void SetFlywheelOffset(double offset) {
        flywheeloffset = offset;
    };

//    public static double flywheelSpeed(double goalDist) {
//        double d = goalDist + distoffset;
//        double speed =
//                0.0126474 * Math.pow(d, 2)
//                        + 2.51531 * d
//                        + 1166.72216;
//        return MathUtils.clamp(speed, 900, 4000) + flywheeloffset;
//    }
}