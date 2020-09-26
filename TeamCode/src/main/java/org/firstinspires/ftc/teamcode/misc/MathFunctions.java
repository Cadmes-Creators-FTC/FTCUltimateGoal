package org.firstinspires.ftc.teamcode.misc;

public class MathFunctions {
    //convert cm to encoder ticks
    public static int CMToTicks(double CM){
        double tickCM = 1120 / 26.928;
        tickCM *= (100.0/141.0);
        long ticks = Math.round(tickCM * CM);

        return (int) ticks;
    }

    public static double clambAngleDegrees(double angle){
        while (angle < -180)
            angle += 360;
        while (angle > 180)
            angle -= 360;

        return angle;
    }
}

