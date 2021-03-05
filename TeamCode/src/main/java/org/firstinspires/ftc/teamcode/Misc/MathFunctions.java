package org.firstinspires.ftc.teamcode.Misc;

public class MathFunctions {

    //region translate ticks and cm
    public static double ticksToCMs(double ticks, double wheelCircumference, double ticksPerRot){
        double cmPerTick = wheelCircumference/ticksPerRot;
        return cmPerTick*ticks;
    }
    public static double centimetersToTicks(double cm, double wheelCircumference, double ticksPerRot){
        double ticksPerCM = ticksPerRot/wheelCircumference;
        return ticksPerCM*cm;
    }
    //endregion

    //region angles
    public static double clambAngleDegrees(double angle){
        while (angle < -180)
            angle += 360;
        while (angle > 180)
            angle -= 360;

        return angle;
    }
    //endregion

    public static double xOverAbsX(double x){
        return x/Math.abs(x);
    }
}

