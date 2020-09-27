package org.firstinspires.ftc.teamcode.misc;

public class MathFunctions {

    //region translate ticks and cm
    public static double TicksToCMs(double ticks, double wheelCircumference, double ticksPerRot){
        double cmPerTick = wheelCircumference/ticksPerRot;
        return cmPerTick*ticks;
    }
    public static double CMsToTicks(double cm, double wheelCircumference, double ticksPerRot){
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
}

