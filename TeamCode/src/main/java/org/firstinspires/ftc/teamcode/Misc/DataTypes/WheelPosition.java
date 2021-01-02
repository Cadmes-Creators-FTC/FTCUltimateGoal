package org.firstinspires.ftc.teamcode.Misc.DataTypes;

import org.firstinspires.ftc.teamcode.Misc.MathFunctions;

public class  WheelPosition {
    public double lf,rf,rb,lb;

    public WheelPosition(double lfPos, double rfPos, double rbPos, double lbPos){
        lf = lfPos;
        rf = rfPos;
        rb = rbPos;
        lb = lbPos;
    }

    public void toCM(double wheelCircumference, double ticksPerRot){
        lf = MathFunctions.ticksToCMs(lf, wheelCircumference, ticksPerRot);
        rf = MathFunctions.ticksToCMs(rf, wheelCircumference, ticksPerRot);
        rb = MathFunctions.ticksToCMs(rb, wheelCircumference, ticksPerRot);
        lb = MathFunctions.ticksToCMs(lb, wheelCircumference, ticksPerRot);
    }
    public void toTicks(double wheelCircumference, double ticksPerRot){
        lf = MathFunctions.centimetersToTicks(lf, wheelCircumference, ticksPerRot);
        rf = MathFunctions.centimetersToTicks(rf, wheelCircumference, ticksPerRot);
        rb = MathFunctions.centimetersToTicks(rb, wheelCircumference, ticksPerRot);
        lb = MathFunctions.centimetersToTicks(lb, wheelCircumference, ticksPerRot);
    }


    public static WheelPosition subtract(WheelPosition pos1, WheelPosition pos2){
        return new WheelPosition(
                pos1.lf-pos2.lf,
                pos1.rf-pos2.rf,
                pos1.rb-pos2.rb,
                pos1.lb-pos2.lb
        );
    }
}
