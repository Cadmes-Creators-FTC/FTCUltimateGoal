package org.firstinspires.ftc.teamcode.misc.DataTypes;

import org.firstinspires.ftc.teamcode.misc.MathFunctions;

public class WheelPosition {
    public double lf,rf,rb,lb;

    public WheelPosition(double lfPos, double rfPos, double rbPos, double lbPos){
        lf = lfPos;
        rf = rfPos;
        rb = rbPos;
        lb = lbPos;
    }

    public void ToCM(double wheelCircumference, double ticksPerRot){
        lf = MathFunctions.TicksToCMs(lf, wheelCircumference, ticksPerRot);
        rf = MathFunctions.TicksToCMs(rf, wheelCircumference, ticksPerRot);
        rb = MathFunctions.TicksToCMs(rb, wheelCircumference, ticksPerRot);
        lb = MathFunctions.TicksToCMs(lb, wheelCircumference, ticksPerRot);
    }
    public void ToTicks(double wheelCircumference, double ticksPerRot){
        lf = MathFunctions.CMsToTicks(lf, wheelCircumference, ticksPerRot);
        rf = MathFunctions.CMsToTicks(rf, wheelCircumference, ticksPerRot);
        rb = MathFunctions.CMsToTicks(rb, wheelCircumference, ticksPerRot);
        lb = MathFunctions.CMsToTicks(lb, wheelCircumference, ticksPerRot);
    }

    public WheelPosition Subtract(WheelPosition pos){
        return new WheelPosition(lf+pos.lf, rf+pos.rf, rb+pos.rb, lb+pos.lb);
    }
}
