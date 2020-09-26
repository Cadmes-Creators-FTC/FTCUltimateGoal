package org.firstinspires.ftc.teamcode.misc.DataTypes;

public class WheelPosition {
    public double lf,rf,rb,lb;

    public WheelPosition(double lfPos, double rfPos, double rbPos, double lbPos){
        lf = lfPos;
        rf = rfPos;
        rb = rbPos;
        lb = lbPos;
    }

    public WheelPosition Subtract(WheelPosition pos){
        return new WheelPosition(lf+pos.lf, rf+pos.rf, rb+pos.rb, lb+pos.lb);
    }
}
