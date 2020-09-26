package org.firstinspires.ftc.teamcode.misc.DataTypes;

public class WheelPowerConfig {
    public double lf,rf,rb,lb;

    public WheelPowerConfig(double lfPower, double rfPower, double rbPower, double lbPower){
        lf = lfPower;
        rf = rfPower;
        rb = rbPower;
        lb = lbPower;
    }

    public  void clamp(){
        //clamp all between -1 and 1
        double max1 = Math.max(lf, rf);
        double max2 = Math.max(rb, lb);
        double max = Math.max(max1, max2);
        lf /= max;
        rf /= max;
        rb /= max;
        lb /= max;
    }

    public WheelPowerConfig Add(WheelPowerConfig WPC){
        return new WheelPowerConfig(lf+WPC.lf, rf+WPC.rf, rb+WPC.rb, lb+WPC.lb);
    }

}