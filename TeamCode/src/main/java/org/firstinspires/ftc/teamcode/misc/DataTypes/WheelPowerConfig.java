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
        double max1 = Math.max(Math.abs(lf), Math.abs(rf));
        double max2 = Math.max(Math.abs(rb), Math.abs(lb));
        double max = Math.max(max1, max2);

        if(max > 1) {
            lf /= max;
            rf /= max;
              lb /= max;
        }
    }

    public WheelPowerConfig Add(WheelPowerConfig WPC){
        return new WheelPowerConfig(lf+WPC.lf, rf+WPC.rf, rb+WPC.rb, lb+WPC.lb);
    }

}