package org.firstinspires.ftc.teamcode.misc.DataTypes;

public class WheelPowerConfig {
    public double lf,rf,rb,lb,in;

    public WheelPowerConfig(double lfPower, double rfPower, double rbPower, double lbPower, double inPower){
        lf = lfPower;
        rf = rfPower;
        rb = rbPower;
        lb = lbPower;
        in = inPower;

    }


    public  void clamp(){
        //clamp all between -1 and 1
        double max1 = Math.max(lf, rf);
        double max2 = Math.max(rb, lb);
        //double max3 = Math.max(in);
        double max = Math.max(max1, max2);
        lf /= max;
        rf /= max;
        rb /= max;
        lb /= max;
        

    }

    public WheelPowerConfig Add(WheelPowerConfig WPC){
        return new WheelPowerConfig(lf+WPC.lf, rf+WPC.rf, rb+WPC.rb, lb+WPC.lb, in+WPC.in);
    }

}