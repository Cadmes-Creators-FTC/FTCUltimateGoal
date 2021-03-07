package org.firstinspires.ftc.teamcode.Misc.DataTypes;

public class WheelPowerConfig {
    public double lf,rf,rb,lb;

    public WheelPowerConfig(double lfPower, double rfPower, double rbPower, double lbPower){
        lf = lfPower;
        rf = rfPower;
        rb = rbPower;
        lb = lbPower;
    }


    public void clamp(){
        //clamp all between -1 and 1
        double max1 = Math.max(Math.abs(lf), Math.abs(rf));
        double max2 = Math.max(Math.abs(rb), Math.abs(lb));
        double max = Math.max(max1, max2);

        if(max > 1) {
            lf /= max;
            rf /= max;
            rb /= max;
            lb /= max;
        }
    }
    public void scale(){
        //scale all between -1 and 1
        double max1 = Math.max(Math.abs(lf), Math.abs(rf));
        double max2 = Math.max(Math.abs(rb), Math.abs(lb));
        double max = Math.max(max1, max2);

        if(max < 1) {
            lf /= max;
            rf /= max;
            rb /= max;
            lb /= max;
        }
    }
    public void clampScale(){
        clamp();
        scale();
    }

    public double getAverage(){
        return (Math.abs(lf)+Math.abs(rf)+Math.abs(rb)+Math.abs(lb))/4;
    }

    public static WheelPowerConfig add(WheelPowerConfig wpc1, WheelPowerConfig wpc2){
        return new WheelPowerConfig(
                wpc1.lf+wpc2.lf,
                wpc1.rf+wpc2.rf,
                wpc1.rb+wpc2.rb,
                wpc1.lb+wpc2.lb
        );
    }
    public static WheelPowerConfig multiply(WheelPowerConfig wpc, double scaler){
        return new WheelPowerConfig(
                wpc.lf*scaler,
                wpc.rf*scaler,
                wpc.rb*scaler,
                wpc.lb*scaler
        );
    }
}