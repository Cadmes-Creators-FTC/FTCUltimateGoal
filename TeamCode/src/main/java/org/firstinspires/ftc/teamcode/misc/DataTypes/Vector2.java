package org.firstinspires.ftc.teamcode.misc.DataTypes;

public class Vector2 {
    public double x, y;

    public Vector2(double xIn, double yIN){
        x = xIn;
        y = yIN;
    }

    public void clamp(double n){
        double max = Math.max(Math.abs(x), Math.abs(y));
        x /= max;
        y /= max;
        x *= n;
        y *= n;
    }

    public Vector2 Add(Vector2 pos){
        return new Vector2(x+pos.x, y+pos.y);
    }
    public Vector2 Subtract(Vector2 pos){
        return new Vector2(x-pos.x, y-pos.y);
    }
}
