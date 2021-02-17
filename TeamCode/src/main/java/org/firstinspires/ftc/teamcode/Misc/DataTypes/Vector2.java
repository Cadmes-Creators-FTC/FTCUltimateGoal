package org.firstinspires.ftc.teamcode.Misc.DataTypes;

public class  Vector2 {
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

    public Matrix toMatrix(){
        return new Matrix(new double[][]{
                { x, y }
        });
    }


    public static Vector2 add(Vector2 v1, Vector2 v2){
        return new Vector2(v1.x+v2.x, v1.y+v2.y);
    }
    public static Vector2 subtract(Vector2 v1, Vector2 v2){
        return new Vector2(v1.x-v2.x, v1.y-v2.y);
    }
    public static Vector2 multiply(Vector2 vector, double scaler){
        return new Vector2(vector.x*scaler, vector.y*scaler);
    }
    public static Vector2 divide(Vector2 vector, double scaler){
        return new Vector2(vector.x/scaler, vector.y/scaler);
    }
}
