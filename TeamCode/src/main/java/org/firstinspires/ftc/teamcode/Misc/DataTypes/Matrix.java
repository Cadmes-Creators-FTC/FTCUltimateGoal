package org.firstinspires.ftc.teamcode.Misc.DataTypes;

import java.util.Arrays;

public class Matrix {
    public int rows, columns;
    public double[][] matrix;

    public Matrix(int t_rows, int t_columns){
        rows = t_rows;
        columns = t_columns;

        matrix = new double[rows][columns];
        for (double[] row: matrix)
            Arrays.fill(row, 0.0);
    }
    public Matrix(double[][] t_matrix){
        if(t_matrix == null)
            return;

        rows = t_matrix.length;
        columns = t_matrix[0] != null ? t_matrix[0].length : 0;
        matrix = t_matrix;
    }

    public void set(double[][] t_matrix){
        matrix = t_matrix;
    }

    public Vector2 toVector2(){
        return new Vector2(matrix[0][0], matrix[0][1]);
    }


    public static Matrix scale(Matrix a, double scaler){
        for(int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.columns; j++) {
                a.matrix[i][j] *= scaler;
            }
        }

        return a;
    }
    public static Matrix multiply(Matrix a, Matrix b){
        Matrix c = new Matrix(a.rows, b.columns);

        for (int i = 0; i < a.rows; i++) { // aRow
            for (int j = 0; j < b.columns; j++) { // bColumn
                for (int k = 0; k < a.columns; k++) { // aColumn
                    c.matrix[i][j] += a.matrix[i][k] * b.matrix[k][j];
                }
            }
        }

        return c;
    }
}
