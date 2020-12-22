package org.firstinspires.ftc.teamcode.misc.DataTypes;

public class Matrix {
    public int rows, columns;
    public double[][] matrix;

    public Matrix(int t_rows, int t_columns, double[][] t_matrix) {
        rows = t_rows;
        columns = t_columns;

        // make all 0 matrix if input is null
        if (t_matrix == null){
            t_matrix = new double[][]{{}};
            for (int i = 0; i < t_rows; i++) {
                for (int j = 0; j < t_columns; j++) {
                    t_matrix[i][j] = 0;
                }
            }
        }

        matrix = t_matrix;
    }

    public void set(double[][] t_matrix){
        matrix = t_matrix;
    }

    public static Matrix scale(Matrix a, double scaler){
        Matrix b = new Matrix(a.rows, a.columns, null);

        for(int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.columns; j++) {
                b.matrix[i][j] = a.matrix[i][j] * scaler;
            }
        }

        return b;
    }
    public static Matrix multiply(Matrix a, Matrix b){
        Matrix c = new Matrix(a.rows, b.columns, null);

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
