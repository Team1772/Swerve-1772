package frc.robot.utils;

public class Main {
    public static void main(String[] args) {
        System.out.println("Hello, World!");

        double[][] data = {
            {0.0, 0.0, 0.0, 30.0, 10.0},
            {0.0, 0.0, 1.0, 35.0, 12.0},
            {0.0, 1.0, 0.0, 40.0, 14.0},
            {0.0, 1.0, 1.0, 42.0, 15.0},
            {1.0, 0.0, 0.0, 45.0, 16.0},
            {1.0, 0.0, 1.0, 47.0, 17.0},
            {1.0, 1.0, 0.0, 48.0, 18.0},
            {1.0, 1.0, 1.0, 50.0, 20.0}
        };
        
        TrilinearInterpolator interpolator = new TrilinearInterpolator(data);
        double[] result = interpolator.getInterpolatedValues(0.5, 0.5, 0.5);

        System.out.println("Ângulo do Shooter: " + result[0]);
        System.out.println("Rotações do Motor: " + result[1]);
    }
}
