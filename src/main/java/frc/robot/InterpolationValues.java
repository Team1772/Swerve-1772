package frc.robot;

import frc.robot.utils.LinearInterpolator;
import frc.robot.utils.LinearInterpolator3D;

public class InterpolationValues {
    public static final double[][] SHOOTER_ANGLE_ARRAY = {
            { 9.6, 30 },
            { 4, 30 },
            { 3, 30 },
            { -0.3, 31 },
            { -0.6, 32 },
            { -3, 32 },
            { -4.6, 33 },
            { -6.3, 35 },
            { -8.6, 35 },
            { -9, 35 },
            { -13, 36 },
            { -14.7, 37 },
            { -17, 38 },
            { -19.1, 39 }
    };

    public static final LinearInterpolator SHOOTER_ANGLE_INTERPOLATOR = new LinearInterpolator(SHOOTER_ANGLE_ARRAY);

    private static final double[][] TEST_DATA = {
            { 1.0, 2.0, 3.0, 10.0, 20.0 },
            { 2.0, 3.0, 4.0, 20.0, 30.0 },
            { 3.0, 4.0, 5.0, 30.0, 40.0 },
            { 4.0, 5.0, 6.0, 40.0, 50.0 }
    };

    public static final LinearInterpolator3D TEST_LINEAR3D = new LinearInterpolator3D(TEST_DATA);
}
