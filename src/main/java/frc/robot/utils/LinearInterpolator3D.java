package frc.robot.utils;

import java.util.Arrays;

public class LinearInterpolator3D {

    private double[][] table;
    private boolean initialized = false;

    public LinearInterpolator3D(double[][] data) {
        buildTable(data);
    }

    public boolean isInitialized() {
        return initialized;
    }

    private void buildTable(double[][] data) {
        int rows = data.length;
        if (rows < 1) {
            System.out.println("ERROR: LinearInterpolator3D needs at least one data point.");
            return;
        }
        int cols = data[0].length;
        if (cols != 5) {
            System.out.println("ERROR: LinearInterpolator3D requires 5 columns (3 inputs, 2 outputs)");
            return;
        }

        table = Arrays.copyOf(data, rows);
        Arrays.sort(table, (a, b) -> Double.compare(a[0], b[0]));
        initialized = true;
    }

    public double[] getInterpolatedValues(double x, double y, double z) {
        if (!initialized) {
            System.out.println("ERROR: LinearInterpolator3D is not initialized.");
            return new double[]{0.0, 0.0};
        }

        int index = 0;
        for (index = 0; index < table.length; index++) {
            if (table[index][0] >= x) {
                break;
            }
        }

        if (index >= table.length) {
            return new double[]{table[table.length - 1][3], table[table.length - 1][4]};
        }

        double[] high = table[index];
        if (index == 0 || (high[0] == x)) {
            return new double[]{high[3], high[4]};
        }
        double[] low = table[index - 1];

        double weightX = (x - low[0]) / (high[0] - low[0]);
        double weightY = (y - low[1]) / (high[1] - low[1]);
        double weightZ = (z - low[2]) / (high[2] - low[2]);
        
        double interpOut1 = low[3] + weightX * (high[3] - low[3]) + weightY * (high[3] - low[3]) + weightZ * (high[3] - low[3]);
        double interpOut2 = low[4] + weightX * (high[4] - low[4]) + weightY * (high[4] - low[4]) + weightZ * (high[4] - low[4]);
        
        return new double[]{interpOut1, interpOut2};
    }
}
