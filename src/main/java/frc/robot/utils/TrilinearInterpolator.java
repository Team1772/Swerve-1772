package frc.robot.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class TrilinearInterpolator {

    private double[][] table;
    private boolean initialized = false;

    public TrilinearInterpolator(double[][] data) {
        buildTable(data);
    }

    public boolean isInitialized() {
        return initialized;
    }

    private void buildTable(double[][] data) {
        if (data.length < 1) {
            System.out.println("ERROR: TrilinearInterpolator needs at least one data point.");
            return;
        }
        if (data[0].length != 5) {
            System.out.println("ERROR: Data should have exactly 5 columns (x, y, z, angle, rotations)");
            return;
        }

        table = Arrays.copyOf(data, data.length);

        // Ordena os dados por x, depois y, depois z para facilitar a busca
        Arrays.sort(table, Comparator.comparingDouble((double[] a) -> a[0])
                .thenComparingDouble(a -> a[1])
                .thenComparingDouble(a -> a[2]));

        initialized = true;
    }

    public double[] getInterpolatedValues(double x, double y, double z) {
        if (!initialized) {
            System.out.println("ERROR: TrilinearInterpolator is not initialized.");
            return new double[]{0.0, 0.0};
        }

        // Encontra os 8 pontos mais próximos
        double[][] corners = findNearestPoints(x, y, z);
        if (corners.length < 2) {
            System.out.println("ERROR: Not enough data points for interpolation.");
            return new double[]{0.0, 0.0};
        }

        // Aplica interpolação trilinear (ou bilinear/unidimensional se houver menos de 8 pontos)
        double interpolatedAngle = interpolate(x, y, z, corners, 3);
        double interpolatedRotations = interpolate(x, y, z, corners, 4);

        return new double[]{interpolatedAngle, interpolatedRotations};
    }

    private double[][] findNearestPoints(double x, double y, double z) {
        List<double[]> nearest = new ArrayList<>();

        for (double[] point : table) {
            if (nearest.size() < 8) {
                nearest.add(point);
            } else {
                // Mantém sempre os 8 pontos mais próximos
                nearest.sort(Comparator.comparingDouble(p -> distance3D(p, x, y, z)));
                if (distance3D(point, x, y, z) < distance3D(nearest.get(7), x, y, z)) {
                    nearest.set(7, point);
                }
            }
        }

        return nearest.toArray(new double[0][0]);
    }

    private double interpolate(double x, double y, double z, double[][] points, int valueIndex) {
        double sumWeights = 0.0;
        double weightedSum = 0.0;

        for (double[] point : points) {
            double weight = 1.0 / (distance3D(point, x, y, z) + 1e-6); // Evita divisão por zero
            sumWeights += weight;
            weightedSum += weight * point[valueIndex];
        }

        return weightedSum / sumWeights;
    }

    private double distance3D(double[] p, double x, double y, double z) {
        return Math.sqrt(Math.pow(p[0] - x, 2) + Math.pow(p[1] - y, 2) + Math.pow(p[2] - z, 2));
    }
}
