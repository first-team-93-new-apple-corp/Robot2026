package frc.robot.util;

import java.util.ArrayList;

public class RollingAverageDouble {
    private final int windowSize;
    private final ArrayList<Double> values;

    public RollingAverageDouble(int windowSize) {
        this.windowSize = windowSize;
        this.values = new ArrayList<>();
    }

    public double getAverage() {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.size();
    }

    public void addValue(double value) {
        values.add(value);
        if (values.size() > windowSize) {
            values.remove(0);
        }
    }

    public void clear() {
        values.clear();
    }
}
