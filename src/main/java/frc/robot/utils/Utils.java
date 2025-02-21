package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotController;

public class Utils {

    private static final double LOW_VOLTAGE_THRESHOLD = 10.5; //
    private static Alert lowBatteryAlert = new Alert("Tensão da bateria está baixa!", Alert.AlertType.WARNING);

    public static void monitorBatteryVoltage() {
        double voltage = RobotController.getBatteryVoltage();

        if (voltage < LOW_VOLTAGE_THRESHOLD) {
            lowBatteryAlert.set(true);
        } else {
            lowBatteryAlert.set(false);  
        }
    }
}