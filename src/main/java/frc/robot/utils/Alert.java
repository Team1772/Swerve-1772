package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Alert {

    public enum AlertType {
        ERROR, WARNING, INFO
    }

    private final String message;
    private final AlertType type;
    private boolean isActive;
    private final Timer timer; 

    public Alert(String message, AlertType type) {
        this.message = message;
        this.type = type;
        this.isActive = false;
        this.timer = new Timer(); 
    }

    public void set(boolean active) {
        if (active != isActive) {
            isActive = active;
            if (isActive) {
                SmartDashboard.putString("ALERTA", type + ": " + message);
                System.out.println(type + ": " + message);
                timer.reset(); 
                timer.start();
            }
        }
    }

    public void update() {
        if (isActive && timer.hasElapsed(30.0)) { 
            isActive = false;
            SmartDashboard.putString("ALERTA", "");
            System.out.println("Alerta removido: " + message);
            timer.stop();
        }
    }
}
