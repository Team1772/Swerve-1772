package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.Vision;

public class Shooter extends SubsystemBase {
    Vision vision;

    public Shooter(Vision vision){
        this.vision = vision;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Distance From Apriltag", vision.getDistanceFromAprilTag(1));
        SmartDashboard.putNumber("Apriltag X", vision.getXApriltag(1));
        SmartDashboard.putNumber("Apriltag Y", vision.getYApriltag(1));

    }
}
