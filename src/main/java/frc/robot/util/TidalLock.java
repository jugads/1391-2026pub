package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.VisionConstants.TidalLockConstants.*;
public class TidalLock {
    PIDController pid = new PIDController(kP, kI, kD);

    public TidalLock() {
        
    }

    public double getOutput(double robotAngle, double desiredAngle, double dy) {
        SmartDashboard.putNumber("Tidallock dy multi", dy * kVELOCITY_MULTIPLIER);
        return pid.calculate(robotAngle, desiredAngle) + dy * kVELOCITY_MULTIPLIER;
    }
}