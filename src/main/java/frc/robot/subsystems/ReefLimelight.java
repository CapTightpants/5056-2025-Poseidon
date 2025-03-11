package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class ReefLimelight extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;
    /**
     * 
     * @param driveSubsystem The drive subsystem used.
     */
    public ReefLimelight(DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
    }
    
    /**
     * 
     * @param direction The desired coral side to score; false for left, true for right.
     */
    public void alignRobot(boolean direction) {
        PIDController pidControllerX = new PIDController(.1, 0, 0);
        PIDController pidControllerY = new PIDController(.1, 0, 0);
        pidControllerX.setSetpoint(direction ? 0 : 3);
        pidControllerY.setSetpoint(3.75);
        double limelightX = LimelightHelpers.getTXNC("limelight-reef");
        double limelightY = LimelightHelpers.getTA("limelight-reef");

        m_driveSubsystem.drive(pidControllerY.calculate(limelightY) * .5, pidControllerX.calculate(limelightX) * .2, 0, false);
        pidControllerX.close();
        pidControllerY.close();
    }
}
