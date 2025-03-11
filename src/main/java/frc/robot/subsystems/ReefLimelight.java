package frc.robot.subsystems;

import java.util.function.ToDoubleBiFunction;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Vars.Tuning;

public class ReefLimelight extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;
    private final Pigeon2 m_gyro;
    /**
     * 
     * @param driveSubsystem The drive subsystem used.
     */
    public ReefLimelight(DriveSubsystem driveSubsystem, Pigeon2 gyro) {
        m_driveSubsystem = driveSubsystem;
        m_gyro = gyro;
    }
    
    /**
     * 
     * @param direction The desired coral side to score; false for left, true for right.
     */
    public void alignRobot(boolean direction) {
        // PIDController pidControllerX = new PIDController(.1, 0, 0);
        // PIDController pidControllerY = new PIDController(.1, 0, 0);
        // pidControllerX.setSetpoint(direction ? 0 : 3);
        // pidControllerY.setSetpoint(3.75);
        double limelightX = LimelightHelpers.getTXNC("limelight-reef");
        double limelightA = LimelightHelpers.getTA("limelight-reef");
        double robotYaw = m_gyro.getRotation2d().getDegrees();

        m_driveSubsystem.drive(
            (limelightA - Tuning.kAimingTargetA) * Tuning.kAimingProportionalA, 
            (limelightX - Tuning.kAimingTargetX) * Tuning.kAimingProportionalX, 
            (robotYaw - Tuning.kAimingTargetRotation) * Tuning.kAimingProportionalRotation, false);
        // pidControllerX.close();
        // pidControllerY.close();
    }
}
