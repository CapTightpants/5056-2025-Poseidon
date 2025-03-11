package frc.robot.subsystems;

import java.util.function.ToDoubleBiFunction;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.Vars.Tuning;
import frc.robot.Vars.Tuning.kAimingRotations;

public class ReefLimelight extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;
    private final Pigeon2 m_gyro;
    private final ShuffleboardTab m_sensorsTab = Shuffleboard.getTab("Sensors");
    /**
     * 
     * @param driveSubsystem The drive subsystem used.
     */
    public ReefLimelight(DriveSubsystem driveSubsystem, Pigeon2 gyro) {
        m_driveSubsystem = driveSubsystem;
        m_gyro = gyro;
        m_sensorsTab.add("Rotation", 0);
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
        int aprilTagId = (int) LimelightHelpers.getFiducialID("limelight-reef");
        kAimingRotations targetRotation;
        switch (aprilTagId) {
            case 7, 18 :
                targetRotation = kAimingRotations.Front;
                break;
                case 6, 19 :
                targetRotation = kAimingRotations.FrontLeft;
                break;
            case 11, 20:
                targetRotation = kAimingRotations.BackLeft;
                break;
            case 10, 21:
                targetRotation = kAimingRotations.Back;
                break;
            case 9, 22:
                targetRotation = kAimingRotations.BackRight;
                break;
            case 8, 17:
                targetRotation = kAimingRotations.FrontRight;
                break;
            default:
                targetRotation = kAimingRotations.Front;
                break;
        }

        m_driveSubsystem.drive(
            (limelightA - Tuning.kAimingTargetA) * Tuning.kAimingProportionalA, 
            (limelightX - Tuning.kAimingTargetX) * Tuning.kAimingProportionalX, 
            (robotYaw - targetRotation.RotationDeg) * Tuning.kAimingProportionalRotation, false);
        // pidControllerX.close();
        // pidControllerY.close();
    }

    public void periodic() {
        SmartDashboard.putNumber("Rotation", m_gyro.getRotation2d().getDegrees());
    }
}
