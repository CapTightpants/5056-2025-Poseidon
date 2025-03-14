package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Vars.Tuning;
import frc.robot.Vars.Tuning.kAimingPositions;
import frc.robot.Vars.Tuning.kAimingRotations;

public class ReefLimelight extends SubsystemBase {
    private final DriveSubsystem m_driveSubsystem;
    private final Pigeon2 m_gyro;
    private final ShuffleboardTab m_sensorsTab = Shuffleboard.getTab("Sensors");
    private final GenericEntry gyroEntry = m_sensorsTab.add("Gyro", 0).getEntry();
    /**
     * 
     * @param driveSubsystem The drive subsystem used.
     */
    public ReefLimelight(DriveSubsystem driveSubsystem, Pigeon2 gyro) {
        m_driveSubsystem = driveSubsystem;
        m_gyro = gyro;
    }
    
    /**
     *  While called, aligns the robot with an apriltag for scoring or intaking.
     * 
     * @param direction The desired coral side to score, defined by the kAimingPositions enum
     * @param teleop If true, sends commands directly to the DriveSubsystem. If false, uses the PPOverride methods.
     */
    public void alignRobot(kAimingPositions position, boolean teleop) {
        String limelightName = position == kAimingPositions.Intake ? "limelight-intake" : "limelight-reef";
        double limelightX = LimelightHelpers.getTXNC(limelightName);
        double limelightA = LimelightHelpers.getTA(limelightName);
        int aprilTagId = (int) LimelightHelpers.getFiducialID(limelightName);
        double robotYaw = m_gyro.getRotation2d().getDegrees();
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

        if (teleop) {
            m_driveSubsystem.drive(
                (limelightA - position.TargetA) * position.ProportionalA,
                (limelightX - position.TargetX) * position.ProportionalX,
                (targetRotation.RotationDeg - robotYaw) * Tuning.kAimingProportionalRotation,
                false
            );
        } else {
            PPHolonomicDriveController.overrideXFeedback(() -> {
                return (limelightA - position.TargetA) * position.ProportionalA;
            });
            PPHolonomicDriveController.overrideYFeedback(() -> {
                return (limelightX - position.TargetX) * position.ProportionalX;
            });
            PPHolonomicDriveController.overrideRotationFeedback(() -> {
                return (targetRotation.RotationDeg - robotYaw) * Tuning.kAimingProportionalRotation;
            });
        }
    }

    public void periodic() {
        gyroEntry.setDouble(m_gyro.getRotation2d().getDegrees());
    }

    public void clearFeedbackOverride() {
        PPHolonomicDriveController.clearFeedbackOverrides();
    }
}
