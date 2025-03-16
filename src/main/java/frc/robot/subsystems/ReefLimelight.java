package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
    private final ShuffleboardTab m_teleopTab = Shuffleboard.getTab("Teleop");
    private final GenericEntry rotateEntry = m_teleopTab.add("Auto Rotate Enabled", true)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry();
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
        boolean autoRotate = rotateEntry.getBoolean(true);

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
                position == kAimingPositions.Intake
                    ? 0
                    : coerceInRange(
                        (limelightA - position.TargetA) * position.ProportionalA,
                        Tuning.kAimingMinStrafeSpeed,
                        Tuning.kAimingMaxStrafeSpeed
                    ),
                coerceInRange(
                    (limelightX - position.TargetX) * position.ProportionalX,
                    Tuning.kAimingMinStrafeSpeed,
                    Tuning.kAimingMaxStrafeSpeed
                ),
                // 0,
                (position == kAimingPositions.Intake || !autoRotate)
                    ? 0
                    : coerceInRange(
                        (targetRotation.RotationDeg - robotYaw) * Tuning.kAimingProportionalRotation,
                        Tuning.kAimingMinRotateSpeed,
                        Tuning.kAimingMaxRotateSpeed
                    ),
                false
            );
        } else {
            PPHolonomicDriveController.overrideXFeedback(() -> {
                return coerceInRange(
                    (limelightA - position.TargetA) * position.ProportionalA,
                    Tuning.kAimingMinStrafeSpeed, 
                    Tuning.kAimingMaxStrafeSpeed
                    );
            });
            PPHolonomicDriveController.overrideYFeedback(() -> {
                return coerceInRange(
                    (limelightX - position.TargetX) * position.ProportionalX, 
                    Tuning.kAimingMinStrafeSpeed,
                    Tuning.kAimingMaxStrafeSpeed
                );
            });
            // PPHolonomicDriveController.overrideRotationFeedback(() -> {
            //     return (targetRotation.RotationDeg - robotYaw) * Tuning.kAimingProportionalRotation;
            // });
        }
    }

    public void clearFeedbackOverride() {
        PPHolonomicDriveController.clearFeedbackOverrides();
    }

    /**
     * Coerces an input to be within the given range.
     * @return The coerced value.
     * @param Input The value to coerce.
     * @param Minimum The lowest desired value.
     * @param Maximum The highest desired value.
     */
    public double coerceInRange(double Input, double Minimum, double Maximum) {
        if (Input < Minimum) {
            return Minimum;
        }
        else if (Input > Maximum) {
            return Maximum;
        }
        else {
            return Input;
        }
    }
}
