package frc.robot;

import frc.robot.Constants.Setpoints;

public final class Vars {
    public static class Positions {
        public static final class RobotStates {
            public static Setpoints.kLiftPosition kLiftState;
            public static Setpoints.kHarpoonPosition kHarpoonState;
        }
    }

    public static class Throttles {
        public static double kAlgaeIntakeThrottle = 1;
        public static double kCoralIntakeThrottle = 0.5;
        public static double kHarpoonThrottle = 1;

        public static double kCreep = 0.45;
        public static double kNormal = 0.7;
        public static double kBoost = 1;
        public static double kDriveThrottle = kNormal;
    }

    public static class Tuning {
        public static double kAimingProportionalA = .05;
        public static double kAimingProportionalX = .02;
        public static double kAimingProportionalRotation = .05;
        public static double kAimingTargetA = .05;
        public static double kAimingTargetX = .02;
        public static double kAimingTargetRotation = .01;
        /**
         * The target rotation in degrees for the robot to face.
         */
        public static enum kAimingRotations {
            Front(0.0),
            FrontLeft(60.0),
            BackLeft(120.0),
            Back(180.0),
            BackRight(-60.0),
            FrontRight(-120.0);

        public final double RotationDeg;

        kAimingRotations(double RotationDeg) {
            this.RotationDeg = RotationDeg;
        }
        }
    }
}
