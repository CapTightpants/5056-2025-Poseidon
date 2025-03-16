package frc.robot;

import frc.robot.Constants.Setpoints;

public final class Vars {
    public static class Positions {
        public static final class RobotStates {
            public static Setpoints.kLiftPosition kLiftState;
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
        public static double kAimingTargetRotation = .01;
        public static double kAimingProportionalRotation = .015;
        public static double kAimingMaxStrafeSpeed = .1;
        public static double kAimingMaxRotateSpeed = .3;
        public static double kAimingMinStrafeSpeed = -.1;
        public static double kAimingMinRotateSpeed = -.3;

        /**
         * The target rotation in degrees for the robot to face.
         */
        public static enum kAimingRotations {
            Front(0.0),
            FrontLeft(-60.0),
            BackLeft(-120.0),
            Back(180.0),
            BackRight(120.0),
            FrontRight(60.0);

            public double RotationDeg;

            kAimingRotations(double RotationDeg) {
                this.RotationDeg = RotationDeg;
            }
        }

        public static enum kAimingPositions {
            CoralLeft(-.3, -.02, 3.3, 7.5),
            CoralRight(-.3, -.02, 3.3, -15),
            Intake(-.01, -.02, 4.4, -8);

            public final double ProportionalA;
            public final double ProportionalX;
            public final double TargetA;
            public final double TargetX;

            kAimingPositions(double ProportionalA, double ProportionalX, double TargetA, double TargetX) {
                this.ProportionalA = ProportionalA;
                this.ProportionalX = ProportionalX;
                this.TargetA = TargetA;
                this.TargetX = TargetX;
            }
        }
    }
}
