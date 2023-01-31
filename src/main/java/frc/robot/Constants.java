package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.swerve.SecondOrderSwerveKinematics;
import frc.lib.swerve.SwerveModuleConstants;
import java.util.List;
import java.util.TreeMap;

public final class Constants {
    public static final boolean competitionMode = false;

    public static final class GlobalConstants {
        public static final String CANIVORE_NAME = "CANivore";
        public static final int PCM_ID = 17;
        public static final double targetVoltage = 12.0; // Used for voltage compensation
    }

    public static final class ControllerConstants {
        public static final int LEFT_DRIVE_CONTROLLER = 0;
        public static final int RIGHT_DRIVE_CONTROLLER = 1;
        public static final int OPERATOR_CONTROLLER = 2;
    }

    public static final class TimesliceConstants {
        public static final double ROBOT_PERIODIC_ALLOCATION = 0.007;
        public static final double CONTROLLER_PERIOD = 0.010;

        public static final double DRIVETRAIN_PERIOD = 0.0025;
    }

    public static final class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);

        public static final double robotLengthWithBumpers = Units.inchesToMeters(30 + 7);

        /* X Placement constants from 6328 */
        public static final double outerX = Units.inchesToMeters(54.25);
        public static final double lowX =
                outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
        public static final double midX = outerX - Units.inchesToMeters(22.75);
        public static final double highX = outerX - Units.inchesToMeters(39.75);

        /* Z Placement constants from 6328 */
        public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
        public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
        public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
        public static final double highConeZ = Units.inchesToMeters(46.0);
        public static final double midConeZ = Units.inchesToMeters(34.0);

        public static class PlacementLocation {
            public Pose2d robotPlacementPose;
            public boolean isCone;

            public PlacementLocation(Pose2d poseAlignedWithEdge, double lengthOfRobotWithBumpers, boolean isCone) {
                var transformHybridToRobot = new Transform2d(
                        new Translation2d(lengthOfRobotWithBumpers / 2, 0), Rotation2d.fromDegrees(180));
                robotPlacementPose = poseAlignedWithEdge.transformBy(transformHybridToRobot);
                this.isCone = isCone;
            }

            public Pose3d getHybridPose() {
                return new Pose3d(lowX, robotPlacementPose.getY(), 0, new Rotation3d());
            }

            public Pose3d getMidPose() {
                return new Pose3d(midX, robotPlacementPose.getY(), isCone ? midConeZ : midCubeZ, new Rotation3d());
            }

            public Pose3d getHighPose() {
                return new Pose3d(highX, robotPlacementPose.getY(), isCone ? highConeZ : highCubeZ, new Rotation3d());
            }
        }

        public static final int numberOfNodeRows = 9;
        public static final double separationBetweenNodeRows = Units.inchesToMeters(22.0);
        public static final Pose2d firstPlacingPose = new Pose2d(outerX, Units.inchesToMeters(20.19), new Rotation2d());

        public static final boolean[] isCone = new boolean[] {true, false, true, true, false, true, true, false, true};

        // Store the locations we will score from on the field (for automatic placement)
        public static final PlacementLocation[] placingPoses = new PlacementLocation[numberOfNodeRows];

        static {
            for (int i = 0; i < numberOfNodeRows; i++) {
                placingPoses[i] = new PlacementLocation(
                        new Pose2d(
                                firstPlacingPose.getX(),
                                firstPlacingPose.getY() + i * separationBetweenNodeRows,
                                new Rotation2d()),
                        robotLengthWithBumpers,
                        isCone[i]);
            }
        }

        private static TreeMap<Double, PlacementLocation> locationMap = new TreeMap<>();

        static {
            for (PlacementLocation placementLocation : placingPoses) {
                locationMap.put(placementLocation.robotPlacementPose.getY(), placementLocation);
            }
        }

        /**
         * Finds the game piece placement area closest to the robot.
         * @param robotPose
         * @return The nearest placement location
         */
        public static PlacementLocation getNearestPlacementLocation(Pose2d robotPose) {
            double target = robotPose.getY();
            Double lowerYValue = locationMap.floorKey(target);
            Double upperYValue = locationMap.ceilingKey(target);

            // Account for the pose being below or above the range
            if (lowerYValue == null) return locationMap.get(upperYValue);
            else if (upperYValue == null) return locationMap.get(lowerYValue);

            boolean isLowerCloser = Math.abs(target - lowerYValue) < Math.abs(target - upperYValue);

            return isLowerCloser ? locationMap.get(lowerYValue) : locationMap.get(upperYValue);
        }

        public static final List<AprilTag> aprilTags = List.of(
                new AprilTag(
                        1,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(42.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        2,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(108.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        3,
                        new Pose3d(
                                Units.inchesToMeters(610.77),
                                Units.inchesToMeters(174.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        4,
                        new Pose3d(
                                Units.inchesToMeters(636.96),
                                Units.inchesToMeters(265.74),
                                Units.inchesToMeters(27.38),
                                new Rotation3d(0.0, 0.0, Math.PI))),
                new AprilTag(
                        5,
                        new Pose3d(
                                Units.inchesToMeters(14.25),
                                Units.inchesToMeters(265.74),
                                Units.inchesToMeters(27.38),
                                new Rotation3d())),
                new AprilTag(
                        6,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(174.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())),
                new AprilTag(
                        7,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(108.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())),
                new AprilTag(
                        8,
                        new Pose3d(
                                Units.inchesToMeters(40.45),
                                Units.inchesToMeters(42.19),
                                Units.inchesToMeters(18.22),
                                new Rotation3d())));

        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
                new AprilTagFieldLayout(aprilTags, fieldLength, fieldWidth);

        public static void setAprilTagOrigin() {
            APRIL_TAG_FIELD_LAYOUT.setOrigin(
                    DriverStation.getAlliance() == Alliance.Red
                            ? OriginPosition.kRedAllianceWallRightSide
                            : OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    public static final class LightsConstants {
        public static final int CANDLE_PORT = 59;
    }

    public static final class ArmConstants {
        // Length of the first arm segment
        public static final double arm1Length = Units.inchesToMeters(30);
        // Length of the second arm segment
        public static final double arm2Length = Units.inchesToMeters(30);

        // Length from starting joint to center of mass of respective arm segments
        public static final double arm1CenterOfMass = Units.inchesToMeters(15);
        public static final double arm2CenterOfMass = Units.inchesToMeters(15);

        // arm
        public static final double arm1Mass = Units.lbsToKilograms(15);
        public static final double arm2Mass = Units.lbsToKilograms(15);

        public static final double arm1MomentOfInertia = 2;
        public static final double arm2MomentOfInertia = 2;

        public static final double arm1GearRatio = 60;
        public static final double arm2GearRatio = 60;

        public static final double stallTorque = 4.69;
        public static final double stallCurrent = 257;
        public static final double freeSpeed = 630;

        public static final Transform3d armToRobot = new Transform3d();

        // Rotation relative to positive x-axis, counterclockwise-positive
        public static final Rotation2d arm1StartingAngle = Rotation2d.fromDegrees(100);

        // Rotation relative to first arm
        public static final Rotation2d arm2StartingAngle = Rotation2d.fromDegrees(-160);
    }

    public static final class VisionConstants {
        public static final double tapeWidth = Units.inchesToMeters(2.0);

        public static final String photonCameraName = "Global_Shutter_Camera";

        // Includes 3d transform from camera(s) to robot origin
        public static final Transform3d photonCameraToRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(10), Units.inchesToMeters(6 + 2), Units.inchesToMeters(-22)),
                new Rotation3d(0, 0, Math.PI));

        public static final Transform3d photonRobotToCamera = photonCameraToRobot.inverse();

        public static final double limelightHeight = Units.inchesToMeters(10);
        public static final double retroreflectiveHeight = Units.inchesToMeters(30);

        public static final Transform3d limelightCameraToRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(6), Units.inchesToMeters(-4)),
                new Rotation3d(0, 0, 0));

        public static final Transform3d limelightRobotToCamera = limelightCameraToRobot.inverse();
    }

    public static final class SwerveConstants extends Mk4SwerveConstants {}

    public static class Mk4SwerveConstants {
        // See https://github.com/Team364/BaseFalconSwerve for getting these values.

        public static final int PIGEON_PORT = 60;

        public static final double trackWidth = 0.5969;
        public static final double wheelBase = 0.5969;
        public static final double wheelDiameter = 0.10033;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.0; // 0.25
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SecondOrderSwerveKinematics swerveKinematics = new SecondOrderSwerveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.2;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS =
                (0.31382 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.15681 / 12);
        public static final double driveKA = 0;

        /* Angle Motor Characterization Values */
        public static final double angleKS = 0;
        // (0.368 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double angleKV = (0.234 / 12);
        public static final double angleKA = (0.003 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.968230454756032; // meters per second
        public static final double maxAngularVelocity = 11.771048567785275;

        /* Precise Driving Mode Values */
        public static final double preciseDrivingModeSpeedMultiplier = 0.2;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Drive Motor Inverts */
        public static final boolean driveMotorInvert = false;

        /* Drive Encoder Inverts */
        public static final boolean driveEncoderInvert = false;

        /* Angle Motor Inverts */
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 24;
            public static final double angleOffset = 241.179;
            //     public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 26;
            public static final double angleOffset = 167.432; // 348.135;
            //     public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 25;
            public static final double angleOffset = 159.609;
            //     public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 27;
            public static final double angleOffset = 268.506; // 94.043;
            //     public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static class Mk3SwerveConstants {
        // See https://github.com/Team364/BaseFalconSwerve for getting these values.

        public static final int PIGEON_PORT = 60;

        public static final double trackWidth = 0.5969;
        public static final double wheelBase = 0.5969;
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.0; // 0.25
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.86 / 1.0); // 6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.2;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS =
                (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.968230454756032; // meters per second
        public static final double maxAngularVelocity = 11.771048567785275;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Drive Motor Inverts */
        public static final boolean driveMotorInvert = false;

        /* Drive Encoder Inverts */
        public static final boolean driveEncoderInvert = false;

        /* Angle Motor Inverts */
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 17;
            public static final double angleOffset = -104.5898 + 360;
            // public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 19;
            public static final double angleOffset = 91.582; // 348.135;
            // public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 18;
            public static final double angleOffset = 37.617;
            // public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 20;
            public static final double angleOffset = -50.9766 + 360; // 94.043;
            // public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
