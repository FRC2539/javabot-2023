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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.logging.Logger;
import frc.lib.swerve.SecondOrderSwerveKinematics;
import frc.lib.swerve.SwerveModuleConstants;
import java.util.Arrays;
import java.util.List;
import java.util.TreeMap;

public final class Constants {
    public static final boolean competitionMode = false;

    public static final class GlobalConstants {
        public static final String CANIVORE_NAME = "CANivore";
        public static final int PCM_ID = 12; // 19;
        public static final double targetVoltage = 12.0; // Used for voltage compensation

        public static final double batteryVoltageThreshold = 12.3;

        public static final double minimumPressure = 100; // PSI
        public static final double maximumPressure = 120; // try 120
    }

    public static final class ControllerConstants {
        public static final int LEFT_DRIVE_CONTROLLER = 0;
        public static final int RIGHT_DRIVE_CONTROLLER = 1;
        public static final int OPERATOR_CONTROLLER = 2;
    }

    public static final class FieldConstants {
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5);

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
                var transformHybridToRobot =
                        new Transform2d(new Translation2d(lengthOfRobotWithBumpers / 2, 0), Rotation2d.fromDegrees(0));
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
                        SwerveConstants.lengthWithBumpers,
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
            double target =
                    DriverStation.getAlliance() == Alliance.Red ? fieldWidth - robotPose.getY() : robotPose.getY();
            Double lowerYValue = locationMap.floorKey(target);
            Double upperYValue = locationMap.ceilingKey(target);

            // Account for the pose being below or above the range
            if (lowerYValue == null) return flipPlacementLocation(locationMap.get(upperYValue));
            else if (upperYValue == null) return flipPlacementLocation(locationMap.get(lowerYValue));

            boolean isLowerCloser = Math.abs(target - lowerYValue) < Math.abs(target - upperYValue);

            var finalPlacementLocation = isLowerCloser ? locationMap.get(lowerYValue) : locationMap.get(upperYValue);

            return flipPlacementLocation(finalPlacementLocation);
        }

        public static PlacementLocation flipPlacementLocation(PlacementLocation placementLocation) {
            PlacementLocation resultPlacementLocation = placementLocation;

            if (DriverStation.getAlliance() == Alliance.Red) {
                var pose = placementLocation.robotPlacementPose;
                resultPlacementLocation = new PlacementLocation(
                        new Pose2d(firstPlacingPose.getX(), fieldWidth - pose.getY(), pose.getRotation()),
                        SwerveConstants.lengthWithBumpers,
                        placementLocation.isCone);
            }

            Logger.log("/SwerveDriveSubsystem/PlacementPose", resultPlacementLocation.robotPlacementPose);
            Logger.log("/SwerveDriveSubsystem/isCone", resultPlacementLocation.isCone);

            return resultPlacementLocation;
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
        public static final int CANDLE_PORT = 17;
    }

    public static final class IntakeConstants {
        public static final int intakeMotor = 16;

        public static final int positionForwardChannel = 3;
        public static final int positionReverseChannel = 6;

        public static final int shootingForwardChannel = 7;
        public static final int shootingReverseChannel = 0; // 1;
    }

    public static final class ArmConstants {
        public static final double maxVoltage = 8;

        // Length of the first arm segment
        public static final double arm1Length = Units.inchesToMeters(40.5);
        // Length of the second arm segment
        public static final double arm2Length = Units.inchesToMeters(30);

        // Length from starting joint to center of mass of respective arm segments
        public static final double arm1CenterOfMass = Units.inchesToMeters(20.06);
        public static final double arm2CenterOfMass = Units.inchesToMeters(20.5);

        public static final double arm1Mass = Units.lbsToKilograms(11.47);
        public static final double arm2Mass = Units.lbsToKilograms(8.16);

        public static final double arm1MomentOfInertia = 2.3;
        public static final double arm2MomentOfInertia = 2.44;

        public static final double arm1GearRatio = 64 * 2; // Gearbox is 64:1, sprocket is 2:1
        public static final double arm2GearRatio = 64 * 2;

        public static final double arm1MinimumAngle = Math.toRadians(36.36);
        public static final double arm1MaximumAngle = Math.toRadians(90);

        public static final double arm2MinimumAngle = Math.toRadians(-174);
        public static final double arm2MaximumAngle = Math.toRadians(-56.24);

        public static final double stallTorque = 4.69;
        public static final double stallCurrent = 257;
        public static final double freeSpeed = 6380;

        // Adjust these to offset for gas shocks
        public static final double arm1kS = 0.1;
        public static final double arm2kS = 0.1;

        public static final double arm1kSDeadband = 0.05;
        public static final double arm2kSDeadband = 0.05;

        public static final Transform3d robotToArm =
                new Transform3d(new Translation3d(0, 0, Units.inchesToMeters(8)), new Rotation3d());

        // Rotation relative to positive x-axis, counterclockwise-positive
        public static final Rotation2d arm1StartingAngle = Rotation2d.fromDegrees(103.35);

        // Rotation relative to first arm
        public static final Rotation2d arm2StartingAngle = Rotation2d.fromDegrees(-180);

        public static final double angularTolerance = Math.toRadians(4);
        public static final double gripperAngularTolerance = Math.toRadians(8);

        public static final int mastMotorPort = 8;
        public static final int boomMotorPort = 9;
        public static final int wristMotorPort = 13;

        // Store the channels for through bore encoders
        public static final int mastEncoderChannel = 0;
        public static final int boomEncoderChannel = 1;
        public static final int gripperEncoderChannel = 2;

        // Store offsets for the through bore encoders (measured - offset = absolute position)
        // Measurement Guide
        //
        // - Orient mast arm facing straight upwards (+90 degrees relative to straight forward)
        // - Orient boom arm fully inside of mast arm (-180 degrees relative to mast arm)
        // - Orient gripper fully inside of boom arm (180 degrees relative to boom arm)
        //
        // When measuring values, replace the 0 in the offset with the value measured
        public static final double mastEncoderOffset = -2.108 + Math.PI / 2;
        public static final double boomEncoderOffset = -(-1.865) + -Math.PI;
        public static final double gripperEncoderOffset = -(0.907) + Math.PI;

        // Set these to -1 to invert the encoder measurements (find offsets again)
        public static final int mastEncoderMultiplier = 1;
        public static final int boomEncoderMultiplier = 1;
        public static final int gripperEncoderMultiplier = 1;

        // Set these to true if a positive voltage makes the arm turn clockwise (we want ccw+)
        public static final boolean invertMastMotor = false;
        public static final boolean invertBoomMotor = true;
        public static final boolean invertWristMotor = true;

        public static final double placementHeightOffset = 0.1;
    }

    public static final class GripperConstants {
        public static final double length = Units.inchesToMeters(9.625);
        public static final double centerOfMass = Units.inchesToMeters(2);

        public static final double mass = Units.lbsToKilograms(4.21);

        public static final double momentOfInertia = 0.02;

        public static final Rotation2d startingAngle = Rotation2d.fromDegrees(180);

        // This is old
        // public static final double minimumAngle = Math.toDegrees(-90);
        // public static final double maximumAngle = Math.toDegrees(180);

        // weird implementation. basically does not allow values between -90 and -110
        public static final double minimumAngle = Math.toDegrees(-90);
        public static final double maximumAngle = Math.toDegrees(-110);

        public static final double gearRatio = 5 * 4 * 7; // 3 gear boxes

        public static final double ks = 0;
        public static final double kg = 2 * 0.19;
        public static final double kv = 1.22;
        public static final double ka = 0;

        public static final int FORWARD_CHANNEL = 5; // Tory made a mistake lol
        public static final int REVERSE_CHANNEL = 8; // 4;

        public static final int gripperMotor = 14;

        public static final double placementOffset = length / 2;
    }

    public static final class VisionConstants {
        public static final String photonCameraName = "Global_Shutter_Camera";

        // Includes 3d transform from camera(s) to robot origin
        public static final Transform3d photonCameraToRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(7.82), 0, Units.inchesToMeters(-31.2897)),
                new Rotation3d(0, 0, 0));

        public static final Transform3d photonRobotToCamera = photonCameraToRobot.inverse();

        public static final double tapeWidth = Units.inchesToMeters(2.0);

        public static final double upperRetroreflectiveHeight = Units.inchesToMeters(41.875 + 1);
        public static final double lowerRetroreflectiveHeight = Units.inchesToMeters(22.125 + 1);

        public static final double retroreflectiveAngleThreshold = 10;

        // Pretty sure 6 needs to negative
        public static final Transform3d limelightRobotToCamera = new Transform3d(
                new Translation3d(Units.inchesToMeters(6), Units.inchesToMeters(0), Units.inchesToMeters(34.25)),
                new Rotation3d(0, Math.toRadians(15), Math.PI));

        public static final Transform3d limelightCameraToRobot = limelightRobotToCamera.inverse();
    }

    public static final class SwerveConstants extends CompBotConstants {}

    public static class DevelopmentBotConstants {
        // See https://github.com/Team364/BaseFalconSwerve for getting these values.

        public static final boolean hasPigeon = false;
        public static final int PIGEON_PORT = 60;

        public static final double trackWidth = 0.5969;
        public static final double wheelBase = 0.5969;
        public static final double wheelDiameter = 0.10033;
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        // robot size
        public static final double widthWithBumpers = Units.inchesToMeters(30 + 3.25 * 2);
        public static final double lengthWithBumpers = Units.inchesToMeters(30 + 3.25 * 2);

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

        /* Motor Information */
        public static final double driveMotorFreeSpeed = 6380; // RPM of Falcon 500
        public static final double angleMotorFreeSpeed = 6380; // RPM of Falcon 500

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

    public static class CompBotConstants {
        // See https://github.com/Team364/BaseFalconSwerve for getting these values.

        public static final boolean hasPigeon = true;
        public static final int PIGEON_PORT = 29;

        public static final double lengthWithBumpers = Units.inchesToMeters(26 + 3.25 * 2);
        public static final double widthWithBumpers = Units.inchesToMeters(26 + 3.25 * 2);

        public static final double trackWidth = Units.inchesToMeters(19.5);
        public static final double wheelBase = Units.inchesToMeters(19.5);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double robotMass = Units.lbsToKilograms(115);

        public static final double openLoopRamp = 0.0; // 0.25
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (5.14 / 1.0); // 5.14:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        };

        public static final SecondOrderSwerveKinematics swerveKinematics =
                new SecondOrderSwerveKinematics(moduleTranslations);

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

        /* Motor Information */
        public static final double driveMotorFreeSpeed = 6380; // RPM of Falcon 500
        public static final double angleMotorFreeSpeed = 6380; // RPM of Falcon 500
        public static final double stallTorque = 4.69;

        /* Drive Motor Characterization Values */
        public static final double driveKS =
                (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Angle Motor Characterization Values */
        public static final double angleKS = 0;
        // (0.368 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double angleKV = (0.234 / 12);
        public static final double angleKA = (0.003 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 6.52; // meters per second
        public static final double maxAcceleration =
                (stallTorque * driveGearRatio * 4) / (wheelDiameter * robotMass); // 16.52; // meters per second^2
        public static final double maxAngularVelocity = maxSpeed // rad/s
                / Arrays.stream(moduleTranslations)
                        .map(translation -> translation.getNorm())
                        .max(Double::compare)
                        .get();

        /* Calculated Characterization Values */
        public static final double calculatedDriveKS = 0;
        public static final double calculatedDriveKV = (12 / maxSpeed) / GlobalConstants.targetVoltage;
        public static final double calculatedDriveKA = (12 / maxAcceleration) / GlobalConstants.targetVoltage;
        public static final double calculatedAngleKV =
                (12 * 60) / (angleMotorFreeSpeed * Math.toRadians(360 / angleGearRatio));

        /* Precise Driving Mode Values */
        public static final double preciseDrivingModeSpeedMultiplier = 0.2;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Drive Motor Inverts */
        public static final boolean driveMotorInvert = true;

        /* Drive Encoder Inverts */
        public static final boolean driveEncoderInvert = false;

        /* Angle Motor Inverts */
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        // Note, bevel gears should face left (relative to back-to-front)

        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 24;
            public static final double angleOffset = 256.553;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 26;
            public static final double angleOffset = 91.143;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 25;
            public static final double angleOffset = 38.760;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 27;
            public static final double angleOffset = 310.342;
            public static final String canivoreName = "CANivore";
            public static final SwerveModuleConstants constants =
                    new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, canivoreName);
        }
    }
}
