package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DoubleSubstationAssistCommand extends CommandBase {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private VisionSubsystem visionSubsystem;
    private double savedValue;

    private Constraints distanceConstraints = new Constraints(2, 3);
    private ProfiledPIDController distanceController = new ProfiledPIDController(1.5, 0, 0.01, distanceConstraints);
    private PIDController thetaController = new PIDController(2, 0, 0);

    private final Pose2d positionApriltag = FieldConstants.APRIL_TAG_FIELD_LAYOUT
            .getTagPose(DriverStation.getAlliance() == Alliance.Red ? 5 : 4)
            .get()
            .toPose2d();

    private final double FORK_SIZE = Units.feetToMeters(1.9);
    private final double ROBOT_DISTANCE_TO_APRILTAG =
            Units.inchesToMeters(10 + 1.5 + 13 + 8); // this and the above are polaceholders

    public DoubleSubstationAssistCommand(SwerveDriveSubsystem swerveDriveSubsystem, VisionSubsystem visionSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.visionSubsystem = visionSubsystem;

        distanceController.setTolerance(0.03);
        thetaController.setTolerance(Math.toRadians(1));

        thetaController.setSetpoint(Math.PI);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        distanceController.reset(0);
        savedValue = visionSubsystem.translationStdDevCoefficient;
        visionSubsystem.translationStdDevCoefficient = 0.1;
    }

    @Override
    public void execute() {
        var robotPosition = swerveDriveSubsystem.getPose();

        var goalPosition = positionApriltag
                .getTranslation()
                .plus(new Translation2d(
                        ROBOT_DISTANCE_TO_APRILTAG * (DriverStation.getAlliance() == Alliance.Red ? -1 : 1),
                        robotPosition.getY() > positionApriltag.getY() ? FORK_SIZE : -FORK_SIZE));

        var targetPositionRobotRelative = goalPosition.minus(robotPosition.getTranslation());

        double distance = targetPositionRobotRelative.getNorm();
        var direction = targetPositionRobotRelative.getAngle();

        double translationVelocity =
                distanceController.calculate(distance, 0) + distanceController.getSetpoint().velocity;
        double omega =
                thetaController.calculate(swerveDriveSubsystem.getRotation().getRadians());

        if (distanceController.atGoal()) translationVelocity = 0;
        if (thetaController.atSetpoint()) omega = 0;

        swerveDriveSubsystem.setVelocity(
                new ChassisSpeeds(
                        -direction.getCos() * translationVelocity, -direction.getSin() * translationVelocity, omega),
                true);
    }

    @Override
    public void end(boolean isInterrupted) {
        visionSubsystem.translationStdDevCoefficient = savedValue;
    }
}
