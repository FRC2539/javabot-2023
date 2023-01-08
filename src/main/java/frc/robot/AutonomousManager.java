package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.List;

public class AutonomousManager {
    private NetworkTable autonomousTable;
    private NetworkTableEntry selectedAuto;
    private final String[] autoStrings = {"demo"};

    SwerveDriveSubsystem swerveDriveSubsystem;

    private HashMap<String, Command> eventMap = new HashMap<>();
    private SwerveAutoBuilder autoBuilder;

    // Load all autonomous paths
    List<PathPlannerTrajectory> demoPath = PathPlanner.loadPathGroup("demo2", new PathConstraints(2, 3));

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();

        // Allow thd custom driver station to select an auto
        initializeNetworkTablesValues();

        // Create an event map for use in all autos
        eventMap.put("stop", runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem));

        SwerveDriveSubsystem swerveDriveSubsystem = container.getSwerveDriveSubsystem();

        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                new PIDConstants(3.0, 0.0, 0.0), // try decreasing P here
                new PIDConstants(0.02, 0.0, 0.0),
                (ChassisSpeeds velocity) -> swerveDriveSubsystem.setVelocity(velocity, false, false),
                eventMap,
                swerveDriveSubsystem);

        // Prevent the server from running at competitions
        if (!Constants.competitionMode) {
            PathPlannerServer.startServer(5811);
        }

        // Temporary Robot Auto Guide

        // Forward is 180 degrees always.
        // To make an appropriate angle, make the back of the robot face the angle you want in pathplanner

        // To make a stop point, add a time duration in the marker page.
        // Then, add the "stop" event.
    }

    public Command getAutonomousCommand() {
        switch (selectedAuto.getString(autoStrings[0])) {
            case "demo":
                return pathGroupCommand(demoPath);
        }

        // Return an empty command group if no auto is specified
        return new SequentialCommandGroup();
    }

    private Command pathGroupCommand(List<PathPlannerTrajectory> pathGroup) {
        return autoBuilder.fullAuto(pathGroup).andThen(reversePoseCommand());
    }

    private Command pathFollowCommand(PathPlannerTrajectory path) {
        return autoBuilder.followPath(path).andThen(reversePoseCommand());
    }

    private Command reversePoseCommand() {
        return runOnce(() -> swerveDriveSubsystem.setRotation(
                swerveDriveSubsystem.getRotation().rotateBy(Rotation2d.fromDegrees(180))));
    }

    /**
     * Currently only drives to the correct coordinate, angle is not guaranteed.
     * @param targetPose
     * @return A command that will drive to the specified pose
     */
    public Command driveToPoseCommand(Pose2d targetPose) {
        return new ProxyCommand(() -> generateDriveToPoseCommand(targetPose));
    }

    private Command generateDriveToPoseCommand(Pose2d targetPose) {
        var initialPose = swerveDriveSubsystem.getPose();
        var initialRotation = initialPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        var targetRotation = targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));
        var currentVelocityDirection = swerveDriveSubsystem.getVelocityRotation();

        var path = PathPlanner.generatePath(
                new PathConstraints(2.5, 5),
                new PathPoint(
                        initialPose.getTranslation(),
                        currentVelocityDirection,
                        initialRotation,
                        swerveDriveSubsystem.getVelocityMagnitude()),
                new PathPoint(
                        targetPose.getTranslation(),
                        targetRotation,
                        targetPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))));

        return pathFollowCommand(path).andThen(reversePoseCommand());
    }

    private void initializeNetworkTablesValues() {
        autonomousTable = NetworkTableInstance.getDefault().getTable("Autonomous");

        // Insert all of the auto options into the network tables
        autonomousTable.getEntry("autos").setStringArray(autoStrings);

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Choose the first auto as the default
        selectedAuto.setString(autoStrings[0]);
    }
}
