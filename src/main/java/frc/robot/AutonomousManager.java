package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

public class AutonomousManager {
    private NetworkTable autonomousTable;
    private NetworkTableEntry selectedAuto;

    private SwerveAutoBuilder autoBuilder;

    private final AutonomousOption defaultAuto = AutonomousOption.DEMO;

    public AutonomousManager(RobotContainer container) {
        SwerveDriveSubsystem swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        ArmSubsystem armSubsystem = container.getArmSubsystem();

        // Allow thd custom driver station to select an auto
        initializeNetworkTablesValues();

        // Create an event map for use in all autos
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("stop", runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem));
        eventMap.put(
                "placeHigh",
                sequence(
                        runOnce(armSubsystem::setHigh, armSubsystem),
                        waitSeconds(1),
                        runOnce(armSubsystem::setAwaitingPiece, armSubsystem)));

        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                new PIDConstants(3.0, 0.0, 0.0), // try decreasing P here
                new PIDConstants(0.03, 0.0, 0.001),
                (ChassisSpeeds velocity) -> swerveDriveSubsystem.setVelocity(velocity, false, false),
                eventMap,
                swerveDriveSubsystem);

        // Prevent the server from running at competitions
        if (!Constants.competitionMode) {
            PathPlannerServer.startServer(5811);
        }
    }

    public Command getAutonomousCommand() {
        String nameOfSelectedAuto = selectedAuto.getString(defaultAuto.name());

        Command autonomousCommand;

        // Run the default auto if an invalid auto has been chosen
        try {
            autonomousCommand = AutonomousOption.valueOf(nameOfSelectedAuto).getCommand(autoBuilder);
        } catch (Exception e) {
            autonomousCommand = AutonomousOption.valueOf(defaultAuto.name()).getCommand(autoBuilder);
        }

        // Return an empty command group if no auto is specified
        return autonomousCommand;
    }

    private enum AutonomousOption {
        DEMO("demo2", new PathConstraints(2, 3)),
        PLACE1ANDCLIMB("place1andclimb", new PathConstraints(5, 4));

        private List<PathPlannerTrajectory> path;

        private AutonomousOption(String pathName, PathConstraints constraints) {
            this.path = PathPlanner.loadPathGroup(pathName, constraints);
        }

        public Command getCommand(SwerveAutoBuilder autoBuilder) {
            return autoBuilder.fullAuto(path);
        }
    }

    private void initializeNetworkTablesValues() {
        autonomousTable = NetworkTableInstance.getDefault().getTable("Autonomous");

        // Insert all of the auto options into the network tables
        autonomousTable.getEntry("autos").setStringArray(getAutonomousOptionNames());

        selectedAuto = autonomousTable.getEntry("selectedAuto");

        // Choose the first auto as the default
        selectedAuto.setString(defaultAuto.name());
    }

    public static String[] getAutonomousOptionNames() {
        return Stream.of(AutonomousOption.values()).map(AutonomousOption::name).toArray(String[]::new);
    }
}
