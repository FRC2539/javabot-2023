package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

public class AutonomousManager {
    private LoggedReceiver selectedAuto;

    private SwerveAutoBuilder autoBuilder;

    private final AutonomousOption defaultAuto = AutonomousOption.DEMO;

    SwerveDriveSubsystem swerveDriveSubsystem;

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        ArmSubsystem armSubsystem = container.getArmSubsystem();

        // Allow the custom driver station to select an auto
        initializeNetworkTablesValues();

        // Create an event map for use in all autos
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("stop", runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem));
        eventMap.put("levelChargeStation", swerveDriveSubsystem.levelChargeStationCommand());
        eventMap.put(
                "placeHigh",
                sequence(
                        runOnce(armSubsystem::setHigh, armSubsystem),
                        waitSeconds(1),
                        runOnce(armSubsystem::setAwaitingPiece, armSubsystem)));

        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(1.0, 0.0, 0.001),
                (ChassisSpeeds velocity) -> swerveDriveSubsystem.setVelocity(velocity, false, false),
                eventMap,
                true,
                swerveDriveSubsystem);

        // Prevent the server from running at competitions
        if (!Constants.competitionMode) {
            PathPlannerServer.startServer(5811);
        }
    }

    private enum AutonomousOption {
        DEMO("demo2", new PathConstraints(2, 3)),
        AUTOCLIMB("autoclimb", new PathConstraints(3, 2)),
        PLACE1ANDCLIMB("place1andclimb", new PathConstraints(5, 4)),
        PLACE2ANDCLIMB("place2andclimb", new PathConstraints(5, 4)),
        COLLISIONTESTING("collisiontesting", new PathConstraints(4, 3));

        private List<PathPlannerTrajectory> path;

        private AutonomousOption(String pathName, PathConstraints constraints) {
            this.path = PathPlanner.loadPathGroup(pathName, constraints);
        }

        public Command getCommand(SwerveAutoBuilder autoBuilder) {
            return autoBuilder.fullAuto(path);
        }
    }

    public Command getAutonomousCommand() {
        String nameOfSelectedAuto = selectedAuto.getString();

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

    private void initializeNetworkTablesValues() {
        // Insert all of the auto options into the network tables
        Logger.log("/Autonomous/autos", getAutonomousOptionNames());

        selectedAuto = Logger.tunable("/Autonomous/selectedAuto", defaultAuto.name());
    }

    public static String[] getAutonomousOptionNames() {
        return Stream.of(AutonomousOption.values()).map(AutonomousOption::name).toArray(String[]::new);
    }
}
