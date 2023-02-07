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
    private static final AutonomousOption defaultAuto = AutonomousOption.PLACE1ANDCLIMB;

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration = Logger.tunable("/Autonomous/Wait Duration", 0.0);
    LoggedReceiver startPosition =
            Logger.tunable("/Autonomous/Start Position", defaultAuto.startPosition); // 0 = Left, 1 = Center, 2 = Right
    LoggedReceiver gamePieces = Logger.tunable("/Autonomous/Game Pieces", defaultAuto.gamePieces);
    LoggedReceiver shouldClimb = Logger.tunable("/Autonomous/Should Climb", true);

    static {
        Logger.log("Autonomous/Start Position Options", new long[] {0, 1, 2});

        // Determine all of the game piece options for this starting position
        long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                .filter(option -> option.startPosition == defaultAuto.startPosition)
                .map(option -> option.gamePieces)
                .mapToLong(i -> i)
                .toArray();

        Logger.log("/Autonomous/Game Piece Options", gamePieceOptions);
    }

    private int previousStartPosition = defaultAuto.startPosition;
    private int previousGamePieces = defaultAuto.gamePieces;

    private SwerveAutoBuilder autoBuilder;

    private List<PathPlannerTrajectory> chosenAuto;

    SwerveDriveSubsystem swerveDriveSubsystem;

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        ArmSubsystem armSubsystem = container.getArmSubsystem();

        // Create an event map for use in all autos
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("stop", runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem));
        eventMap.put(
                "shouldClimb",
                either(none(), run(swerveDriveSubsystem::stop, swerveDriveSubsystem), () -> shouldClimb.getBoolean()));
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

    public void update() {
        var newStartPosition = gamePieces.getInteger(); 
        var newGamePieces = gamePieces.getInteger();

        // TODO Flip auto side depending on alliance color
        // Thinking of using an enum based on bump, charge station, and carpet

        // Only update the chosen auto if a different option has been chosen
        if (previousStartPosition != newStartPosition || previousGamePieces != newGamePieces) {
            // Match the auto based on the dashboard configuration
            List<AutonomousOption> options = Stream.of(AutonomousOption.values())
                    .filter(option -> option.startPosition == newStartPosition && option.gamePieces == newGamePieces)
                    .toList();

            if (options.size() == 1) chosenAuto = options.get(0).getPath();
            else chosenAuto = defaultAuto.getPath();

            // Determine all of the game piece options for this starting position
            long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                    .filter(option -> option.startPosition == newStartPosition)
                    .map(option -> option.gamePieces)
                    .mapToLong(i -> i)
                    .toArray();

            Logger.log("/Autonomous/Game Piece Options", gamePieceOptions);

            previousStartPosition = (int) newStartPosition;
            previousGamePieces = (int) newGamePieces;
        }
    }

    public Command getAutonomousCommand() {
        Command chosenPathCommand = autoBuilder.fullAuto(chosenAuto);

        var chosenWaitDuration = waitDuration.getInteger();

        if (chosenWaitDuration > 0) chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));

        return chosenPathCommand;
    }

    private enum AutonomousOption {
        PLACE1ANDCLIMB(0, 1, "place1andclimb", new PathConstraints(5, 4)),
        PLACE2ANDCLIMB(0, 2, "place2andclimb", new PathConstraints(5, 4)),
        PLACE3ANDCLIMB(0, 3, "place3andclimb", new PathConstraints(6, 5)),
        FIVEPIECE(0, 5, "fivepiece", new PathConstraints(5, 6));

        private List<PathPlannerTrajectory> path;
        private String pathName;
        private PathConstraints constraints;
        public int startPosition;
        public int gamePieces;

        private AutonomousOption(int startPosition, int gamePieces, String pathName, PathConstraints constraints) {
            this.startPosition = startPosition;
            this.gamePieces = gamePieces;
            this.pathName = pathName;
            this.constraints = constraints;
        }

        public List<PathPlannerTrajectory> getPath() {
            // Lazy load the path
            if (path == null) path = PathPlanner.loadPathGroup(pathName, constraints);

            return path;
        }
    }

    public static String[] getAutonomousOptionNames() {
        return Stream.of(AutonomousOption.values()).map(AutonomousOption::name).toArray(String[]::new);
    }
}
