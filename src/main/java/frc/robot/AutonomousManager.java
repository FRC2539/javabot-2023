package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Stream;

public class AutonomousManager {
    private static final AutonomousOption defaultAuto = AutonomousOption.OPEN_PLACE1ANDCLIMB;

    // Add tunables for all autonomous configuration options
    LoggedReceiver waitDuration;
    LoggedReceiver startPosition;
    LoggedReceiver gamePieces;
    LoggedReceiver shouldClimb;

    private String previousStartPosition = defaultAuto.startPosition.name();
    private int previousGamePieces = defaultAuto.gamePieces;
    private boolean previousDoesClimb = defaultAuto.doesClimb;

    private SwerveAutoBuilder autoBuilder;

    private List<PathPlannerTrajectory> chosenAuto = defaultAuto.getPath();

    SwerveDriveSubsystem swerveDriveSubsystem;
    LightsSubsystem lightsSubsystem;

    public AutonomousManager(RobotContainer container) {
        swerveDriveSubsystem = container.getSwerveDriveSubsystem();
        ArmSubsystem armSubsystem = container.getArmSubsystem();
        GripperSubsystem gripperSubsystem = container.getGripperSubsystem();
        IntakeSubsystem intakeSubsystem = container.getIntakeSubsystem();
        lightsSubsystem = container.getLightsSubsystem();

        initializeNetworkTables();

        // Create an event map for use in all autos
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("stop", runOnce(swerveDriveSubsystem::stop, swerveDriveSubsystem));
        eventMap.put("levelChargeStation", swerveDriveSubsystem.levelChargeStationCommandDestiny());
        eventMap.put(
                "placeHigh",
                armSubsystem
                        .highManualConeCommand()
                        .andThen(waitSeconds(0.06))
                        .andThen(container
                                .getGripperSubsystem()
                                .ejectFromGripperCommand()
                                .withTimeout(0.3)
                                .asProxy())
                        .andThen(armSubsystem.awaitingDeploymentCommand().asProxy())
                        .asProxy());
        eventMap.put(
                "placeHighCube",
                waitUntil(() -> armSubsystem.isArmAtGoal() && armSubsystem.getState() == ArmState.AWAITING_DEPLOYMENT)
                        .andThen(armSubsystem
                                .highManualCubeCommand()
                                .andThen(waitSeconds(0.06))
                                .andThen(container
                                        .getGripperSubsystem()
                                        .ejectFromGripperCommand()
                                        .withTimeout(0.3)
                                        .asProxy())
                                .andThen(
                                        armSubsystem.awaitingDeploymentCommand().asProxy()))
                        .asProxy().unless(() -> !gripperSubsystem.hasGamePiece()).asProxy());
        eventMap.put(
                "intakePickup",
                container
                        .getIntakeSubsystem()
                        .intakeModeCommand()
                        .withTimeout(1.4)
                        .asProxy());
        eventMap.put(
                "reverseIntake",
                container
                        .getIntakeSubsystem()
                        .reverseIntakeModeCommand()
                        .withTimeout(0.75)
                        .asProxy());
        eventMap.put(
                "handoff",
                armSubsystem
                        .handoffCommand()
                        .deadlineWith(gripperSubsystem.dropFromGripperCommand())
                        .andThen(gripperSubsystem
                                .openGripperCommand()
                                .deadlineWith(waitSeconds(0.2).andThen(intakeSubsystem.handoffCommand()))
                                .withTimeout(1.5))
                        .andThen(armSubsystem.undoHandoffCommand())
                        .asProxy()
                        .unless(() -> !intakeSubsystem.isDeadOn()));

        autoBuilder = new SwerveAutoBuilder(
                swerveDriveSubsystem::getPose,
                swerveDriveSubsystem::setPose,
                new PIDConstants(3.5, 0.0, 0.0),
                new PIDConstants(1.3, 0.0, 0.001),
                (ChassisSpeeds velocity) -> swerveDriveSubsystem.setVelocity(velocity, false, false),
                eventMap,
                true,
                swerveDriveSubsystem);
    }

    public void update() {
        var newStartPosition = startPosition.getString();
        var newGamePieces = gamePieces.getInteger();
        var newDoesClimb = shouldClimb.getBoolean();

        // Only update the chosen auto if a different option has been chosen
        if (previousStartPosition != newStartPosition
                || previousGamePieces != newGamePieces
                || previousDoesClimb != newDoesClimb) {
            // Match the auto based on the dashboard configuration
            List<AutonomousOption> options = Stream.of(AutonomousOption.values())
                    .filter(option -> option.startPosition.name().equals(newStartPosition)
                            && option.gamePieces == newGamePieces
                            && option.doesClimb == newDoesClimb)
                    .toList();

            if (options.size() == 1) chosenAuto = options.get(0).getPath();
            else chosenAuto = defaultAuto.getPath();

            // Determine all of the game piece options for this starting position
            long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                    .filter(option ->
                            option.startPosition.name().equals(newStartPosition) && option.doesClimb == newDoesClimb)
                    .mapToLong(option -> option.gamePieces)
                    .toArray();

            Logger.log("/Autonomous/Game Piece Options", gamePieceOptions).alwaysNT();

            previousStartPosition = newStartPosition;
            previousGamePieces = (int) newGamePieces;
            previousDoesClimb = newDoesClimb;
        }
    }

    public Command getAutonomousCommand() {
        Command chosenPathCommand = autoBuilder.fullAuto(chosenAuto);

        var chosenWaitDuration = waitDuration.getInteger();

        if (chosenWaitDuration > 0) chosenPathCommand.beforeStarting(waitSeconds(chosenWaitDuration));

        return chosenPathCommand;
    }

    private void initializeNetworkTables() {
        waitDuration = Logger.tunable("/Autonomous/Wait Duration", 0.0);
        startPosition = Logger.tunable(
                "/Autonomous/Start Position", defaultAuto.startPosition.name()); // 0 = Left, 1 = Center, 2 = Right
        gamePieces = Logger.tunable("/Autonomous/Game Pieces", defaultAuto.gamePieces);
        shouldClimb = Logger.tunable("/Autonomous/Should Climb", defaultAuto.doesClimb);

        Logger.log("/Autonomous/Start Position Options", getStartingLocations()).alwaysNT();

        // Determine all of the game piece options for this starting position
        long[] gamePieceOptions = Stream.of(AutonomousOption.values())
                .filter(option -> option.startPosition.equals(defaultAuto.startPosition)
                        && option.doesClimb == defaultAuto.doesClimb)
                .mapToLong(option -> option.gamePieces)
                .toArray();

        Logger.log("/Autonomous/Game Piece Options", gamePieceOptions).alwaysNT();
    }

    private enum AutonomousOption {
        OPEN_PLACE1ANDCLIMB(StartingLocation.OPEN, 1, true, "open_place1andclimb", new PathConstraints(5, 5)),
        // OPEN_PLACE2(StartingLocation.OPEN, 2, "open_place2", new PathConstraints(3.5, 3)),
        OPEN_PLACE2HANDOFF(StartingLocation.OPEN, 2, true, "open_place2handoff", new PathConstraints(4, 3)),
        // OPEN_PLACE2ANDCLIMB(StartingLocation.OPEN, 2, true, "open_place2andclimb", new PathConstraints(4, 3)),
        OPEN_PLACE3(StartingLocation.OPEN, 3, false, "open_place3", new PathConstraints(4, 3)),
        // OPEN_PLACE3ANDCLIMB(StartingLocation.OPEN, 3, "open_place3andclimb", new PathConstraints(6, 5)),
        // OPEN_FIVEPIECE(StartingLocation.OPEN, 5, "open_fivepiece", new PathConstraints(5, 6)),
        STATION_PLACE1ANDCLIMB(
                StartingLocation.STATION, 1, true, "station_place1andclimb_fancy", new PathConstraints(3, 2.25)),
        CABLE_PLACE1ANDCLIMB(StartingLocation.CABLE, 1, true, "cable_place1andclimb", new PathConstraints(5, 5)),
        CABLE_PLACE2(StartingLocation.CABLE, 2, false, "cable_place2", new PathConstraints(4, 3)),
        CABLE_PLACE3(StartingLocation.CABLE, 3, false, "cable_place3", new PathConstraints(3.5, 3));

        private List<PathPlannerTrajectory> path;
        private String pathName;
        private PathConstraints constraint;
        private PathConstraints[] constraints;
        public StartingLocation startPosition;
        public int gamePieces;
        public boolean doesClimb;

        private AutonomousOption(
                StartingLocation startPosition,
                int gamePieces,
                boolean doesClimb,
                String pathName,
                PathConstraints constraint,
                PathConstraints... constraints) {
            this.startPosition = startPosition;
            this.gamePieces = gamePieces;
            this.doesClimb = doesClimb;
            this.pathName = pathName;
            this.constraint = constraint;
            this.constraints = constraints;
        }

        public List<PathPlannerTrajectory> getPath() {
            // Lazy load the path
            if (path == null) path = PathPlanner.loadPathGroup(pathName, constraint, constraints);

            return path;
        }
    }

    private enum StartingLocation {
        OPEN,
        STATION,
        CABLE
    }

    public static String[] getStartingLocations() {
        return Stream.of(StartingLocation.values()).map(StartingLocation::name).toArray(String[]::new);
    }

    public static String[] getAutonomousOptionNames() {
        return Stream.of(AutonomousOption.values()).map(AutonomousOption::name).toArray(String[]::new);
    }
}
