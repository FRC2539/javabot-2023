package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.Conversions;
import frc.lib.math.MathUtils;
import frc.lib.math.TwoJointedFourBarArmFeedforward;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.PlacementLocation;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Robot;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.stream.Stream;

public class ArmSubsystem extends SubsystemBase {
    private Mechanism2d mechanism = new Mechanism2d(3, 2);
    private MechanismRoot2d root = mechanism.getRoot("Arm", 1, 0.5);
    private MechanismLigament2d arm1;
    private MechanismLigament2d arm2;
    private MechanismLigament2d gripper;

    private MechanismLigament2d ghostArm1;
    private MechanismLigament2d ghostArm2;
    private MechanismLigament2d ghostGripper;

    private double joint1DesiredMotorPosition = 0;
    private double joint2DesiredMotorPosition = 0;
    private double gripperDesiredMotorPosition = 0;

    private Rotation2d arm1Angle = new Rotation2d();
    private Rotation2d arm2Angle = new Rotation2d();
    private Rotation2d gripperAngle = new Rotation2d();

    private double arm1Speed = 0; // radians / second
    private double arm2Speed = 0;

    @SuppressWarnings("unused")
    private double gripperSpeed = 0;

    private LinearFilter arm1SpeedFilter = LinearFilter.movingAverage(10);
    private LinearFilter arm2SpeedFilter = LinearFilter.movingAverage(10);
    private LinearFilter gripperSpeedFilter = LinearFilter.movingAverage(10);

    private Rotation2d lastGripperPosition;
    private Rotation2d lastArm1Position;
    private Rotation2d lastArm2Position;

    LoggedReceiver desiredNetworkTablesArmPosition;

    private WPI_TalonFX joint1Motor;
    private WPI_TalonFX joint2Motor;
    private WPI_TalonSRX gripperMotor;

    // Used for correctly offsetting integrated encoders
    private DutyCycleEncoder joint1AbsoluteEncoder;
    private DutyCycleEncoder joint2AbsoluteEncoder;

    // Wrist isn't run on a falcon, so this is used as the main encoder
    private DutyCycleEncoder gripperAbsoluteEncoder;

    private boolean brakingActivated;

    private ProfiledPIDController motor1Controller;
    private ProfiledPIDController motor2Controller;
    private ProfiledPIDController gripperMotorController;

    // private static final TrapezoidProfile.Constraints motor1Constraints = new TrapezoidProfile.Constraints(3, 8);
    private static final TrapezoidProfile.Constraints motor1Constraints = new TrapezoidProfile.Constraints(5, 20);
    private static final TrapezoidProfile.Constraints motor2Constraints = new TrapezoidProfile.Constraints(9, 20);
    private static final TrapezoidProfile.Constraints gripperProfileConstraints =
            new TrapezoidProfile.Constraints(6, 15);

    private Translation2d endEffector = new Translation2d();
    private Rotation2d gripperEndAngle = new Rotation2d();

    private ArmState armState = ArmState.AWAITING_DEPLOYMENT;

    private TwoJointedFourBarArmFeedforward simFeedforward;
    private TwoJointedFourBarArmFeedforward feedforward;
    private ArmFeedforward gripperJointFeedforward;

    private SwerveDriveSubsystem swerveDriveSubsystem;

    private double springConstant = 150; // 175;
    private double arm1PValue;

    private double lastAbsoluteGripper = Math.PI / 2;

    private Optional<Double> desiredGripperSpeedOverride = Optional.empty();

    public ArmSubsystem(SwerveDriveSubsystem swerveDriveSubsystem) {
        // springConstant = Logger.tunable("/ArmSubsystem/springConstant", 160);
        // voltageOffset = Logger.tunable("/ArmSubsystem/voltageOffset", -1.0);
        // Arm P value tuning
        arm1PValue = 7;
        // arm1PValueReceiver = Logger.tunable("/ArmSubsystem/arm1PValue", arm1PValue);

        this.swerveDriveSubsystem = swerveDriveSubsystem;

        /* Create arm and ghost arms in mechanism 2d */
        arm1 = root.append(
                new MechanismLigament2d("Arm 1", ArmConstants.arm1Length, ArmConstants.arm1StartingAngle.getDegrees()));
        arm2 = arm1.append(
                new MechanismLigament2d("Arm 2", ArmConstants.arm2Length, ArmConstants.arm2StartingAngle.getDegrees()));
        gripper = arm2.append(new MechanismLigament2d(
                "Gripper", GripperConstants.length, GripperConstants.startingAngle.getDegrees()));

        ghostArm1 = root.append(new MechanismLigament2d(
                "Ghost Arm 1", ArmConstants.arm1Length, ArmConstants.arm1StartingAngle.getDegrees()));
        ghostArm2 = ghostArm1.append(new MechanismLigament2d(
                "Ghost Arm 2", ArmConstants.arm2Length, ArmConstants.arm2StartingAngle.getDegrees()));
        ghostGripper = ghostArm2.append(new MechanismLigament2d(
                "Ghost Gripper", GripperConstants.length, GripperConstants.startingAngle.getDegrees()));

        ghostArm1.setLineWeight(5);
        ghostArm1.setColor(new Color8Bit(Color.kGray));
        ghostArm2.setLineWeight(5);
        ghostArm2.setColor(new Color8Bit(Color.kGray));
        ghostGripper.setLineWeight(5);
        ghostGripper.setColor(new Color8Bit(Color.kGray));

        arm1.setLineWeight(5);
        arm2.setLineWeight(5);
        gripper.setLineWeight(5);

        /* Create the grid */
        var robotBase = root.append(new MechanismLigament2d(
                "RobotBase", ArmConstants.robotToArm.getZ(), -90, 3, new Color8Bit(Color.kBlue)));
        var robotRight = robotBase.append(new MechanismLigament2d(
                "RobotRight", SwerveConstants.lengthWithBumpers / 2, 90, 3, new Color8Bit(Color.kBlue)));
        robotBase.append(new MechanismLigament2d(
                "RobotLeft", SwerveConstants.lengthWithBumpers / 2, -90, 3, new Color8Bit(Color.kBlue)));

        var gridBottom = robotRight.append(
                new MechanismLigament2d("GridBottom", FieldConstants.outerX, 0, 2, new Color8Bit(Color.kGray)));
        var highBase = gridBottom.append(
                new MechanismLigament2d("HighBase", FieldConstants.highX, 180, 2, new Color8Bit(Color.kGray)));
        highBase.append(
                new MechanismLigament2d("HighCone", FieldConstants.highConeZ, -90, 2, new Color8Bit(Color.kGray)));
        var midBase = gridBottom.append(
                new MechanismLigament2d("MidBase", FieldConstants.midX, 180, 2, new Color8Bit(Color.kGray)));
        midBase.append(new MechanismLigament2d("MidCone", FieldConstants.midConeZ, -90, 2, new Color8Bit(Color.kGray)));

        arm1Angle = ArmConstants.arm1StartingAngle;
        arm2Angle = ArmConstants.arm2StartingAngle;
        gripperAngle = GripperConstants.startingAngle;

        joint1Motor = new WPI_TalonFX(ArmConstants.mastMotorPort, GlobalConstants.CANIVORE_NAME);
        joint2Motor = new WPI_TalonFX(ArmConstants.boomMotorPort, GlobalConstants.CANIVORE_NAME);
        gripperMotor = new WPI_TalonSRX(ArmConstants.wristMotorPort);

        joint1Motor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        joint2Motor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        gripperMotor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        joint1Motor.enableVoltageCompensation(true);
        joint2Motor.enableVoltageCompensation(true);
        gripperMotor.enableVoltageCompensation(true);

        joint1AbsoluteEncoder = new DutyCycleEncoder(ArmConstants.mastEncoderChannel);
        joint2AbsoluteEncoder = new DutyCycleEncoder(ArmConstants.boomEncoderChannel);
        gripperAbsoluteEncoder = new DutyCycleEncoder(ArmConstants.gripperEncoderChannel);

        joint1Motor.setInverted(ArmConstants.invertMastMotor);
        joint2Motor.setInverted(ArmConstants.invertBoomMotor);
        gripperMotor.setInverted(ArmConstants.invertWristMotor);

        // Calibrate the joint motors
        if (Robot.isReal()) calibrateIntegratedEncoders();

        gripperEndAngle = GripperConstants.startingAngle;

        endEffector = forwardKinematics(
                ArmConstants.arm1Length,
                ArmConstants.arm1StartingAngle,
                ArmConstants.arm2Length,
                ArmConstants.arm2StartingAngle,
                GripperConstants.length,
                GripperConstants.startingAngle);

        desiredNetworkTablesArmPosition = Logger.tunable("/ArmSubsystem/Arm Pose", new double[] {1, 0.1, 0});

        SmartDashboard.putData("Arm Mechanism", mechanism);

        simFeedforward = new TwoJointedFourBarArmFeedforward(
                ArmConstants.arm1Length,
                ArmConstants.arm2Length,
                ArmConstants.arm1CenterOfMass,
                ArmConstants.arm2CenterOfMass,
                ArmConstants.arm1Mass,
                ArmConstants.arm2Mass,
                ArmConstants.arm1MomentOfInertia,
                ArmConstants.arm2MomentOfInertia,
                ArmConstants.arm1GearRatio,
                ArmConstants.arm2GearRatio,
                1,
                1,
                ArmConstants.stallTorque,
                ArmConstants.stallCurrent,
                ArmConstants.freeSpeed,
                9.81,
                12);

        // for testing
        feedforward = new TwoJointedFourBarArmFeedforward(
                ArmConstants.arm1Length,
                ArmConstants.arm2Length,
                ArmConstants.arm1CenterOfMass,
                ArmConstants.arm2CenterOfMass,
                ArmConstants.arm1Mass,
                ArmConstants.arm2Mass,
                ArmConstants.arm1MomentOfInertia,
                ArmConstants.arm2MomentOfInertia,
                ArmConstants.arm1GearRatio,
                ArmConstants.arm2GearRatio,
                1,
                1,
                ArmConstants.stallTorque,
                ArmConstants.stallCurrent,
                ArmConstants.freeSpeed,
                9.81,
                12);

        gripperJointFeedforward =
                new ArmFeedforward(GripperConstants.ks, GripperConstants.kg, GripperConstants.kv, GripperConstants.ka);

        // motor1Controller = new ProfiledPIDController(12.8, 0, 0.12, motor1Constraints);
        motor1Controller = new ProfiledPIDController(arm1PValue, 0, 0.15, motor1Constraints);
        motor2Controller = new ProfiledPIDController(5, 0, 0.15, motor2Constraints);
        gripperMotorController = new ProfiledPIDController(4, 0, 0.1, gripperProfileConstraints);

        resetPIDControllers();

        // Don't die when arm is backwards slightly
        motor2Controller.enableContinuousInput(-Math.PI, Math.PI);

        motor1Controller.setTolerance(ArmConstants.angularTolerance);
        motor2Controller.setTolerance(ArmConstants.angularTolerance);
        gripperMotorController.setTolerance(ArmConstants.angularTolerance);

        setState(armState);
    }

    public Command highManualConeCommand() {
        return Commands.sequence(
                armStateApproximateCommand(ArmState.HIGH_MANUAL_1), armStateCommand(ArmState.HIGH_MANUAL_CONE));
    }

    public Command hoverCommand() {
        return Commands.sequence(
                armStateApproximateCommand(ArmState.HIGH_MANUAL_1), armStateCommand(ArmState.HOVER_MANUAL));
    }

    public Command highManualCubeCommand() {
        return Commands.sequence(
                armStateApproximateCommand(ArmState.HIGH_MANUAL_1), armStateCommand(ArmState.HIGH_MANUAL_CUBE));
    }

    public Command handoffCommand() {
        return armHandoffStateCommand(ArmState.COOL_HANDOFF);
    }

    public Command undoHandoffCommand() {
        return Commands.sequence(
                armStateApproximateCommand(ArmState.COOL_HANDOFF_REVERSE),
                armStateCommand(ArmState.AWAITING_DEPLOYMENT));
    }

    public Command partialUndoHandoffCommand() {
        return armStateApproximateCommand(ArmState.COOL_HANDOFF_REVERSE);
    }

    public Command substationPickupCommand() {
        return armStateCommand(ArmState.SUBSTATION_PICKUP);
    }

    public Command pickupCommand() {
        return Commands.sequence(
                armStateApproximateCommand(ArmState.AWAITING_DEPLOYMENT_1), armStateCommand(ArmState.HYBRID_MANUAL));
    }

    public Command slidePickupCommand() {
        return armStateCommand(ArmState.SLIDE_PICKUP);
    }

    public Command tippedPickupCommand() {
        return Commands.sequence(
                armStateApproximateCommand(ArmState.AWAITING_DEPLOYMENT_1),
                armStateCommand(ArmState.TIPPED_CONE_MANUAL));
    }

    public Command awaitingDeploymentCommand() {
        return Commands.either(
                Commands.sequence(
                        armStateApproximateCommand(ArmState.HIGH_MANUAL_1),
                        armStateApproximateCommand(ArmState.MID_MANUAL_CONE),
                        armStateApproximateCommand(ArmState.AWAITING_DEPLOYMENT_1),
                        armStateCommand(ArmState.AWAITING_DEPLOYMENT)),
                Commands.sequence(
                        armStateApproximateCommand(ArmState.AWAITING_DEPLOYMENT_1),
                        armStateCommand(ArmState.AWAITING_DEPLOYMENT)),
                () -> armState == ArmState.HIGH_MANUAL_CONE
                        || armState == ArmState.HIGH_MANUAL_CUBE
                        || armState == ArmState.HOVER_MANUAL);
    }

    // public Command awaitingDeploymentCommand() {
    //     return Commands.sequence(
    //             armStateApproximateCommand(ArmState.AWAITING_DEPLOYMENT_1),
    //             armStateCommand(ArmState.AWAITING_DEPLOYMENT));
    // }

    public Command midManualConeCommand() {
        return armStateCommand(ArmState.MID_MANUAL_CONE);
    }

    public Command midManualCubeCommand() {
        // return Commands.sequence(
        //         armStateApproximateCommand(ArmState.MID_MANUAL_CUBE_1), armStateCommand(ArmState.MID_MANUAL_CUBE));
        return armStateCommand(ArmState.MID_MANUAL_CUBE_NEW);
    }

    public boolean isCurrentLocationCone() {
        return FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose()).isCone;
    }

    public void resetPIDControllers() {
        motor1Controller.reset(arm1Angle.getRadians());
        motor2Controller.reset(arm2Angle.getRadians());
        gripperMotorController.reset(gripperAngle.getRadians());
    }

    private Matrix<N3, N1> inverseKinematics(Translation2d gripperEndEffector, Rotation2d gripperAngle) {
        Translation2d endEffector = gripperEndEffector.minus(new Translation2d(gripper.getLength(), gripperAngle));
        try {
            if (endEffector.getNorm() >= arm1.getLength() + arm2.getLength())
                throw new Exception("Arm Not Long Enough");
            double arm2InverseAngle = -Math.acos((Math.pow(endEffector.getX(), 2)
                            + Math.pow(endEffector.getY(), 2)
                            - Math.pow(arm1.getLength(), 2)
                            - Math.pow(arm2.getLength(), 2))
                    / (2 * arm1.getLength() * arm2.getLength()));
            double arm1InverseAngle = Math.atan2(endEffector.getY(), endEffector.getX())
                    - Math.atan2(
                            arm2.getLength() * Math.sin(arm2InverseAngle),
                            arm1.getLength() + arm2.getLength() * Math.cos(arm2InverseAngle));
            return VecBuilder.fill(
                    arm1InverseAngle,
                    arm2InverseAngle,
                    gripperAngle.getRadians() - arm1InverseAngle - arm2InverseAngle);
        } catch (Exception e) {
            double arm1InverseAngle = endEffector.getAngle().getRadians();
            return VecBuilder.fill(arm1InverseAngle, 0, gripperAngle.getRadians() - arm1InverseAngle);
        }
    }

    private Translation2d forwardKinematics(
            double arm1Length,
            Rotation2d arm1Angle,
            double arm2Length,
            Rotation2d arm2Angle,
            double gripperLength,
            Rotation2d gripperAngle) {
        return new Translation2d(
                arm1Length * arm1Angle.getCos()
                        + arm2Length * arm2Angle.plus(arm1Angle).getCos()
                        + gripperLength
                                * gripperAngle.plus(arm1Angle).plus(arm2Angle).getCos(),
                arm1Length * arm1Angle.getSin()
                        + arm2Length * arm2Angle.plus(arm1Angle).getSin()
                        + gripperLength
                                * gripperAngle.plus(arm1Angle).plus(arm2Angle).getSin());
    }

    public void setState(ArmState state) {
        armState = state;
        Logger.log("/ArmSubsystem/armState", armState.toString());
        if (armState.getType() instanceof Brake) {
            stopMotors();
        } else if (armState.getType() instanceof Coast) {
            coastMotors();
        } else {
            startMotors();
        }
        if (state == ArmState.SUBSTATION_PICKUP) {
            motor2Controller.setPID(5, 0, 0.3);
        } else {
            motor2Controller.setPID(6, 0, 0.15);
        }
        updateArmDesiredPosition();
    }

    private void setMotorsNeutralMode(NeutralMode mode) {
        joint1Motor.setNeutralMode(mode);
        joint2Motor.setNeutralMode(mode);
        gripperMotor.setNeutralMode(mode);
    }

    private void stopMotors() {
        brakingActivated = true;
        setMotorsNeutralMode(NeutralMode.Brake);
        joint1Motor.stopMotor();
        joint2Motor.stopMotor();
        gripperMotor.stopMotor();
    }

    private void coastMotors() {
        brakingActivated = false;
        setMotorsNeutralMode(NeutralMode.Coast);
        joint1Motor.stopMotor();
        joint2Motor.stopMotor();
        gripperMotor.stopMotor();
    }

    private void startMotors() {
        brakingActivated = false;
        setMotorsNeutralMode(NeutralMode.Brake);
    }

    public void updateArmDesiredPosition() {
        // Store the current desired end effector position (where the end of the arm should be)
        if (armState.getType() instanceof Static) {
            Static armType = (Static) armState.getType();
            endEffector = armType.getEndEffector();
            gripperEndAngle = armType.getGripperAngle();
        } else if (armState.getType() instanceof Dynamic) {
            Dynamic armType = (Dynamic) armState.getType();
            endEffector = armType.getEndEffector(this);
            gripperEndAngle = armType.getGripperAngle();
        } else if (armState.getType() instanceof NetworkTablesAim) {
            NetworkTablesAim armType = (NetworkTablesAim) armState.getType();
            endEffector = armType.getEndEffector(this);
            gripperEndAngle = armType.getGripperAngle(this);
        }

        Logger.log("/ArmSubsystem/desiredEndEffector", new double[] {endEffector.getX(), endEffector.getY()});

        // Find the joint angles needed to reach the end effector
        Matrix<N3, N1> armAndWristAngles = inverseKinematics(endEffector, gripperEndAngle);
        joint1DesiredMotorPosition = MathUtils.ensureRange(
                armAndWristAngles.get(0, 0), ArmConstants.arm1MinimumAngle, ArmConstants.arm1MaximumAngle);
        joint2DesiredMotorPosition = MathUtils.ensureRange(
                armAndWristAngles.get(1, 0), ArmConstants.arm2MinimumAngle, ArmConstants.arm2MaximumAngle);
        gripperDesiredMotorPosition = armAndWristAngles.get(2, 0);
        if (gripperDesiredMotorPosition > GripperConstants.maximumAngle
                && gripperDesiredMotorPosition < GripperConstants.minimumAngle) {
            if (Math.abs(gripperDesiredMotorPosition - GripperConstants.maximumAngle)
                    < Math.abs(gripperDesiredMotorPosition - GripperConstants.minimumAngle)) {
                gripperDesiredMotorPosition = GripperConstants.maximumAngle;
            } else {
                gripperDesiredMotorPosition = GripperConstants.minimumAngle;
            }
        }
        ghostArm1.setAngle(Math.toDegrees(armAndWristAngles.get(0, 0)));
        ghostArm2.setAngle(Math.toDegrees(armAndWristAngles.get(1, 0)));
        ghostGripper.setAngle(Math.toDegrees(armAndWristAngles.get(2, 0)));

        // Calibrate integrated encoders each time we go to a new position
        calibrateIntegratedEncoders();

        // Update PID controllers
        motor1Controller.setGoal(joint1DesiredMotorPosition);
        motor2Controller.setGoal(joint2DesiredMotorPosition);
        gripperMotorController.setGoal(gripperDesiredMotorPosition);
    }

    public boolean isMastThroughBoreConnected() {
        return joint1AbsoluteEncoder.isConnected();
    }

    public boolean isBoomThroughBoreConnected() {
        return joint2AbsoluteEncoder.isConnected();
    }

    public boolean isWristThroughBoreConnected() {
        return gripperAbsoluteEncoder.isConnected();
    }

    private Rotation2d getGripperEncoderAngle() {
        double currentEncoderAngle = gripperAbsoluteEncoder.getAbsolutePosition();
        // ready position is approximately 90 degrees default starting position of the gripper is 90 degrees. When
        // booting, make sure gripper is between -90 and 270 degrees. (Not backwards.)
        double result = MathUtils.accomidateOverflow(
                lastAbsoluteGripper,
                ArmConstants.gripperEncoderMultiplier * currentEncoderAngle * 2 * Math.PI
                        + ArmConstants.gripperEncoderOffset,
                Math.PI * 2);
        lastAbsoluteGripper = result;
        return new Rotation2d(result);
    }

    private Rotation2d getJoint1EncoderAngle() {
        return putAngleInto180Scope(
                ArmConstants.mastEncoderMultiplier * joint1AbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI
                        + ArmConstants.mastEncoderOffset);
    }

    private Rotation2d getJoint2EncoderAngle() {
        // Invert this encoder because of mounting location
        return putAngleInto180Scope(
                ArmConstants.boomEncoderMultiplier * (1 - joint2AbsoluteEncoder.getAbsolutePosition()) * 2 * Math.PI
                        + ArmConstants.boomEncoderOffset);
    }

    @SuppressWarnings("unused")
    private Rotation2d getJoint1IntegratedAngle() {
        // These motors don't loop around after a full rotation so no need to change scope
        return new Rotation2d(
                Conversions.falconToRadians(joint1Motor.getSelectedSensorPosition(), ArmConstants.arm1GearRatio));
    }

    @SuppressWarnings("unused")
    private Rotation2d getJoint2IntegratedAngle() {
        // These motors don't loop around after a full rotation so no need to change scope
        return new Rotation2d(
                Conversions.falconToRadians(joint2Motor.getSelectedSensorPosition(), ArmConstants.arm2GearRatio));
    }

    private Rotation2d putAngleInto180Scope(double radians) {
        var rotationIn360Scope = new Rotation2d(radians);

        return new Rotation2d(rotationIn360Scope.getCos(), rotationIn360Scope.getSin());
    }

    private void calibrateIntegratedEncoders() {
        joint1Motor.setSelectedSensorPosition(
                Conversions.radiansToFalcon(getJoint1EncoderAngle().getRadians(), ArmConstants.arm1GearRatio));

        // Inverted because inverted motor needs uninverted measurement first
        joint2Motor.setSelectedSensorPosition(
                Conversions.radiansToFalcon(-getJoint2EncoderAngle().getRadians(), ArmConstants.arm2GearRatio));
    }

    public boolean isArmAtGoal() {
        return MathUtils.equalsWithinError(
                        arm1Angle.getRadians(), joint1DesiredMotorPosition, ArmConstants.angularTolerance)
                && MathUtils.equalsWithinError(
                        arm2Angle.getRadians(), joint2DesiredMotorPosition, ArmConstants.angularTolerance)
                && MathUtils.equalsWithinError(
                        MathUtil.angleModulus(gripperAngle.getRadians() - gripperDesiredMotorPosition),
                        0,
                        ArmConstants.gripperAngularTolerance * 2);
    }

    public boolean isArmAtHandoffGoal() {
        return MathUtils.equalsWithinError(
                        arm1Angle.getRadians(), joint1DesiredMotorPosition, ArmConstants.angularTolerance * 1.3)
                && MathUtils.equalsWithinError(
                        arm2Angle.getRadians(), joint2DesiredMotorPosition, ArmConstants.angularTolerance * 1.3)
                && MathUtils.equalsWithinError(
                        MathUtil.angleModulus(gripperAngle.getRadians() - gripperDesiredMotorPosition),
                        0,
                        ArmConstants.gripperAngularTolerance * 2);
    }

    public boolean isArmApproximatelyAtGoal() {
        return MathUtils.equalsWithinError(
                        arm1Angle.getRadians(), joint1DesiredMotorPosition, ArmConstants.angularTolerance * 2.5)
                && MathUtils.equalsWithinError(
                        arm2Angle.getRadians(), joint2DesiredMotorPosition, ArmConstants.angularTolerance * 2.5);
        // && MathUtils.equalsWithinError(
        //         gripperAngle.getRadians(), gripperDesiredMotorPosition, ArmConstants.angularTolerance * 2.5);
    }

    public boolean isArmWithinCustomTolerances(double toleranceMast, double toleranceBoom) {
        return MathUtils.equalsWithinError(arm1Angle.getRadians(), joint1DesiredMotorPosition, toleranceMast)
                && MathUtils.equalsWithinError(arm2Angle.getRadians(), joint2DesiredMotorPosition, toleranceBoom);
    }

    private void passthroughMotorSpeeds(double shoulderPercent, double elbowPercent, double wristPercent) {
        joint1Motor.set(shoulderPercent);
        joint2Motor.set(elbowPercent);
        gripperMotor.set(wristPercent);
    }

    public Command passthroughCommand(
            DoubleSupplier shoulderPercent, DoubleSupplier elbowPercent, DoubleSupplier wristSupplier) {
        return run(() -> {
            setState(ArmState.PASSTHROUGH);
            passthroughMotorSpeeds(
                    shoulderPercent.getAsDouble() * 0.1,
                    elbowPercent.getAsDouble() * -0.1,
                    wristSupplier.getAsDouble() * 0.1);
        });
    }

    @Override
    public void periodic() {
        var startTimeMS = Timer.getFPGATimestamp() * 1000;

        // Update internal model with real motor values
        if (Robot.isReal()) {
            lastArm1Position = arm1Angle;
            lastArm2Position = arm2Angle;
            lastGripperPosition = gripperAngle;

            arm1Angle = getJoint1EncoderAngle();
            // arm1Angle = getJoint1IntegratedAngle();
            arm2Angle = getJoint2EncoderAngle();
            gripperAngle = getGripperEncoderAngle();

            arm1Speed = arm1Angle.minus(lastArm1Position).getRadians() / 0.02;
            arm2Speed = arm2Angle.minus(lastArm2Position).getRadians() / 0.02;

            arm1Speed =
                    arm1SpeedFilter.calculate(arm1Angle.minus(lastArm1Position).getRadians() / 0.02);
            arm2Speed =
                    arm2SpeedFilter.calculate(arm2Angle.minus(lastArm2Position).getRadians() / 0.02);
            gripperSpeed = gripperSpeedFilter.calculate(
                    gripperAngle.minus(lastGripperPosition).getRadians() / 0.02);
        }

        // Update Mechanism2d widget
        arm1.setAngle(arm1Angle.getDegrees());
        arm2.setAngle(arm2Angle.getDegrees());
        gripper.setAngle(gripperAngle.getDegrees());

        // Run inverse kinematics and update the desired joint angles
        if (armState.getType() instanceof Dynamic || armState.getType() instanceof NetworkTablesAim) {
            updateArmDesiredPosition();
        }

        // Enable brake mode when the joints are at the right position
        if ((isArmAtGoal() || armState.getType() instanceof Brake) && !(armState.getType() instanceof PassthroughAim)) {
            // stopMotors();

            calibrateIntegratedEncoders();
        } else {
            startMotors();
        }

        // Run the PIDF system unless we are in one of the "special" modes
        if (armState != ArmState.COAST && armState != ArmState.BRAKE && armState != ArmState.PASSTHROUGH) {
            executePIDFeedforward();

            if (armState == ArmState.AWAITING_DEPLOYMENT && isArmApproximatelyAtGoal()) {
                joint1Motor.stopMotor();
            }

            // if (armState == ArmState.AWAITING_DEPLOYMENT && isArmAtGoal()) {
            //     // backwards is positive for mast and negative for boom
            //     // run this only when we are decelerating in the direction of the arm or accelerating in the opposite
            //     // direction
            //     if (Math.abs(swerveDriveSubsystem.getVelocityMagnitude()) > 0.05) {
            //         joint1Motor.stopMotor();
            //         joint2Motor.set(-0.06);
            //     } else {
            //         stopMotors();
            //     }
            // } else {
            //     executePIDFeedforward();
            // }
        }

        // Logger.log("/ArmSubsystem/arm1Percent", joint1Motor.get());
        // Logger.log("/ArmSubsystem/arm2Percent", joint2Motor.get());

        Logger.log("/ArmSubsystem/arm1Angle", arm1Angle.getRadians());
        Logger.log("/ArmSubsystem/arm2Angle", arm2Angle.getRadians());
        // Logger.log("/ArmSubsystem/arm1Speed", arm1Speed);
        // Logger.log("/ArmSubsystem/arm2Speed", arm2Speed);
        Logger.log("/ArmSubsystem/gripperAngle", gripperAngle.getRadians());
        // Logger.log("/ArmSubsystem/gripperSpeed", gripperSpeed);
        // Logger.log("/ArmSubsystem/arm1DesiredPosition", joint1DesiredMotorPosition);
        // Logger.log("/ArmSubsystem/arm2DesiredPosition", joint2DesiredMotorPosition);
        // Logger.log("/ArmSubsystem/gripperDesiredPosition", gripperDesiredMotorPosition);
        Logger.log("/ArmSubsystem/encoders/gripperRaw", gripperAbsoluteEncoder.get() * 2 * Math.PI);
        Logger.log("/ArmSubsystem/encoders/arm1Raw", joint1AbsoluteEncoder.get() * 2 * Math.PI);
        Logger.log("/ArmSubsystem/encoders/arm2Raw", joint2AbsoluteEncoder.get() * 2 * Math.PI);
        // Logger.log("/ArmSubsystem/isBraking", brakingActivated);
        // Logger.log("/ArmSubsystem/isCoasting", armState == ArmState.COAST);
        Logger.log("/ArmSubsystem/isArmAtPosition", isArmAtGoal());
        Logger.log("/ArmSubsystem/isArmAtHandoffPosition", isArmAtHandoffGoal());

        Logger.log("/ArmSubsystem/LoopDuration", Timer.getFPGATimestamp() * 1000 - startTimeMS);
    }

    public Command overrideGripper(double desiredSpeed) {
        return runEnd(() -> desiredGripperSpeedOverride = Optional.of(desiredSpeed), () -> {
            desiredGripperSpeedOverride = Optional.empty();
            lastAbsoluteGripper = Math.PI / 2;
        });
    }

    private void executePIDFeedforward() {
        // Calculate PID outputs
        double arm1VoltageCorrection = motor1Controller.calculate(arm1Angle.getRadians());
        double arm2VoltageCorrection = motor2Controller.calculate(arm2Angle.getRadians());
        double wristVoltageCorrection = gripperMotorController.calculate(gripperAngle.getRadians());

        // Combine motion profile velocity setpoints with PID corrections
        double arm1DesiredSpeed = motor1Controller.getSetpoint().velocity;
        double arm2DesiredSpeed = motor2Controller.getSetpoint().velocity;
        double gripperDesiredSpeed = gripperMotorController.getSetpoint().velocity;

        // Calculate feedforward voltages from dynamics
        double[] ffVoltages = feedforward.calculateFeedforwardVoltages(
                arm1Angle.getRadians(), arm2Angle.getRadians(), arm1DesiredSpeed, arm2DesiredSpeed, 0, 0);

        // Calculate the wrist motor feedforward voltage
        double gripperVoltage = gripperJointFeedforward.calculate(
                gripperAngle.plus(arm1Angle).plus(arm2Angle).getRadians(), gripperDesiredSpeed, 0);

        // Combine pid corrections and feedforward, limiting the max voltage to prevent brownouts
        double arm1VoltageOutput = calculateShockFeedforwardArm1Voltage(arm1VoltageCorrection, ffVoltages);

        double arm2VoltageOutput = MathUtils.ensureRange(
                applyKs(arm2VoltageCorrection, ArmConstants.arm2kS, ArmConstants.arm2kSDeadband)
                        + ffVoltages[1]
                        + (armState == ArmState.HOVER_MANUAL ? .1 : 0)
                        + (armState == ArmState.SUBSTATION_PICKUP ? .4 : 0),
                -ArmConstants.maxVoltage,
                ArmConstants.maxVoltage);
        double gripperVoltageOutput = MathUtils.ensureRange(
                wristVoltageCorrection + gripperVoltage, -ArmConstants.maxVoltage, ArmConstants.maxVoltage);

        // this is hopefully temporary and should go away someday
        if (desiredGripperSpeedOverride.isPresent()) {
            gripperVoltageOutput = gripperJointFeedforward.calculate(0, desiredGripperSpeedOverride.get(), 0);
        }

        joint1Motor.set(arm1VoltageOutput / GlobalConstants.targetVoltage);
        joint2Motor.set(arm2VoltageOutput / GlobalConstants.targetVoltage);
        gripperMotor.set(gripperVoltageOutput / GlobalConstants.targetVoltage);

        Logger.log("/ArmSubsystem/arm1AngleSetpoint", motor1Controller.getSetpoint().position);
        Logger.log("/ArmSubsystem/arm2AngleSetpoint", motor2Controller.getSetpoint().position);
        Logger.log("/ArmSubsystem/gripperPositionSetpoint", gripperMotorController.getSetpoint().position);

        Logger.log("/ArmSubsystem/arm1VoltageFeedforward", ffVoltages[0]);
        Logger.log("/ArmSubsystem/arm2VoltageFeedforward", ffVoltages[1]);
        Logger.log("/ArmSubsystem/gripperVoltageFeedforward", gripperVoltage);

        Logger.log("/ArmSubsystem/arm1VoltageCorrection", arm1VoltageCorrection);
        Logger.log("/ArmSubsystem/arm2VoltageCorrection", arm2VoltageCorrection);
        Logger.log("/ArmSubsystem/gripperVoltageCorrection", wristVoltageCorrection);

        // Logger.log("/ArmSubsystem/arm1SpeedSetpoint", motor1Controller.getSetpoint().velocity);
        // Logger.log("/ArmSubsystem/arm2SpeedSetpoint", motor2Controller.getSetpoint().velocity);
        // Logger.log("/ArmSubsystem/gripperSpeedSetpoint", gripperMotorController.getSetpoint().velocity);
    }

    private double shockFeedforward(Rotation2d armAngle) {
        double torque = springConstant
                * (ArmConstants.l1 * ArmConstants.l2 * armAngle.getSin())
                / Math.sqrt(Math.pow(ArmConstants.l1, 2)
                        + Math.pow(ArmConstants.l2, 2)
                        - 2 * ArmConstants.l1 * ArmConstants.l2 * armAngle.getCos());

        double gasShockFeedforwardVoltage = DCMotor.getFalcon500(1)
                .withReduction(ArmConstants.arm1GearRatio)
                .getVoltage(-torque, 0);

        Logger.log("/ArmSubsystem/shockFeedforward", gasShockFeedforwardVoltage);

        return gasShockFeedforwardVoltage;
    }

    private double calculateShockFeedforwardArm1Voltage(double arm1VoltageCorrection, double[] ffVoltages) {
        double gasShockFeedforwardVoltage = shockFeedforward(arm1Angle);

        double arm1VoltageOutput = MathUtils.ensureRange(
                arm1VoltageCorrection + ffVoltages[0] + gasShockFeedforwardVoltage,
                -ArmConstants.maxVoltage,
                ArmConstants.maxVoltage);
        return arm1VoltageOutput;
    }

    private static double applyKs(double volts, double kS, double kSDeadband) {
        if (Math.abs(volts) < kSDeadband) return volts;

        return volts + Math.copySign(kS, volts);
    }

    @Override
    public void simulationPeriodic() {
        if (brakingActivated == true) return;

        Matrix<N2, N1> angles = VecBuilder.fill(arm1Angle.getRadians(), arm2Angle.getRadians());
        Matrix<N2, N1> speeds = VecBuilder.fill(arm1Speed, arm2Speed);
        Matrix<N2, N1> voltages = VecBuilder.fill(
                joint1Motor.get() * GlobalConstants.targetVoltage, joint2Motor.get() * GlobalConstants.targetVoltage);
        Matrix<N2, N1> acceleration;
        if (armState == ArmState.COAST) {
            acceleration = simFeedforward
                    .calculateArmInertiaMatrix(angles)
                    .inv()
                    .times(simFeedforward
                            .calculateGravityMatrix(angles)
                            .plus(simFeedforward
                                    .calculateCoriolisMatrix(speeds, angles)
                                    .times(speeds))
                            .times(-1));
        } else {
            acceleration = simFeedforward
                    .calculateArmInertiaMatrix(angles)
                    .inv()
                    .times((simFeedforward.calculateMotorTorqueMatrix().times(voltages))
                            .minus(simFeedforward
                                    .calculateCoriolisMatrix(speeds, angles)
                                    .times(speeds))
                            .minus(simFeedforward.calculateGravityMatrix(angles))
                            .minus(simFeedforward.calculateBackEmfMatrix().times(speeds)));
        }

        angles = angles.plus(speeds.times(0.02)).plus(acceleration.times(.5 * 0.02 * 0.02));
        speeds = speeds.plus(acceleration.times(0.02)).times(1);

        arm1Angle = new Rotation2d(angles.get(0, 0));
        arm2Angle = new Rotation2d(angles.get(1, 0));
        gripperAngle = gripperAngle.plus(new Rotation2d(gripperMotorController.getSetpoint().velocity * 0.02));

        arm1Speed = speeds.get(0, 0);
        arm2Speed = speeds.get(1, 0);
        gripperSpeed = gripperMotorController.getSetpoint().velocity + gripperMotor.get() / 10;
    }

    public Translation2d getDynamicArmPosition() {
        PlacementLocation targetLocation = FieldConstants.getNearestPlacementLocation(swerveDriveSubsystem.getPose());

        ArmState armState = getState();
        Pose3d targetPose3d;

        switch (armState) {
            case HYBRID:
                targetPose3d = targetLocation.getHybridPose();
                break;
            case MID:
                targetPose3d = targetLocation.getMidPose();
                break;
            case HIGH:
                targetPose3d = targetLocation.getHighPose();
                break;
            default:
                targetPose3d = targetLocation.getHybridPose();
                break;
        }

        Translation3d robotToGoalTranslation =
                targetPose3d.minus(new Pose3d(swerveDriveSubsystem.getPose())).getTranslation();

        Translation2d targetEndEffector = new Translation2d(
                Math.hypot(robotToGoalTranslation.getX(), robotToGoalTranslation.getY())
                        - ArmConstants.robotToArm.getY(),
                robotToGoalTranslation.getZ() - ArmConstants.robotToArm.getZ());

        return targetEndEffector;
    }

    public Translation2d getNetworkTablesArmPosition() {
        double[] armPosition = desiredNetworkTablesArmPosition.getDoubleArray();
        try {
            return new Translation2d(armPosition[0], armPosition[1]);
        } catch (Exception e) {
            return new Translation2d(0, 0);
        }
    }

    public Rotation2d getNetworkTablesGripperRotation() {
        double[] armPosition = desiredNetworkTablesArmPosition.getDoubleArray();
        try {
            return Rotation2d.fromDegrees(armPosition[2]);
        } catch (Exception e) {
            return Rotation2d.fromDegrees(0);
        }
    }

    public Transform3d getArmEndEffectorTransform3d() {
        Translation2d endEffector = forwardKinematics(
                arm1.getLength(), arm1Angle, arm2.getLength(), arm2Angle, gripper.getLength(), gripperAngle);
        double gripperAbsoluteRotation =
                arm1Angle.plus(arm2Angle).plus(gripperAngle).getRadians();
        return new Transform3d(
                new Translation3d(endEffector.getX(), 0, endEffector.getY()),
                new Rotation3d(0, gripperAbsoluteRotation, 0));
    }

    public Command armStateCommand(ArmState armState) {
        return runOnce(() -> setState(armState)).andThen(Commands.waitUntil(this::isArmAtGoal));
    }

    public Command armHandoffStateCommand(ArmState armState) {
        return runOnce(() -> setState(armState)).andThen(Commands.waitUntil(this::isArmAtHandoffGoal));
    }

    public Command armStateApproximateCommand(ArmState armState) {
        return runOnce(() -> setState(armState)).andThen(Commands.waitUntil(this::isArmApproximatelyAtGoal));
    }

    public Command armSequence(ArmState... armStates) {
        return Commands.sequence(Stream.of(armStates)
                .map((state) -> armStateCommand(state))
                .toList()
                .toArray(new Command[armStates.length]));
    }

    public ArmState getState() {
        return armState;
    }

    public enum ArmState {
        AWAITING_DEPLOYMENT_1(Static.fromWrist(0.2, 0.3, Rotation2d.fromDegrees(20))),
        AWAITING_DEPLOYMENT(Static.fromWrist(0.091, 0.27, Rotation2d.fromDegrees(53))),
        SHOOT_HYBRID(Static.fromWrist(0.091, 0.27, Rotation2d.fromDegrees(0))),
        SHOOT_MID(Static.fromWrist(0.091, 0.27, Rotation2d.fromDegrees(40))),
        SHOOT_HIGH(new Static(0.88, 0.8, Rotation2d.fromDegrees(40))),
        // SUBSTATION_PICKUP(Static.fromWrist(0.21, 0.34, Rotation2d.fromDegrees(67))),  // starting
        SLIDE_PICKUP(Static.fromWrist(0.19, 0.32, Rotation2d.fromDegrees(55))),
        // SLIDE_PICKUP_COMP(Static.fromWrist(0.21, 0.33, Rotation2d.fromDegrees(50))),
        HYBRID_MANUAL(new Static(0.97, -0.08, Rotation2d.fromDegrees(-10))),
        // HYBRID_MANUAL(new Static(0.8, -0.08, Rotation2d.fromDegrees(-10))), // experimental
        TIPPED_CONE_MANUAL(new Static(0.8, -0.12, Rotation2d.fromDegrees(-90))),
        // MID_MANUAL(Static.fromBumper(
        //         FieldConstants.midX,
        //         FieldConstants.midConeZ + ArmConstants.placementHeightOffset,
        //         Rotation2d.fromDegrees(20))),
        SUBSTATION_PICKUP(Static.fromBumper(
                FieldConstants.midX, FieldConstants.highConeZ - 0.07 + 0.0, Rotation2d.fromDegrees(0))), // -10
        // MID_MANUAL_CONE(Static.fromBumper(
        //         FieldConstant[\]s.midX + 0.12, FieldConstants.highConeZ - 0.09, Rotation2d.fromDegrees(60))),
        // MID_MANUAL_CUBE_1(new Static(0.66, 0.70, Rotation2d.fromDegrees(55))),
        // MID_MANUAL_CUBE(Static.fromBumper(
        //         FieldConstants.midX + 0.10, FieldConstants.midCubeZ + 0.30, Rotation2d.fromDegrees(-20))),
        MID_MANUAL_CONE(Static.fromBumper(
                FieldConstants.midX + 0.12, FieldConstants.highConeZ - 0.05 - 0.08, Rotation2d.fromDegrees(60))),
        MID_MANUAL_CUBE_1(new Static(0.66, 0.70, Rotation2d.fromDegrees(55))),
        MID_MANUAL_CUBE(Static.fromBumper(
                FieldConstants.midX + 0.08, FieldConstants.midCubeZ + 0.35, Rotation2d.fromDegrees(-10))), // -20
        MID_MANUAL_CUBE_NEW(Static.fromBumper(
                FieldConstants.midX + 0.08, FieldConstants.midCubeZ + 0.45, Rotation2d.fromDegrees(-20))),
        HIGH_MANUAL_1(new Static(0.9, 1.24, Rotation2d.fromDegrees(70))),
        // HIGH_MANUAL_1(new Static(0.9, 1.24, Rotation2d.fromDegrees(70))),
        // HIGH_MANUAL_1(new Static(0.9, 1.32, Rotation2d.fromDegrees(65))),
        HOVER_MANUAL(Static.fromBumper(
                FieldConstants.highX + -0.06, // + 0.1, // gripper offset
                FieldConstants.highConeZ
                        + ArmConstants.placementHeightOffset
                        + 0.3
                        - 0.10, // because of poor pid behavior
                Rotation2d.fromDegrees(24))), // 18
        HIGH_MANUAL_CONE(Static.fromBumper(
                FieldConstants.highX + 0.01 + 0.02, // + 0.1, // gripper offset
                FieldConstants.highConeZ
                        + ArmConstants.placementHeightOffset
                        + 0.3
                        - 0.17
                        + 0.04, // because of poor pid behavior // -0.20
                Rotation2d.fromDegrees(25))), // 18
        // HIGH_MANUAL_CONE(Static.fromBumper(
        //         FieldConstants.highX + 0.035 - 0.08, // + 0.1, // gripper offset
        //         FieldConstants.highConeZ
        //                 + ArmConstants.placementHeightOffset
        //                 + 0.3
        //                 - 0.18, // because of poor pid behavior
        //         Rotation2d.fromDegrees(24))), // 18
        HIGH_MANUAL_CUBE(Static.fromBumper(
                FieldConstants.highX + 0.14, // gripper offset
                FieldConstants.highCubeZ + ArmConstants.placementHeightOffset + 0.3, // because of poor pid behavior
                Rotation2d.fromDegrees(-25))),
        COOL_HANDOFF(Static.fromWrist(0.091, 0.27, Rotation2d.fromDegrees(163))), // 156
        COOL_HANDOFF_REVERSE(Static.fromWrist(0.4, 0.4, Rotation2d.fromDegrees(170))),
        HYBRID(new Dynamic(sus -> sus.getDynamicArmPosition(), new Rotation2d())),
        MID(new Dynamic(sussy -> sussy.getDynamicArmPosition(), new Rotation2d())),
        HIGH(new Dynamic(sussier -> sussier.getDynamicArmPosition(), new Rotation2d())),
        NETWORK_TABLES_AIM(new NetworkTablesAim()),
        PASSTHROUGH(new PassthroughAim()),
        COAST(new Coast()),
        BRAKE(new Brake());

        private Object type;

        public Object getType() {
            return type;
        }

        private ArmState(Object type) {
            this.type = type;
        }
    }

    private static class Brake {}

    private static class Coast {}

    private static class Static {
        private Translation2d endEffector;
        private Rotation2d angle;

        public static Static fromBumper(double absoluteX, double absoluteY, Rotation2d gripperAngle) {
            return new Static(
                    FieldConstants.outerX
                            - absoluteX
                            + SwerveConstants.lengthWithBumpers / 2
                            - ArmConstants.robotToArm.getX(),
                    absoluteY - ArmConstants.robotToArm.getZ(),
                    gripperAngle);
        }

        public static Static fromWrist(double x, double y, Rotation2d gripperAngle) {
            return new Static(
                    x + gripperAngle.getCos() * GripperConstants.length,
                    y + gripperAngle.getSin() * GripperConstants.length,
                    gripperAngle);
        }

        public Static(double x, double y, Rotation2d gripperAngle) {
            endEffector = new Translation2d(x, y);
            angle = gripperAngle;
        }

        public Translation2d getEndEffector() {
            return endEffector;
        }

        public Rotation2d getGripperAngle() {
            return angle;
        }

        @SuppressWarnings("unused")
        public Translation2d getWrist() {
            return new Translation2d(
                    endEffector.getX() - angle.getCos() * GripperConstants.length,
                    endEffector.getY() - angle.getSin() * GripperConstants.length);
        }
    }

    private static class Dynamic {
        private Function<ArmSubsystem, Translation2d> endEffectorSupplier;

        private Rotation2d angle;

        public Dynamic(Function<ArmSubsystem, Translation2d> endEffectorSupplier, Rotation2d gripperAngle) {
            this.endEffectorSupplier = endEffectorSupplier;
            angle = gripperAngle;
        }

        public Translation2d getEndEffector(ArmSubsystem armSubsystem) {
            return endEffectorSupplier.apply(armSubsystem);
        }

        public Rotation2d getGripperAngle() {
            return angle;
        }
    }

    private static class NetworkTablesAim {
        public Translation2d getEndEffector(ArmSubsystem armSubsystem) {
            return armSubsystem.getNetworkTablesArmPosition();
        }

        public Rotation2d getGripperAngle(ArmSubsystem armSubsystem) {
            return armSubsystem.getNetworkTablesGripperRotation();
        }
    }

    private static class PassthroughAim {}
}
