package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
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
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;

public class ArmSubsystem extends SubsystemBase {
    private Mechanism2d mechanism = new Mechanism2d(4, 4);
    private MechanismRoot2d root = mechanism.getRoot("Arm", 2, 2);
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
    private double gripperSpeed = 0;

    private LinearFilter arm1SpeedFilter = LinearFilter.movingAverage(10);
    private LinearFilter arm2SpeedFilter = LinearFilter.movingAverage(10);
    private LinearFilter arm1SpeedFilter2 = LinearFilter.movingAverage(10);
    private LinearFilter arm2SpeedFilter2 = LinearFilter.movingAverage(10);

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

    private static final TrapezoidProfile.Constraints motor1Constraints = new TrapezoidProfile.Constraints(5, 4);
    private static final TrapezoidProfile.Constraints motor2Constraints = new TrapezoidProfile.Constraints(3, 1);
    private static final TrapezoidProfile.Constraints wristProfileConstraints = new TrapezoidProfile.Constraints(3, 3);
    // private static final TrapezoidProfile.Constraints motor1Constraints = new TrapezoidProfile.Constraints(12, 6);
    // private static final TrapezoidProfile.Constraints motor2Constraints = new TrapezoidProfile.Constraints(12, 6);
    // private static final TrapezoidProfile.Constraints wristProfileConstraints = new TrapezoidProfile.Constraints(12,
    // 6);

    private Translation2d endEffector = new Translation2d();
    private Rotation2d gripperEndAngle = new Rotation2d();

    private ArmState armState = ArmState.AWAITING_DEPLOYMENT;
    private boolean doCycleMode = false;
    private Timer cycleModeTimer = new Timer();
    private int cycleModeIndex = 0;

    private TwoJointedFourBarArmFeedforward simFeedforward;
    private TwoJointedFourBarArmFeedforward feedforward;
    private ArmFeedforward gripperJointFeedforward;

    private Supplier<Pose2d> robotPoseSupplier;

    public ArmSubsystem(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;

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

        arm1Angle = ArmConstants.arm1StartingAngle;
        arm2Angle = ArmConstants.arm2StartingAngle;
        gripperAngle = GripperConstants.startingAngle;

        joint1Motor = new WPI_TalonFX(ArmConstants.mastMotorPort, GlobalConstants.CANIVORE_NAME);
        joint2Motor = new WPI_TalonFX(ArmConstants.boomMotorPort, GlobalConstants.CANIVORE_NAME);
        gripperMotor = new WPI_TalonSRX(ArmConstants.wristMotorPort); // wrist motor

        joint1Motor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        joint2Motor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        gripperMotor.configVoltageCompSaturation(GlobalConstants.targetVoltage);
        joint1Motor.enableVoltageCompensation(true);
        joint2Motor.enableVoltageCompensation(true);
        gripperMotor.enableVoltageCompensation(true);

        // Configure motor soft limits for safety
        // joint1Motor.configForwardSoftLimitEnable(true);
        // joint1Motor.configForwardSoftLimitThreshold(Conversions.radiansToFalcon(ArmConstants.arm1MaximumAngle,
        // ArmConstants.arm1GearRatio));
        // joint1Motor.configReverseSoftLimitEnable(true);
        // joint1Motor.configReverseSoftLimitThreshold(Conversions.radiansToFalcon(ArmConstants.arm1MinimumAngle,
        // ArmConstants.arm1GearRatio));

        // joint2Motor.configForwardSoftLimitEnable(true);
        // joint2Motor.configForwardSoftLimitThreshold(Conversions.radiansToFalcon(ArmConstants.arm2MaximumAngle,
        // ArmConstants.arm2GearRatio));
        // joint2Motor.configReverseSoftLimitEnable(true);
        // joint2Motor.configReverseSoftLimitThreshold(Conversions.radiansToFalcon(ArmConstants.arm2MinimumAngle,
        // ArmConstants.arm2GearRatio));

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
                9.81);

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
                9.81);

        gripperJointFeedforward =
                new ArmFeedforward(GripperConstants.ks, GripperConstants.kg, GripperConstants.kv, GripperConstants.ka);

        motor1Controller = new ProfiledPIDController(0.65, 0, 0.01, motor1Constraints);
        motor2Controller = new ProfiledPIDController(0.2, 0, 0.05, motor2Constraints);
        gripperMotorController = new ProfiledPIDController(0.3, 0, 0, wristProfileConstraints);

        resetPIDControllers();

        // Don't die when arm is backwards slightly
        motor2Controller.enableContinuousInput(-Math.PI, Math.PI);

        motor1Controller.setTolerance(ArmConstants.angularTolerance);
        motor2Controller.setTolerance(ArmConstants.angularTolerance);
        gripperMotorController.setTolerance(ArmConstants.angularTolerance);

        cycleModeTimer.start();

        setState(armState);

        // Translation2d(X: -0.12, Y: 0.50)
        // System.out.println(forwardKinematics(arm1.getLength(), new Rotation2d(0.873), arm2.getLength(), new
        // Rotation2d(-2.4),
        // gripper.getLength(), new Rotation2d()));
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
        gripperDesiredMotorPosition = MathUtils.ensureRange(
                armAndWristAngles.get(2, 0), GripperConstants.minimumAngle, GripperConstants.maximumAngle);
        ghostArm1.setAngle(Math.toDegrees(armAndWristAngles.get(0, 0)));
        ghostArm2.setAngle(Math.toDegrees(armAndWristAngles.get(1, 0)));
        ghostGripper.setAngle(Math.toDegrees(armAndWristAngles.get(2, 0)));

        // Calibrate integrated encoders each time we go to a new position
        calibrateIntegratedEncoders();
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
        return putAngleInto180Scope(
                ArmConstants.gripperEncoderMultiplier * gripperAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI
                        + ArmConstants.gripperEncoderOffset);

        // return new Rotation2d(ArmConstants.gripperEncoderMultiplier * gripperAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI
        // + ArmConstants.gripperEncoderOffset);
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

    private Rotation2d getJoint1IntegratedAngle() {
        // These motors don't loop around after a full rotation so no need to change scope
        return new Rotation2d(
                Conversions.falconToRadians(joint1Motor.getSelectedSensorPosition(), ArmConstants.arm1GearRatio));
    }

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

    private boolean isArmAtGoal() {
        return MathUtils.equalsWithinError(
                        arm1Angle.getRadians(), joint1DesiredMotorPosition, ArmConstants.angularTolerance)
                && MathUtils.equalsWithinError(
                        arm2Angle.getRadians(), joint2DesiredMotorPosition, ArmConstants.angularTolerance)
                && MathUtils.equalsWithinError(
                        gripperAngle.getRadians(), gripperDesiredMotorPosition, ArmConstants.angularTolerance);
    }

    private void passthroughMotorSpeeds(double shoulderPercent, double elbowPercent, double wristPercent) {
        joint1Motor.set(shoulderPercent);
        joint2Motor.set(elbowPercent);
        gripperMotor.set(wristPercent);
    }

    public Command passthroughCommand(
            DoubleSupplier shoulderPercent, DoubleSupplier elbowPercent, DoubleSupplier wristSupplier) {
        // return runEnd(
        //                 () -> {
        //                     setPassthrough();
        //                     passthroughMotorSpeeds(
        //                         shoulderPercent.getAsDouble(), elbowPercent.getAsDouble(),
        // wristSupplier.getAsDouble());
        //                 },
        //                 () -> setAwaitingPiece());

        return run(() -> {
            setPassthrough();
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

            arm1Angle = getJoint1IntegratedAngle();
            arm2Angle = getJoint2EncoderAngle();
            gripperAngle = getGripperEncoderAngle();

            arm1Speed = arm1Angle.minus(lastArm1Position).getRadians() / 0.02;
            arm2Speed = arm2Angle.minus(lastArm2Position).getRadians() / 0.02;

            // arm1Speed = arm1SpeedFilter2.calculate(
            //         arm1SpeedFilter.calculate(arm1Angle.minus(lastArm1Position).getRadians() / 0.02));
            // arm2Speed = arm2SpeedFilter2.calculate(
            //         arm2SpeedFilter.calculate(arm2Angle.minus(lastArm2Position).getRadians() / 0.02));
            gripperSpeed = gripperAngle.minus(lastGripperPosition).getRadians() / 0.02;
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
            if (armState == ArmState.AWAITING_DEPLOYMENT && isArmAtGoal()) {
                // backwards is positive for mast and negative for boom
                joint1Motor.set(0.05);
                joint2Motor.set(-0.037);
            } else {
                executePIDFeedforward();
            }
        }

        // We still have this for some reason
        // if (doCycleMode && cycleModeTimer.advanceIfElapsed(3)) {
        //     setState(ArmState.values()[cycleModeIndex]);
        //     cycleModeIndex++;
        //     cycleModeIndex %= ArmState.values().length;
        // }


        Logger.log("/ArmSubsystem/arm1Percent", joint1Motor.get());
        Logger.log("/ArmSubsystem/arm2Percent", joint2Motor.get());

        Logger.log("/ArmSubsystem/arm1Angle", arm1Angle.getRadians());
        Logger.log("/ArmSubsystem/arm2Angle", arm2Angle.getRadians());
        // Logger.log("/ArmSubsystem/arm1Speed", arm1Speed);
        // Logger.log("/ArmSubsystem/arm2Speed", arm2Speed);
        Logger.log("/ArmSubsystem/gripperAngle", gripperAngle.getRadians());
        // Logger.log("/ArmSubsystem/gripperSpeed", gripperSpeed);
        Logger.log("/ArmSubsystem/arm1DesiredPosition", joint1DesiredMotorPosition);
        Logger.log("/ArmSubsystem/arm2DesiredPosition", joint2DesiredMotorPosition);
        Logger.log("/ArmSubsystem/gripperDesiredPosition", gripperDesiredMotorPosition);
        // Logger.log("/ArmSubsystem/arm1EncoderPosition", getJoint1EncoderAngle().getRadians());
        // Logger.log("/ArmSubsystem/arm2EncoderPosition", getJoint2EncoderAngle().getRadians());
        // Logger.log("/ArmSubsystem/isBraking", brakingActivated);
        // Logger.log("/ArmSubsystem/isCoasting", armState == ArmState.COAST);
        Logger.log("/ArmSubsystem/isArmAtPosition", isArmAtGoal());

        Logger.log("/ArmSubsystem/LoopDuration", Timer.getFPGATimestamp() * 1000 - startTimeMS);
    }

    private void executePIDFeedforward() {
        motor1Controller.setGoal(joint1DesiredMotorPosition);
        motor2Controller.setGoal(joint2DesiredMotorPosition);
        gripperMotorController.setGoal(gripperDesiredMotorPosition);

        // Calculate PID outputs
        double arm1VoltageCorrection = motor1Controller.calculate(arm1Angle.getRadians());
        double arm2VoltageCorrection = motor2Controller.calculate(arm2Angle.getRadians());
        double wristVoltageCorrection = gripperMotorController.calculate(gripperAngle.getRadians());

        // Combine motion profile velocity setpoints with PID corrections
        double arm1DesiredSpeed = motor1Controller.getSetpoint().velocity;
        double arm2DesiredSpeed = motor2Controller.getSetpoint().velocity;
        double gripperDesiredSpeed = gripperMotorController.getSetpoint().velocity;

        // Calculate the arm motor feedforward voltages
        // double[] ffVoltages = feedforward.calculateFeedforwardVoltages(
        //         arm1Angle.getRadians(),
        //         arm2Angle.getRadians(),
        //         arm1DesiredSpeed,
        //         arm2DesiredSpeed,
        //         (arm1DesiredSpeed - arm1Speed) / 0.02,
        //         (arm2DesiredSpeed - arm2Speed) / 0.02);

        // A little bounce at the end
        double[] ffVoltages = feedforward.calculateFeedforwardVoltages(
                arm1Angle.getRadians(), arm2Angle.getRadians(), arm1DesiredSpeed, arm2DesiredSpeed, 0, 0);

        // Pretty darn close
        // double[] ffVoltages = feedforward.calculateFeedforwardVoltages(
        //         arm1Angle.getRadians(),
        //         arm2Angle.getRadians(),
        //         0,
        //         0,
        //         0,
        //         0);

        // Calculate the wrist motor voltages
        double gripperVoltage = gripperJointFeedforward.calculate(
                gripperAngle.plus(arm1Angle).plus(arm2Angle).getRadians(), gripperDesiredSpeed, 0);

        joint1Motor.set(arm1VoltageCorrection + MathUtils.ensureRange(ffVoltages[0] / 12, -1, 1));
        joint2Motor.set(arm2VoltageCorrection + MathUtils.ensureRange(ffVoltages[1] / 12, -1, 1));
        gripperMotor.set(wristVoltageCorrection + MathUtils.ensureRange(gripperVoltage / 12, -1, 1));

        // joint1Motor.set(arm1VoltageCorrection);
        // joint2Motor.set(arm2VoltageCorrection);
        // gripperMotor.set(wristVoltageCorrection);

        // joint1Motor.set(MathUtils.ensureRange(ffVoltages[0] / 12, -1, 1));
        // joint2Motor.set(MathUtils.ensureRange(ffVoltages[1] / 12, -1, 1));
        // gripperMotor.set(MathUtils.ensureRange(gripperVoltage / 12, -1, 1));

        // Logger.log("/ArmSubsystem/arm1Voltage", ffVoltages[0]);
        // Logger.log("/ArmSubsystem/arm2Voltage", ffVoltages[1]);
        // Logger.log("/ArmSubsystem/gripperVoltage", gripperVoltage);
        // Logger.log("/ArmSubsystem/arm1AngleSetpoint", motor1Controller.getSetpoint().position);
        // Logger.log("/ArmSubsystem/arm2AngleSetpoint", motor2Controller.getSetpoint().position);
        // Logger.log("/ArmSubsystem/gripperPositionSetpoint", gripperMotorController.getSetpoint().position);
        // Logger.log("/ArmSubsystem/arm1SpeedSetpoint", motor1Controller.getSetpoint().velocity);
        // Logger.log("/ArmSubsystem/arm2SpeedSetpoint", motor2Controller.getSetpoint().velocity);
        // Logger.log("/ArmSubsystem/gripperSpeedSetpoint", gripperMotorController.getSetpoint().velocity);
    }

    @Override
    public void simulationPeriodic() {
        if (brakingActivated == true) return;

        Matrix<N2, N1> angles = VecBuilder.fill(arm1Angle.getRadians(), arm2Angle.getRadians());
        Matrix<N2, N1> speeds = VecBuilder.fill(arm1Speed, arm2Speed);
        Matrix<N2, N1> voltages = VecBuilder.fill(joint1Motor.get() * 12, joint2Motor.get() * 12);
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
        PlacementLocation targetLocation = FieldConstants.getNearestPlacementLocation(robotPoseSupplier.get());

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
                targetPose3d.minus(new Pose3d(robotPoseSupplier.get())).getTranslation();

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

    public Command armSequence(ArmState... armStates) {
        return Commands.sequence(Stream.of(armStates)
                .map((state) -> armStateCommand(state))
                .toList()
                .toArray(new Command[armStates.length]));
    }

    public ArmState getState() {
        return armState;
    }

    public void setPickup() {
        setState(ArmState.PICKUP);
    }

    public void setAwaitingPiece() {
        setState(ArmState.AWAITING_PIECE);
    }

    public void setAwaitingDeployment() {
        setState(ArmState.AWAITING_DEPLOYMENT);
    }

    public void setHybrid() {
        setState(ArmState.HYBRID);
    }

    public void setMid() {
        setState(ArmState.MID);
    }

    public void setHigh() {
        setState(ArmState.HIGH);
    }

    public void setHybridManual() {
        setState(ArmState.HYBRID_MANUAL);
    }

    public void setMidManual() {
        setState(ArmState.MID_MANUAL);
    }

    public void setHighManual() {
        setState(ArmState.HIGH_MANUAL);
    }

    public void setDance() {
        setState(ArmState.DANCE);
    }

    public void setNetworkTablesMode() {
        setState(ArmState.NETWORK_TABLES_AIM);
    }

    public void setPassthrough() {
        setState(ArmState.PASSTHROUGH);
    }

    public void setBrake() {
        setState(ArmState.BRAKE);
    }

    public void setCoast() {
        setState(ArmState.COAST);
    }

    public enum ArmState {
        // PICKUP(new Static(0.7, 0, Rotation2d.fromDegrees(-50))),
        PICKUP(new Static(0.9, -0.1, Rotation2d.fromDegrees(-5))),
        AWAITING_PIECE(new Static(0.24, 0.27, new Rotation2d())),
        AWAITING_DEPLOYMENT(new Static(0.34, 0.27, new Rotation2d())),
        // AWAITING_DEPLOYMENT(new Static(0.4, 0.12, Rotation2d.fromDegrees(-60))),
        HYBRID_MANUAL(new Static(0.97, -0.04, Rotation2d.fromDegrees(-5))),
        // HYBRID_MANUAL(Static.fromBumper(FieldConstants.lowX + 0.5, 0.1, Rotation2d.fromDegrees(20))),
        // MID_MANUAL(Static.fromBumper(
        //         FieldConstants.midX,
        //         FieldConstants.midConeZ + ArmConstants.placementHeightOffset,
        //         Rotation2d.fromDegrees(20))),
        MID_MANUAL(Static.fromBumper(FieldConstants.midX - 0.1, FieldConstants.highConeZ, Rotation2d.fromDegrees(30))),
        HIGH_MANUAL(Static.fromBumper(
                FieldConstants.highX + 0.25,
                FieldConstants.highConeZ + ArmConstants.placementHeightOffset + 0.3,
                Rotation2d.fromDegrees(60))),
        HYBRID(new Dynamic(sus -> sus.getDynamicArmPosition(), new Rotation2d())), // this is my
        MID(new Dynamic(sussy -> sussy.getDynamicArmPosition(), new Rotation2d())), // subsystem, i can
        HIGH(new Dynamic(sussier -> sussier.getDynamicArmPosition(), new Rotation2d())), // name my variables
        DANCE(new Dynamic(
                sussiest -> new Translation2d(Math.cos(Timer.getFPGATimestamp()), .2),
                new Rotation2d())), // what i want
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
