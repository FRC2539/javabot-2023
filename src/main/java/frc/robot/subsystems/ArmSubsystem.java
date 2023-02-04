package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logging.LoggedReceiver;
import frc.lib.logging.Logger;
import frc.lib.math.Conversions;
import frc.lib.math.MathUtils;
import frc.lib.math.TwoJointedArmFeedforward;
import frc.lib.swerve.ProfiledPIDControllerPlus;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GripperConstants;
import frc.robot.Robot;
import java.util.function.Function;

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

    private double arm1Angle = 0;
    private double arm2Angle = 0;
    private double gripperAngle = 0;
    private double arm1Speed = 0;
    private double arm2Speed = 0;
    private double gripperSpeed = 0;

    LoggedReceiver desiredNetworkTablesArmPosition;

    private WPI_TalonFX joint1Motor;
    private WPI_TalonFX joint2Motor;
    private WPI_TalonSRX gripperMotor;
    private DutyCycleEncoder gripperEncoder;

    private boolean brakingActivated;

    private double lastGripperPosition;

    private ProfiledPIDController motor1Controller;
    private ProfiledPIDController motor2Controller;
    private ProfiledPIDController gripperMotorController;

    private static final TrapezoidProfile.Constraints motor1Constraints = new TrapezoidProfile.Constraints(12, 6);
    private static final TrapezoidProfile.Constraints motor2Constraints = new TrapezoidProfile.Constraints(12, 6);
    private static final TrapezoidProfile.Constraints wristProfileConstraints = new TrapezoidProfile.Constraints(12, 6);

    private Translation2d endEffector = new Translation2d();
    private Rotation2d gripperEndAngle = new Rotation2d();

    private ArmState armState = ArmState.NETWORK_TABLES_AIM;
    private boolean doCycleMode = false;
    private Timer cycleModeTimer = new Timer();
    private int cycleModeIndex = 0;

    private TwoJointedArmFeedforward simFeedforward;
    private TwoJointedArmFeedforward feedforward;
    private ArmFeedforward gripperJointFeedforward;

    public ArmSubsystem() {
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

        arm1Angle = ArmConstants.arm1StartingAngle.getRadians();
        arm2Angle = ArmConstants.arm2StartingAngle.getRadians();
        gripperAngle = GripperConstants.startingAngle.getRadians();

        joint1Motor = new WPI_TalonFX(ArmConstants.mastMotorPort);
        joint2Motor = new WPI_TalonFX(ArmConstants.boomMotorPort);
        gripperMotor = new WPI_TalonSRX(ArmConstants.wristMotorPort); // wrist motor
        gripperEncoder = new DutyCycleEncoder(0);

        joint1Motor.setSelectedSensorPosition(
                Conversions.radiansToFalcon(ArmConstants.arm1StartingAngle.getRadians(), ArmConstants.arm1GearRatio));
        joint2Motor.setSelectedSensorPosition(
                Conversions.radiansToFalcon(ArmConstants.arm2StartingAngle.getRadians(), ArmConstants.arm2GearRatio));
        gripperMotor.setSelectedSensorPosition(
                Conversions.radiansToFalcon(GripperConstants.startingAngle.getRadians(), GripperConstants.gearRatio));

        gripperEndAngle = GripperConstants.startingAngle;

        endEffector = forwardKinematics(
                ArmConstants.arm1Length,
                ArmConstants.arm1StartingAngle,
                ArmConstants.arm2Length,
                ArmConstants.arm2StartingAngle,
                GripperConstants.length,
                GripperConstants.startingAngle);

        SmartDashboard.putData("Arm Mechanism", mechanism);

        simFeedforward = new TwoJointedArmFeedforward(
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
        feedforward = new TwoJointedArmFeedforward(
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

        motor1Controller = new ProfiledPIDControllerPlus(10, 0, 0, motor1Constraints);
        motor2Controller = new ProfiledPIDControllerPlus(10, 0, 0, motor2Constraints);
        gripperMotorController = new ProfiledPIDControllerPlus(10, 0, 0, wristProfileConstraints);

        motor1Controller.reset(arm1Angle);
        motor2Controller.reset(arm2Angle);
        gripperMotorController.reset(gripperAngle);

        motor1Controller.setTolerance(ArmConstants.angularTolerance);
        motor2Controller.setTolerance(ArmConstants.angularTolerance);
        gripperMotorController.setTolerance(ArmConstants.angularTolerance);

        // motor1Controller.enableContinuousInput(-Math.PI, Math.PI);
        // motor2Controller.enableContinuousInput(-Math.PI, Math.PI);
        // wristMotorController.enableContinuousInput(-Math.PI, Math.PI);

        desiredNetworkTablesArmPosition = Logger.tunable("/ArmSubsystem/ArmPose", new double[] {1, 0, 0});

        cycleModeTimer.start();

        setState(armState);
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
        if (armState.getType() instanceof Brake) {
            stopMotors();
        } else {
            startMotors();
        }
    }

    private void stopMotors() {
        brakingActivated = true;
        joint1Motor.stopMotor();
        joint2Motor.stopMotor();
        gripperMotor.stopMotor();
    }

    private void startMotors() {
        brakingActivated = false;
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
    }

    private double getGripperEncoderAngle() {
        return gripperEncoder.getAbsolutePosition() * 2 * Math.PI - GripperConstants.encoderOffset;
    }

    private boolean isArmAtGoal() {
        return MathUtils.equalsWithinError(arm1Angle, joint1DesiredMotorPosition, ArmConstants.angularTolerance)
                && MathUtils.equalsWithinError(arm2Angle, joint2DesiredMotorPosition, ArmConstants.angularTolerance)
                && MathUtils.equalsWithinError(
                        gripperAngle, gripperDesiredMotorPosition, ArmConstants.angularTolerance);
    }

    public ArmState getState() {
        return armState;
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

    public void setDance() {
        setState(ArmState.DANCE);
    }

    public void setNetworkTablesMode() {
        setState(ArmState.NETWORK_TABLES_AIM);
    }

    public void setBrake() {
        setState(ArmState.BRAKE);
    }

    @Override
    public void periodic() {
        // Update internal model with real motor values
        if (Robot.isReal()) {
            arm1Angle =
                    Conversions.falconToRadians(joint1Motor.getSelectedSensorPosition(), ArmConstants.arm1GearRatio);
            arm2Angle =
                    Conversions.falconToRadians(joint2Motor.getSelectedSensorPosition(), ArmConstants.arm2GearRatio);
            lastGripperPosition = gripperAngle;
            gripperAngle = getGripperEncoderAngle();

            arm1Speed = Conversions.falconToRadPS(joint1Motor.getSelectedSensorVelocity(), ArmConstants.arm1GearRatio);
            arm2Speed = Conversions.falconToRadPS(joint2Motor.getSelectedSensorVelocity(), ArmConstants.arm2GearRatio);
            gripperSpeed = (gripperAngle - lastGripperPosition) / 0.02;
        }

        // Update Mechanism2d widget
        arm1.setAngle(Math.toDegrees(arm1Angle));
        arm2.setAngle(Math.toDegrees(arm2Angle));
        gripper.setAngle(Math.toDegrees(gripperAngle));

        if (armState.getType() instanceof Dynamic || armState.getType() instanceof NetworkTablesAim) {
            updateArmDesiredPosition();
        }

        if (isArmAtGoal() || armState.getType() instanceof Brake) {
            stopMotors();
        } else {
            startMotors();
        }

        if (!brakingActivated) {
            executePIDFeedforward();
        }

        if (doCycleMode && cycleModeTimer.advanceIfElapsed(3)) {
            setState(ArmState.values()[cycleModeIndex]);
            cycleModeIndex++;
            cycleModeIndex %= ArmState.values().length;
        }

        Logger.log("/ArmSubsystem/arm1Angle", arm1Angle);
        Logger.log("/ArmSubsystem/arm2Angle", arm2Angle);
        Logger.log("/ArmSubsystem/arm1Speed", arm1Speed);
        Logger.log("/ArmSubsystem/arm2Speed", arm2Speed);
        Logger.log("/ArmSubsystem/gripperAngle", gripperAngle);
        Logger.log("/ArmSubsystem/gripperSpeed", gripperSpeed);
        Logger.log("/ArmSubsystem/joint1DesiredPosition", joint1DesiredMotorPosition);
        Logger.log("/ArmSubsystem/joint2DesiredPosition", joint2DesiredMotorPosition);
        Logger.log("/ArmSubsystem/gripperDesiredPosition", gripperDesiredMotorPosition);
        Logger.log("/ArmSubsystem/isBraking", brakingActivated);
        Logger.log("/ArmSubsystem/isArmAtPosition", isArmAtGoal());
    }

    private void executePIDFeedforward() {
        motor1Controller.setGoal(joint1DesiredMotorPosition);
        motor2Controller.setGoal(joint2DesiredMotorPosition);
        gripperMotorController.setGoal(gripperDesiredMotorPosition);

        double arm1VoltageCorrection = motor1Controller.calculate(arm1Angle);
        double arm2VoltageCorrection = motor2Controller.calculate(arm2Angle);
        double wristVoltageCorrection = gripperMotorController.calculate(gripperAngle);

        double arm1DesiredSpeed = motor1Controller.getSetpoint().velocity + arm1VoltageCorrection;
        double arm2DesiredSpeed = motor2Controller.getSetpoint().velocity + arm2VoltageCorrection;
        double gripperDesiredSpeed = gripperMotorController.getSetpoint().velocity + wristVoltageCorrection;

        double[] voltages = feedforward.calculateFeedforwardVoltages(
                arm1Angle,
                arm2Angle,
                arm1DesiredSpeed,
                arm2DesiredSpeed,
                (arm1DesiredSpeed - arm1Speed) / 0.02,
                (arm2DesiredSpeed - arm2Speed) / 0.02);

        double gripperVoltage = gripperJointFeedforward.calculate(
                gripperAngle + arm1Angle + arm2Angle, gripperDesiredSpeed, (gripperDesiredSpeed - gripperSpeed) / 0.02);

        joint1Motor.set(MathUtils.ensureRange(voltages[0] / 12, -1, 1));
        joint2Motor.set(MathUtils.ensureRange(voltages[1] / 12, -1, 1));
        gripperMotor.set(MathUtils.ensureRange(gripperVoltage / 12, -1, 1));
    }

    @Override
    public void simulationPeriodic() {
        if (brakingActivated) return;

        Matrix<N2, N1> angles = VecBuilder.fill(arm1Angle, arm2Angle);
        Matrix<N2, N1> speeds = VecBuilder.fill(arm1Speed, arm2Speed);
        Matrix<N2, N1> voltages = VecBuilder.fill(joint1Motor.get() * 12, joint2Motor.get() * 12);
        Matrix<N2, N1> acceleration = simFeedforward
                .calculateArmInertiaMatrix(angles)
                .inv()
                .times((simFeedforward.calculateMotorTorqueMatrix().times(voltages))
                        .minus(simFeedforward
                                .calculateCoriolisMatrix(speeds, angles)
                                .times(speeds))
                        .minus(simFeedforward.calculateGravityMatrix(angles))
                        .minus(simFeedforward.calculateBackEmfMatrix().times(speeds)));
        angles = angles.plus(speeds.times(0.02)).plus(acceleration.times(.5 * 0.02 * 0.02));
        speeds = speeds.plus(acceleration.times(0.02)).times(1);

        arm1Angle = angles.get(0, 0);
        arm2Angle = angles.get(1, 0);
        gripperAngle += gripperMotorController.getSetpoint().velocity * 0.02;

        arm1Speed = speeds.get(0, 0);
        arm2Speed = speeds.get(1, 0);
        gripperSpeed = gripperMotorController.getSetpoint().velocity;
    }

    public Translation2d getDynamicArmPosition() {
        return new Translation2d(0.5, 0).rotateBy(new Rotation2d(Timer.getFPGATimestamp()));
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

    public enum ArmState {
        AWAITING_PIECE(new Static(0.20, 0.07, new Rotation2d())),
        AWAITING_DEPLOYMENT(new Static(0.22, 0.27, new Rotation2d())),
        // HYBRID(0.5, 0.05),
        // MID(0.7, 0.6),
        // HIGH(1.0, 1.1),
        HYBRID(new Dynamic(sus -> sus.getDynamicArmPosition(), new Rotation2d())),
        MID(new Dynamic(a -> a.getDynamicArmPosition(), new Rotation2d())),
        HIGH(new Dynamic(a -> a.getDynamicArmPosition(), new Rotation2d())),
        DANCE(new Dynamic(a -> new Translation2d(Math.cos(Timer.getFPGATimestamp()), .2), new Rotation2d())),
        NETWORK_TABLES_AIM(new NetworkTablesAim()),
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

    private static class Static {
        private Translation2d endEffector;
        private Rotation2d angle;

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
}
