package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    Mechanism2d mechanism = new Mechanism2d(4, 4);
    MechanismRoot2d root = mechanism.getRoot("Arm", 2, 2);
    MechanismLigament2d bone1;
    MechanismLigament2d bone2;

    // Only used for current demo
    Translation2d centralEffector = new Translation2d(1.7, 1.25);

    Translation2d endEffector = new Translation2d(1.7, 1.25);

    public ArmSubsystem() {
        bone1 = root.append(new MechanismLigament2d("Bone 1", 1.2, 80));
        bone2 = bone1.append(new MechanismLigament2d("Bone 2", 1.2, -50));

        SmartDashboard.putData("Arm Mechanism", mechanism);
    }

    private void reachEndEffector() {
        bone2.setAngle(Units.radiansToDegrees(-Math.acos((Math.pow(endEffector.getX(), 2)
                        + Math.pow(endEffector.getY(), 2)
                        - Math.pow(bone1.getLength(), 2)
                        - Math.pow(bone2.getLength(), 2))
                / (2 * bone1.getLength() * bone2.getLength()))));

        bone1.setAngle(Units.radiansToDegrees(Math.atan2(endEffector.getY(), endEffector.getX())
                - Math.atan2(
                        bone2.getLength() * Math.sin(Units.degreesToRadians(bone2.getAngle())),
                        bone1.getLength() + bone2.getLength() * Math.cos(Units.degreesToRadians(bone2.getAngle())))));
    }

    @Override
    public void periodic() {
        endEffector = new Translation2d(
                0.5 * Math.cos(Timer.getFPGATimestamp()) + centralEffector.getX(),
                0.5 * Math.sin(Timer.getFPGATimestamp() + centralEffector.getY()));

        reachEndEffector();
    }
}
