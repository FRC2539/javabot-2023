package frc.robot.commands                                                                                      ;
                                                                                                                
import edu.wpi.first.math.controller.PIDController                                                              ;
import edu.wpi.first.math.controller.ProfiledPIDController                                                      ;
import edu.wpi.first.math.filter.LinearFilter                                                                   ;
import edu.wpi.first.math.kinematics.ChassisSpeeds                                                              ;
import edu.wpi.first.math.trajectory.TrapezoidProfile                                                           ;
import edu.wpi.first.wpilibj2.command.CommandBase                                                               ;
import frc.lib.logging.Logger                                                                                   ;
import frc.lib.math.MathUtils                                                                                   ;
import frc.robot.subsystems.SwerveDriveSubsystem                                                                ;
import frc.robot.subsystems.VisionSubsystem                                                                     ;
import frc.robot.subsystems.VisionSubsystem.LimelightMode                                                       ;
import java.util.function.DoubleSupplier                                                                        ;
                                                                                                                
public class AssistedLLAimCommand extends CommandBase                                                           {
    private DoubleSupplier forward                                                                              ;
    private DoubleSupplier strafe                                                                               ;
                                                                                                                
    private SwerveDriveSubsystem swerveDriveSubsystem                                                           ;
    private VisionSubsystem visionSubsystem                                                                     ;
                                                                                                                
    private LinearFilter txRollingAverage = LinearFilter.movingAverage(5)                                       ;
                                                                                                                
    private static final TrapezoidProfile.Constraints angleConstraints = new TrapezoidProfile.Constraints(4, 4) ;
    private static final double maxStrafeVelocity = 2                                                           ;
                                                                                                                
    private ProfiledPIDController angleController = new ProfiledPIDController(1.5, 0.0, 0.0, angleConstraints)  ;
    private PIDController strafeController = new PIDController(0.05, 0.0, 0.1)                                  ;
                                                                                                                
    public AssistedLLAimCommand(                                                                                
            SwerveDriveSubsystem swerveDriveSubsystem,                                                          
            VisionSubsystem visionSubsystem,                                                                    
            DoubleSupplier forward,                                                                             
            DoubleSupplier strafe,                                                                              
            DoubleSupplier rotate)                                                                              {
        this.swerveDriveSubsystem = swerveDriveSubsystem                                                        ;
        this.visionSubsystem = visionSubsystem                                                                  ;
                                                                                                                
        addRequirements(swerveDriveSubsystem, visionSubsystem)                                                  ;
                                                                                                                
        this.forward = forward                                                                                  ;
        this.strafe = strafe                                                                                    ;
                                                                                                                
        angleController.setGoal(0)                                                                              ;
        angleController.enableContinuousInput(-Math.PI, Math.PI)                                                ;
                                                                                                                
        strafeController.setSetpoint(0)                                                                         ;
        strafeController.setTolerance(1.3)                                                                      ;}
                                                                                                                
    @Override                                                                                                   
    public void initialize()                                                                                    {
        visionSubsystem.setBackLimelightMode(LimelightMode.RETROREFLECTIVEHIGH)                                 ;
                                                                                                                
        angleController.reset(swerveDriveSubsystem.getRotation().getRadians())                                  ;}
                                                                                                                
    @Override                                                                                                   
    public void execute()                                                                                       {
        double strafeingValue                                                                                   ;
        double rollingAverage = 0                                                                               ;
                                                                                                                
        if (visionSubsystem.hasBackRetroreflectiveAngles() && visionSubsystem.isBackLimelightAtPipeline())      {
            double lastTx = visionSubsystem.getBackRetroreflectiveAngles().get().tx()                           ;
                                                                                                                
            strafeingValue = -MathUtils.ensureRange(                                                            
                    strafeController.calculate(rollingAverage = txRollingAverage.calculate(lastTx)),            
                    -maxStrafeVelocity,                                                                         
                    maxStrafeVelocity)                                                                          ;
                                                                                                                
            if (strafeController.atSetpoint()) strafeingValue = 0                                               ;}
        else                                                                                                    {
            strafeingValue = strafe.getAsDouble()                                                               ;}
                                                                                                                
        double angularCorrection = angleController.calculate(                                                   
                swerveDriveSubsystem.getPose().getRotation().getRadians())                                      ;
        double angularSpeed = 0; // angleController.getSetpoint().velocity + angularCorrection                  ;
                                                                                                                
        swerveDriveSubsystem.setVelocity(                                                                       
                new ChassisSpeeds(forward.getAsDouble(), strafeingValue, angularSpeed), true, true)             ;
                                                                                                                
        Logger.log("/LLAimCommand/tx", rollingAverage)                                                          ;}}
