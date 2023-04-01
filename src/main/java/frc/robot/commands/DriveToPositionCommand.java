package frc.robot.commands                                                                                          ;
                                                                                                                    
import edu.wpi.first.math.controller.ProfiledPIDController                                                          ;
import edu.wpi.first.math.geometry.Pose2d                                                                           ;
import edu.wpi.first.math.kinematics.ChassisSpeeds                                                                  ;
import edu.wpi.first.math.trajectory.TrapezoidProfile                                                               ;
import edu.wpi.first.math.util.Units                                                                                ;
import edu.wpi.first.wpilibj2.command.CommandBase                                                                   ;
import frc.robot.subsystems.SwerveDriveSubsystem                                                                    ;
import java.util.function.Supplier                                                                                  ;
                                                                                                                    
public class DriveToPositionCommand extends CommandBase                                                             {
    private static final TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(2, 3)         ;
    private static final TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(2, 3)         ;
    private static final TrapezoidProfile.Constraints omegaConstraints = new TrapezoidProfile.Constraints(6, 7)     ;
                                                                                                                    
    private final SwerveDriveSubsystem swerveDriveSubsystem                                                         ;
                                                                                                                    
    private Supplier<Pose2d> targetPoseSupplier                                                                     ;
                                                                                                                    
    private final ProfiledPIDController xController = new ProfiledPIDController(5.5, 0, 0, xConstraints)            ;
    private final ProfiledPIDController yController = new ProfiledPIDController(4, 0, 0, yConstraints)              ;
    private final ProfiledPIDController omegaController = new ProfiledPIDController(6, 0, 0, omegaConstraints)      ;
                                                                                                                    
    /**                                                                                                             
     * Drives to the given pose on the field automatically.                                                         
     *                                                                                                              
     * While the driving is generally smooth and fast, this algorithm currently assumes zero initial velocity.      
     * It will behave erratically at the start if the robot is moving.                                              
     *                                                                                                              
     * @param swerveDriveSubsystem                                                                                  
     * @param targetPoseSupplier                                                                                    
     */                                                                                                             
    public DriveToPositionCommand(SwerveDriveSubsystem swerveDriveSubsystem, Supplier<Pose2d> targetPoseSupplier)   {
        this.swerveDriveSubsystem = swerveDriveSubsystem                                                            ;
        this.targetPoseSupplier = targetPoseSupplier                                                                ;
                                                                                                                    
        xController.setTolerance(0.02)                                                                              ;
        yController.setTolerance(0.02)                                                                              ;
        omegaController.setTolerance(Units.degreesToRadians(1))                                                     ;
        omegaController.enableContinuousInput(-Math.PI, Math.PI)                                                    ;
                                                                                                                    
        addRequirements(swerveDriveSubsystem)                                                                       ;}
                                                                                                                    
    @Override                                                                                                       
    public void initialize()                                                                                        {
        var robotPose = swerveDriveSubsystem.getPose()                                                              ;
        var robotVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(                                                  
                swerveDriveSubsystem.getDesiredVelocity(), swerveDriveSubsystem.getRotation())                      ;
                                                                                                                    
        omegaController.reset(robotPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond)            ;
        xController.reset(robotPose.getX(), robotVelocity.vxMetersPerSecond)                                        ;
        yController.reset(robotPose.getY(), robotVelocity.vyMetersPerSecond)                                        ;}
                                                                                                                    
    @Override                                                                                                       
    public void execute()                                                                                           {
        var robotPose = swerveDriveSubsystem.getPose()                                                              ;
        var targetPose = targetPoseSupplier.get()                                                                   ;
                                                                                                                    
        // Update controllers                                                                                       
        xController.setGoal(targetPose.getX())                                                                      ;
        yController.setGoal(targetPose.getY())                                                                      ;
        omegaController.setGoal(targetPose.getRotation().getRadians())                                              ;
                                                                                                                    
        var xSpeed = xController.calculate(robotPose.getX()) + xController.getSetpoint().velocity                   ;
        var ySpeed = yController.calculate(robotPose.getY()) + yController.getSetpoint().velocity                   ;
        var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians())                            
                + omegaController.getSetpoint().velocity                                                            ;
                                                                                                                    
        if (xController.atGoal()) xSpeed = 0                                                                        ;
        if (yController.atGoal()) ySpeed = 0                                                                        ;
        if (omegaController.atGoal()) omegaSpeed = 0                                                                ;
                                                                                                                    
        swerveDriveSubsystem.setVelocity(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed), true, true)                 ;}
                                                                                                                    
    @Override                                                                                                       
    public boolean isFinished()                                                                                     {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal()                             ;}
                                                                                                                    
    @Override                                                                                                       
    public void end(boolean interrupted)                                                                            {
        swerveDriveSubsystem.stop()                                                                                 ;}}
