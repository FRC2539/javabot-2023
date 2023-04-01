package frc.lib.swerve                                                                                                      ;
                                                                                                                            
import com.ctre.phoenix.motorcontrol.ControlMode                                                                            ;
import com.ctre.phoenix.motorcontrol.DemandType                                                                             ;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX                                                                        ;
import com.ctre.phoenix.sensors.WPI_CANCoder                                                                                ;
import edu.wpi.first.math.controller.SimpleMotorFeedforward                                                                 ;
import edu.wpi.first.math.geometry.Rotation2d                                                                               ;
import edu.wpi.first.math.kinematics.SwerveModulePosition                                                                   ;
import edu.wpi.first.math.kinematics.SwerveModuleState                                                                      ;
import frc.lib.math.Conversions                                                                                             ;
import frc.robot.Constants                                                                                                  ;
import frc.robot.Robot                                                                                                      ;
                                                                                                                            
public class SwerveModule                                                                                                   {
    public int moduleNumber                                                                                                 ;
    private double angleOffset                                                                                              ;
    private WPI_TalonFX angleMotor                                                                                          ;
    private WPI_TalonFX driveMotor                                                                                          ;
    private WPI_CANCoder angleEncoder                                                                                       ;
    private double lastAngle                                                                                                ;
                                                                                                                            
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(                                                   
            Constants.SwerveConstants.calculatedDriveKS,                                                                    
            Constants.SwerveConstants.calculatedDriveKV,                                                                    
            Constants.SwerveConstants.calculatedDriveKA)                                                                    ;
                                                                                                                            
    // Testing a calculation method                                                                                         
    // SimpleMotorFeedforward angleFeedforward = new SimpleMotorFeedforward(                                                
    //         Constants.SwerveConstants.angleKS, Constants.SwerveConstants.angleKV, Constants.SwerveConstants.angleKA)     ;
                                                                                                                            
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants)                                            {
        this.moduleNumber = moduleNumber                                                                                    ;
        angleOffset = moduleConstants.angleOffset                                                                           ;
                                                                                                                            
        /* Angle Encoder Config */                                                                                          
        angleEncoder = moduleConstants.canivoreName.isEmpty()                                                               
                ? new WPI_CANCoder(moduleConstants.cancoderID)                                                              
                : new WPI_CANCoder(moduleConstants.cancoderID, moduleConstants.canivoreName.get())                          ;
        configAngleEncoder()                                                                                                ;
                                                                                                                            
        /* Angle Motor Config */                                                                                            
        angleMotor = moduleConstants.canivoreName.isEmpty()                                                                 
                ? new WPI_TalonFX(moduleConstants.angleMotorID)                                                             
                : new WPI_TalonFX(moduleConstants.angleMotorID, moduleConstants.canivoreName.get())                         ;
        configAngleMotor()                                                                                                  ;
                                                                                                                            
        /* Drive Motor Config */                                                                                            
        driveMotor = moduleConstants.canivoreName.isEmpty()                                                                 
                ? new WPI_TalonFX(moduleConstants.driveMotorID)                                                             
                : new WPI_TalonFX(moduleConstants.driveMotorID, moduleConstants.canivoreName.get())                         ;
        configDriveMotor()                                                                                                  ;
                                                                                                                            
        lastAngle = getState().angle.getDegrees()                                                                           ;}
                                                                                                                            
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)                                         {
        setDesiredState(desiredState, isOpenLoop, false)                                                                    ;}
                                                                                                                            
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean isSecondOrder)                  {
        // Custom optimize command, since default WPILib optimize assumes continuous controller, which CTRE is not          
        desiredState = CTREModuleState.optimize(desiredState, getState().angle)                                             ;
                                                                                                                            
        if (isOpenLoop)                                                                                                     {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed                   ;
            driveMotor.set(ControlMode.PercentOutput, percentOutput)                                                        ;}
        else                                                                                                                {
            double velocity = Conversions.MPSToFalcon(                                                                      
                    desiredState.speedMetersPerSecond,                                                                      
                    Constants.SwerveConstants.wheelCircumference,                                                           
                    Constants.SwerveConstants.driveGearRatio)                                                               ;
            driveMotor.set(                                                                                                 
                    ControlMode.Velocity,                                                                                   
                    velocity,                                                                                               
                    DemandType.ArbitraryFeedForward,                                                                        
                    driveFeedforward.calculate(desiredState.speedMetersPerSecond))                                          ;}
                                                                                                                            
        // Determine the angle to set the module to                                                                         
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))         
                ? lastAngle                                                                                                 
                : desiredState.angle                                                                                        
                        .getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents Jittering.             
                                                                                                                            
        // Account for the velocity of the angle motor if in second order mode                                              
        if (isSecondOrder && desiredState instanceof SecondOrderSwerveModuleState)                                          {
            angleMotor.set(                                                                                                 
                    ControlMode.Position,                                                                                   
                    Conversions.degreesToFalcon(0, Constants.SwerveConstants.angleGearRatio),                               
                    DemandType.ArbitraryFeedForward,                                                                        
                    ((SecondOrderSwerveModuleState) desiredState).angularVelocityRadiansPerSecond                           
                            * Constants.SwerveConstants.calculatedAngleKV)                                                  ;}
        else                                                                                                                {
            angleMotor.set(                                                                                                 
                    ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveConstants.angleGearRatio))     ;}
                                                                                                                            
        lastAngle = angle                                                                                                   ;}
                                                                                                                            
    public void setDesiredAngleOnly(Rotation2d desiredAngle, boolean optimized)                                             {
        // Set the module to face forwards                                                                                  
        if (optimized)                                                                                                      {
            desiredAngle = CTREModuleState.optimize(new SwerveModuleState(1, desiredAngle), getState().angle).angle         ;}
                                                                                                                            
        angleMotor.set(                                                                                                     
                ControlMode.Position,                                                                                       
                Conversions.degreesToFalcon(desiredAngle.getDegrees(), Constants.SwerveConstants.angleGearRatio))           ;
                                                                                                                            
        lastAngle = 0                                                                                                       ;
                                                                                                                            
        // Stop the motor to bypass the speed check                                                                         
        driveMotor.stopMotor()                                                                                              ;}
                                                                                                                            
    public void setDriveCharacterizationVoltage(double voltage)                                                             {
        // Set the module to face forwards                                                                                  
        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(0, Constants.SwerveConstants.angleGearRatio))      ;
                                                                                                                            
        lastAngle = 0                                                                                                       ;
                                                                                                                            
        // Set the drive motor to the specified voltage                                                                     
        driveMotor.set(ControlMode.PercentOutput, voltage / Constants.GlobalConstants.targetVoltage)                        ;}
                                                                                                                            
    public void setAngleCharacterizationVoltage(double voltage)                                                             {
        // Set the module to face forwards                                                                                  
        angleMotor.set(ControlMode.PercentOutput, voltage / Constants.GlobalConstants.targetVoltage)                        ;
                                                                                                                            
        lastAngle = 0                                                                                                       ;
                                                                                                                            
        // Set the drive motor to just enough to overcome static friction                                                   
        driveMotor.set(ControlMode.PercentOutput, 1.1 * Constants.SwerveConstants.driveKS)                                  ;}
                                                                                                                            
    public void disableMotors()                                                                                             {
        driveMotor.stopMotor()                                                                                              ;
        angleMotor.stopMotor()                                                                                              ;}
                                                                                                                            
    public void resetToAbsolute()                                                                                           {
        double absolutePosition = Conversions.degreesToFalcon(                                                              
                getCanCoder().getDegrees() - angleOffset, Constants.SwerveConstants.angleGearRatio)                         ;
        angleMotor.setSelectedSensorPosition(absolutePosition)                                                              ;}
                                                                                                                            
    private void configAngleEncoder()                                                                                       {
        angleEncoder.configFactoryDefault()                                                                                 ;
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig)                                              ;}
                                                                                                                            
    private void configAngleMotor()                                                                                         {
        angleMotor.configFactoryDefault()                                                                                   ;
        angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig)                                                 ;
        angleMotor.setInverted(Constants.SwerveConstants.angleMotorInvert)                                                  ;
        angleMotor.setNeutralMode(Constants.SwerveConstants.angleNeutralMode)                                               ;
        angleMotor.enableVoltageCompensation(true)                                                                          ;
        resetToAbsolute()                                                                                                   ;}
                                                                                                                            
    private void configDriveMotor()                                                                                         {
        driveMotor.configFactoryDefault()                                                                                   ;
        driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig)                                                 ;
        driveMotor.setNeutralMode(Constants.SwerveConstants.driveNeutralMode)                                               ;
        driveMotor.setSelectedSensorPosition(0)                                                                             ;
        driveMotor.enableVoltageCompensation(true)                                                                          ;
        driveMotor.setSensorPhase(Constants.SwerveConstants.driveEncoderInvert)                                             ;
        driveMotor.setInverted(Constants.SwerveConstants.driveMotorInvert)                                                  ;}
                                                                                                                            
    public Rotation2d getCanCoder()                                                                                         {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition())                                                   ;}
                                                                                                                            
    public WPI_TalonFX getDriveMotor()                                                                                      {
        return driveMotor                                                                                                   ;}
                                                                                                                            
    public WPI_TalonFX getAngleMotor()                                                                                      {
        return angleMotor                                                                                                   ;}
                                                                                                                            
    public SwerveModuleState getState()                                                                                     {
        double velocity = Conversions.falconToMPS(                                                                          
                driveMotor.getSelectedSensorVelocity(),                                                                     
                Constants.SwerveConstants.wheelCircumference,                                                               
                Constants.SwerveConstants.driveGearRatio)                                                                   ;
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(                                              
                angleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio))                          ;
        return new SwerveModuleState(velocity, angle)                                                                       ;}
                                                                                                                            
    public double getAngularVelocity()                                                                                      {
        return Conversions.falconToRPM(angleMotor.getSelectedSensorVelocity(), Constants.SwerveConstants.angleGearRatio)    
                / 60                                                                                                        
                * 2                                                                                                         
                * Math.PI                                                                                                   ;}
                                                                                                                            
    public SwerveModulePosition getPosition()                                                                               {
        double encoder = Conversions.falconToMPS(                                                                           
                        driveMotor.getSelectedSensorPosition(),                                                             
                        Constants.SwerveConstants.wheelCircumference,                                                       
                        Constants.SwerveConstants.driveGearRatio)                                                           
                / 10.0; // Compensate for Talon measuring in 100 ms units                                                   
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(                                              
                angleMotor.getSelectedSensorPosition(), Constants.SwerveConstants.angleGearRatio))                          ;
        return new SwerveModulePosition(encoder, angle)                                                                     ;}
                                                                                                                            
    public double getDriveTemperature()                                                                                     {
        return driveMotor.getTemperature()                                                                                  ;}
                                                                                                                            
    public double getAngleTemperature()                                                                                     {
        return angleMotor.getTemperature()                                                                                  ;}
                                                                                                                            
    public double getDriveVoltage()                                                                                         {
        return driveMotor.getMotorOutputVoltage()                                                                           ;}
                                                                                                                            
    public double getAngleVoltage()                                                                                         {
        return angleMotor.getMotorOutputVoltage()                                                                           ;}
                                                                                                                            
    public double getDriveCurrent()                                                                                         {
        return driveMotor.getSupplyCurrent()                                                                                ;}
                                                                                                                            
    public double getAngleCurrent()                                                                                         {
        return angleMotor.getSupplyCurrent()                                                                                ;}}
