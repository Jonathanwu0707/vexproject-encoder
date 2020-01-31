/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  public static Joystick joy1 = new Joystick(0);
  private final Talon motor1=new Talon(0);
  private final Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);
  double finalspeed = 0; //pwm
  double lastError;
  double lastTimestamp;
  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
    encoder.reset();
    lastError=0;
    lastTimestamp=Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    final double kp=0.3;
   double error=(finalspeed-encoder.getRate();
    
   //final double ki=0.5;
    double dt=Timer.getFPGATimestamp()-lastTimestamp;
    
    final double kd=0.1;
    double derror=(error-lastError)/dt;


    final double output=kp*error + kd*derror;//ki*errorsum
    SmartDashboard.putNumber("encoder rate",encoder. getRate());
    SmartDashboard.putNumber("derror rate",derror);
    
    if(joy1.getRawButton(1)){
      finalspeed=50;//pwm
    }
    else if(joy1.getRawButton(2)){
      finalspeed=0;
    }

     motor1.set(output);
     lastError=error;
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testPeriodic() {
  }
}
