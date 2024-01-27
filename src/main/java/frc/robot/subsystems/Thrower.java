package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Thrower {
    static double targetRPM;
    static double throwerSpeed;
    static int delay;
    static double P = 0.000008;
    static double I = -0.0003;
    static double ix, error = 0.0;
    static int targetReachedCount = 0;
    static double intakeSpeed = 1.0;

    public boolean throwerRPM(int targetRPM) {
        boolean targetReached = false;
        boolean usePidLoop = true;

        // Range check the RPM
        if (targetRPM > 20800) {
            targetRPM = 20800;
        }
        if (targetRPM < 0) {
            targetRPM = 0;
        }

        int rpm = (int) Math.abs(Robot.throwerMotor2.getSelectedSensorVelocity());

        if (usePidLoop == true) {
            /**********************************************************************
             * PID Loop for controlling motor RPM
             **********************************************************************/

            if (targetRPM == 0) { /** Spindown **/
                throwerSpeed = 0;
            } else { /** Normal operation **/
                SmartDashboard.putNumber("ix", ix);

                error = targetRPM - rpm;
                ix = error * 0.02; /** Loop frequency **/
                throwerSpeed += P * error + I * ix;

                SmartDashboard.putNumber("err0r", error);
                SmartDashboard.putNumber("ix", ix);
                SmartDashboard.putNumber("throwerSpeed", throwerSpeed);
            }

            if (throwerSpeed < 0) {
                throwerSpeed = 0;
            } else if (throwerSpeed > 1) {
                throwerSpeed = 1;
            }

            if (targetRPM < rpm + 100 && targetRPM > rpm - 100) {
                targetReached = true;
            }
        } else {
            /**********************************************************************
             * Manual Motor Control
             **********************************************************************/
            if (rpm < targetRPM - 75) {
                // If we are below the rpm target
                if (delay <= 0) {
                    if (rpm < targetRPM - 500) {
                        // If we are more than 500 RPM away, change speed faster
                        if (rpm < targetRPM - 1500) {
                            // If we are more than 1500 RPM away, change speed even faster
                            delay = 1;
                        } else {
                            delay = 2;
                        }
                        throwerSpeed = throwerSpeed + 0.01;
                    } else {
                        // if we less than 500 RPM awawy, change speed slower
                        delay = 2;
                        throwerSpeed = throwerSpeed + 0.002;
                    }
                    if (throwerSpeed > 1) {
                        throwerSpeed = 1;
                    }
                }
            } else if (rpm > targetRPM + 75) {
                // If we are above the rpm target
                if (delay <= 0) {
                    if (rpm > targetRPM + 500) {
                        // If we are more than 500 RPM away, change speed faster
                        if (rpm > targetRPM + 1500) {
                            // If we are more than 1500 RPM away, change speed even faster
                            delay = 1;
                        } else {
                            delay = 2;
                        }
                        throwerSpeed = throwerSpeed - 0.01;
                    } else {
                        // if we less than 500 RPM awawy, change speed slower
                        delay = 2;
                        throwerSpeed = throwerSpeed - 0.002;
                    }
                    if (throwerSpeed < 0) {
                        throwerSpeed = 0;
                    }
                }
            } else {
                targetReached = true;
            }

            if (targetRPM == 0) {
                // Short cut to stop the thrower motors
                throwerSpeed = 0;
            }

            delay--;
        }

        // Set the speed on the Thrower Motors
        Robot.throwerMotor1.set(ControlMode.PercentOutput, throwerSpeed * RobotMap.throwerMotor1Inversion);
        Robot.throwerMotor2.set(ControlMode.PercentOutput, throwerSpeed * RobotMap.throwerMotor2Inversion);

        // Log info to the smart dashboard
        SmartDashboard.putNumber("Throw RPM Current", rpm);
        SmartDashboard.putNumber("Throw RPM Target", targetRPM);
        SmartDashboard.putBoolean("Throw RPM Reached", targetReached);

        return (targetReached);
    }
}
