package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Relay
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.ctre.a
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.ctre.v
import org.team2471.frc.lib.units.asVolts
import org.team2471.frc.lib.units.rotationsPerSecond
import org.team2471.frc.lib.units.seconds
import org.team2471.frc.lib.units.volts
import org.team2471.frc.lib.units.voltsPerSecond


object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    val shootingVelocityEntry = table.getEntry("Shooting Velocity")

    val shootingVelocity get() = shootingVelocityEntry.getDouble(26.0)

    val shooterMotorRight = TalonFX(Falcons.SHOOTER_RIGHT, CANivores.TURRET_CAN)
    val shooterMotorLeft = TalonFX(Falcons.SHOOTER_LEFT, CANivores.TURRET_CAN)

    var ramping = false

    @get:AutoLogOutput(key = "Shooter/Shooter Motor Right")
    val motorSpeedRight
        get() = shooterMotorRight.velocity.valueAsDouble.rotationsPerSecond

    @get:AutoLogOutput(key = "Shooter/Shooter Motor Left")
    val motorSpeedLeft
        get() = shooterMotorLeft.velocity.valueAsDouble.rotationsPerSecond

    @get:AutoLogOutput
    var rightRpmSetpoint = 0.0
        set(value) {
            shooterMotorRight.setControl(VelocityVoltage(value))
            field =value
        }

    @get:AutoLogOutput
    var leftRpmSetpoint = 0.0
        set(value) {
            shooterMotorLeft.setControl(VelocityVoltage(value))
            field =value
        }

    @get:AutoLogOutput
    var shooterSetpoint = 0.0
        set(value) {
            rightRpmSetpoint = value
            leftRpmSetpoint = value
            field =value
        }


    init {

        if (!shootingVelocityEntry.exists()) shootingVelocityEntry.setDouble(26.0)

        shooterMotorRight.applyConfiguration {
            currentLimits(25.0,40.0,1.0)
            inverted(true)
            coastMode()

            p(0.1746)
            d(0.0)
            a(0.0413)
            v(0.117)
            s(0.177, StaticFeedforwardSignValue.UseVelocitySign)
        }
        shooterMotorLeft.applyConfiguration {
            currentLimits(25.0,40.0,1.0)
            inverted(false)
            coastMode()

            p(0.1748)
            d(0.0)
            a(0.0417)
            v(0.114)
            s(0.09, StaticFeedforwardSignValue.UseVelocitySign)
        }
    }

    fun shoot() {
        shooterSetpoint = shootingVelocity
        ramping = true
    }

    fun stop() {
        shooterSetpoint = 0.0
        ramping = false
    }


    val sysIDLeftRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            1.0.voltsPerSecond,
            7.0.volts,
            5.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("SysIdShooterLeft_State", state.toString())
            Logger.recordOutput("Shooter_Left_Position", shooterMotorLeft.position.valueAsDouble)
            Logger.recordOutput("Shooter_Left_Velocity", shooterMotorLeft.velocity.valueAsDouble)
        },
        SysIdRoutine.Mechanism({ output: Voltage ->
            shooterMotorLeft.setControl(VoltageOut(output.asVolts))
            /* also log the requested output for SysId */
            Logger.recordOutput("Shooter_Left_Voltage", output.asVolts + 0.0001 * Math.random())
        }, null, this)
    )

    val sysIDRightRoutine = SysIdRoutine(
        SysIdRoutine.Config(
            1.0.voltsPerSecond,
            7.0.volts,
            12.0.seconds
        ) { state: SysIdRoutineLog.State ->
            SignalLogger.writeString("SysIdShooterRight_State", state.toString())
            Logger.recordOutput("SysIdShooterRight_State", state.toString())
            Logger.recordOutput("Shooter_Right_Position", shooterMotorRight.position.valueAsDouble)
            Logger.recordOutput("Shooter_Right_Velocity", shooterMotorRight.velocity.valueAsDouble)
        },
        SysIdRoutine.Mechanism({ output: Voltage ->
            shooterMotorRight.setControl(VoltageOut(output.asVolts))
            /* also log the requested output for SysId */
            Logger.recordOutput("Shooter_Right_Voltage", output.asVolts + 0.0001 * Math.random())
        }, null, this)
    )
}