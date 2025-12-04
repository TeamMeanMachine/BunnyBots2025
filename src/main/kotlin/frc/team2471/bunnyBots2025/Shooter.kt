package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Relay
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits
import org.team2471.frc.lib.ctre.d
import org.team2471.frc.lib.ctre.inverted
import org.team2471.frc.lib.ctre.p
import org.team2471.frc.lib.ctre.s
import org.team2471.frc.lib.ctre.v


object Shooter: SubsystemBase("Shooter") {
    val table = NetworkTableInstance.getDefault().getTable("Shooter")

    val shootingVelocityEntry = table.getEntry("Shooting Velocity")

    val shootingVelocity get() = shootingVelocityEntry.getDouble(26.0)

    val shooterMotorRight = TalonFX(Falcons.SHOOTER_RIGHT, CANivores.TURRET_CAN)
    val shooterMotorLeft = TalonFX(Falcons.SHOOTER_LEFT, CANivores.TURRET_CAN)

    var ramping = false

    @get:AutoLogOutput(key = "Shooter/Shooter Motor Right")
    val motorRpmRight
        get() = shooterMotorRight.velocity.valueAsDouble

    @get:AutoLogOutput(key = "Shooter/Shooter Motor Left")
    val motorRpmLeft
        get() = shooterMotorLeft.velocity.valueAsDouble

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

            p(0.25)
            d(0.005)
            v(0.12)
            s(0.17, StaticFeedforwardSignValue.UseVelocitySign)
        }
        shooterMotorLeft.applyConfiguration {
            currentLimits(25.0,40.0,1.0)
            inverted(false)
            coastMode()

            p(0.25)
            d(0.005)
            v(0.12)
            s(0.17, StaticFeedforwardSignValue.UseVelocitySign)
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
}