package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.ctre.coastMode
import org.team2471.frc.lib.ctre.currentLimits

object Intake: SubsystemBase("Intake") {
    private val table = NetworkTableInstance.getDefault().getTable("Intake")
    val deployMotor = TalonFX(Falcons.INTAKE_DEPLOY)
    val frontMotor = TalonFX(Falcons.INTAKE_FRONT)

    const val DEPLOY_POSE = 1.0
    const val HOME_POSE = 0.0
    const val INTAKE_POWER = 0.5

    init {
        deployMotor.applyConfiguration {
            currentLimits(20.0,30.0,1.0)
            coastMode()
        }
        frontMotor.applyConfiguration {
            currentLimits(10.0,20.0,1.0)
            coastMode()
        }
    }
    fun deploy() {
        println()
    }
    fun home() {

    }
}