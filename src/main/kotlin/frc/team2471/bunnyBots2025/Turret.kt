package frc.team2471.bunnyBots2025

import com.ctre.phoenix6.hardware.CANdi
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team2471.frc.lib.ctre.applyConfiguration
import org.team2471.frc.lib.units.asDegrees
import org.team2471.frc.lib.units.asRadians
import org.team2471.frc.lib.units.degrees
import org.team2471.frc.lib.units.rotations
import org.team2471.frc.lib.units.wrap
import kotlin.math.IEEErem
import kotlin.math.abs
import kotlin.math.sin

object Turret: SubsystemBase("Turret") {
    private val table = NetworkTableInstance.getDefault().getTable("Turret")
    private val pivotEncoderOffsetEntry = table.getEntry("Turret Encoder Offset")

    val turretMotor = TalonFX(Falcons.TURRET_0)
    val candi = CANdi(CANSensors.CANDI)


    val rawLampreyAngle: Angle
        get() = candi.pwM1Position.valueAsDouble.IEEErem(1.0).rotations.wrap()

    val defaultTurretEncoderOffset = 0.0
    var lampreyEncoderOffset: Double = pivotEncoderOffsetEntry.getDouble(defaultTurretEncoderOffset)

    val unCorrectedLampreyAngle: Angle
        get() = (rawLampreyAngle - lampreyEncoderOffset.degrees)

    val lampreyAlignmentOffset: Angle
        get() = abs((sin(unCorrectedLampreyAngle.asRadians) * 0.0)).degrees

    val turretEncoderAngle: Angle
        get() = (unCorrectedLampreyAngle - lampreyAlignmentOffset)

    val turretMotorAngle: Angle
        get() = turretMotor.position.valueAsDouble.rotations


    init {
//        turretMotor.applyConfiguration {
//
//        }
    }

    override fun periodic() {
        Logger.recordOutput("Turret/lampreyAlignmentOffset", lampreyAlignmentOffset.asDegrees)
        Logger.recordOutput("Turret/rawLampreyAngle", rawLampreyAngle.asDegrees)
        Logger.recordOutput("Turret/unCorrectedLampreyAngle", unCorrectedLampreyAngle.asDegrees)
        Logger.recordOutput("Turret/turretEncoderAngle", turretEncoderAngle.asDegrees)

    }

}