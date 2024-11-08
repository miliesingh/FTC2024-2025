package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// This is far side Blue
@Autonomous(name="testAutonomousProgramBlueSideSpecimen", group="Auto2024")
public class firstAutonomousProgram extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    GoBildaPinpointDriver odo;
    Pose3D pos = new Pose3D(odo.getPosition().getPosition(),odo.getVelocity().getOrientation());

    public void driveDistanceForward(double power, int distance){
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setPower(power);
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        while (pos.getPosition().x < distance){
                if (pos.getPosition().x < distance/2) {
                    backLeft.setPower(power);
                    frontRight.setPower(power);
                    frontLeft.setPower(power);
                    backRight.setPower(power);
                }
                if (pos.getPosition().x > distance/2){
                    backLeft.setPower(power/2);
                    frontRight.setPower(power/2);
                    frontLeft.setPower(power/2);
                    backRight.setPower(power/2);
                }
            }
        backLeft.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
    }

    @Override
    public void runOpMode(){

    }
}
