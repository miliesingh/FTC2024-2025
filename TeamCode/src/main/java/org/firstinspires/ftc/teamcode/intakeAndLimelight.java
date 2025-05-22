package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


@TeleOp (name = "LimelightAndIntake", group  = "Linear OpMode")
public class intakeAndLimelight extends LinearOpMode{


    Limelight3A limelight;


    @Override
    public void runOpMode(){
        List<Double> distances = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        waitForStart();
        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            limelight.pipelineSwitch(2);
            int index = result.getPipelineIndex();
            if (result != null){
                if (result.isValid()){
                    double sum = 0;
                    double average = 0;
                    double ty = result.getTy();
                    double limelightMountAngleDegrees = 24.5;
                    double limelightLensHeightCentimeters = 25.4;
                    double angleToGoalDegrees = limelightMountAngleDegrees - ty;
                    double angleToGoalRadians = angleToGoalDegrees * (3.14159/180.0);
                    double distance = limelightLensHeightCentimeters/Math.tan(angleToGoalRadians);
                    distances.add(0,distance);
                    distances.remove(6);
                    for (int number = 0; number < distances.size(); number++){
                        sum+= distances.get(number);
                    }
                    average = sum/distances.size();
                    telemetry.addData("Distance", distance);
                    telemetry.addData("ty: ", ty);
                    telemetry.addData("Distances", average);
                }
            }
            else{
                telemetry.addData("nothing", 0);
            }


            telemetry.addData("index", index);
            telemetry.update();
        }
        limelight.stop();
    }
}
