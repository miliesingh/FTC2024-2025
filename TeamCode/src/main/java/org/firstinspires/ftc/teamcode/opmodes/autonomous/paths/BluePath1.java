package org.firstinspires.ftc.teamcode.opmodes.autonomous.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.drive.roadrunner.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateCarousel;
import org.firstinspires.ftc.teamcode.subsystems.drive.roadrunner.MecanumDriveSubsystem;

public class BluePath1 {

    private MecanumDriveSubsystem drive;
    private TrajectoryFollowerCommand sample1Follower1;
    private TrajectoryFollowerCommand sample1Follower2;
    private TrajectoryFollowerCommand sample1Follower3;
    private TrajectoryFollowerCommand sample1Follower4;
    private TrajectoryFollowerCommand sample1Follower5;
    private TrajectoryFollowerCommand sample1Follower6;

    SequentialCommandGroup carouselGroup;

    private Pose2d startPose;
    private final HardwareMap hwMap;
    private final Telemetry telemetry;

    public BluePath1(HardwareMap hwMap, Telemetry telemetry){
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        drive = new MecanumDriveSubsystem(new SampleMecanumDrive(hwMap), false);
        startPose = new Pose2d(-36, 60, Math.toRadians(270));
    }

    public void createPath(){
        CreateCarousel createCarousel = new CreateCarousel(hwMap,"carousel",telemetry);
        createCarousel.createAuto();
        carouselGroup = new SequentialCommandGroup(createCarousel.getMoveCarouselToPosition(),
                new WaitUntilCommand(createCarousel.hasMaxEncoderCountSupplier()).andThen(createCarousel.getStopCarousel()));

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                //.strafeTo(new Vector2d(-60, 60))
                .splineToLinearHeading(new Pose2d(-63, 60, Math.toRadians(245)),Math.toRadians(270))
                .addDisplacementMarker(()-> {
                    telemetry.addData("Path 1", "performing path 1 action");
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-60, 24))
                .addDisplacementMarker(()->{
                    telemetry.addData("Path 2", "performing path 2 action");
                }) //step 6
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-32, 24, Math.toRadians(0)),Math.toRadians(270))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(270)),Math.toRadians(270))
                .build();

        /*Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineToLinearHeading(new Pose2d(-32, 24, Math.toRadians(0)),Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-60, 36, Math.toRadians(270)),Math.toRadians(270))
                .build();


        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(()->{
                    telemetry.addData("Path 4", "performing path 4 action");
                    allianceColor.schedule();
                }) //step 10
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(8,64,Math.toRadians(270)),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-15,40),Math.toRadians(57))
                .build();


        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d())
                .back(45)
                .build();*/

        sample1Follower1 = new TrajectoryFollowerCommand(drive,traj1);
        sample1Follower2 = new TrajectoryFollowerCommand(drive,traj2);
        sample1Follower3 = new TrajectoryFollowerCommand(drive,traj3);
        sample1Follower4 = new TrajectoryFollowerCommand(drive,traj4);
        //sample1Follower5 = new TrajectoryFollowerCommand(drive,traj5);
        //sample1Follower6 = new TrajectoryFollowerCommand(drive,traj6);
    }

    public void execute(CommandOpMode commandOpMode){
        commandOpMode.schedule(new WaitUntilCommand(commandOpMode::isStarted).andThen(
                sample1Follower1.andThen(carouselGroup,
                        sample1Follower2, sample1Follower3, sample1Follower4)
        ));
    }
}
