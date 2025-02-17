package pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "NextFTC Autonomous Program 2 Java")
public class AutonomousProgram extends PedroOpMode {
    public static class Lift extends Subsystem {
        // BOILERPLATE
        public static final Lift INSTANCE = new Lift();
        private Lift() { }

        // USER CODE
        public MotorEx motor_glisiere;

        private PIDController lcontroller;
        public static double lp = 0.01, li = 0, ld = 0.0002;
        public static double lf = 0.14;
        public static double ltarget = 0 ;

        private final double ticks_in_mm = 3.20;
        public String name = "lift_motor";

        public Command toLow() {
            return new RunToPosition(motor, // MOTOR TO MOVE
                    0.0, // TARGET POSITION, IN TICKS
                    controller, // CONTROLLER TO IMPLEMENT
                    this); // IMPLEMENTED SUBSYSTEM
        }

        public Command toMiddle() {
            return new RunToPosition(motor, // MOTOR TO MOVE
                    500.0, // TARGET POSITION, IN TICKS
                    controller, // CONTROLLER TO IMPLEMENT
                    this); // IMPLEMENTED SUBSYSTEM
        }

        public Command toHigh() {
            return new RunToPosition(motor, // MOTOR TO MOVE
                    1200.0, // TARGET POSITION, IN TICKS
                    controller, // CONTROLLER TO IMPLEMENT
                    this); // IMPLEMENTED SUBSYSTEM
        }

        @Override
        public void initialize() {
            motor = new MotorEx(name);
        }
    }


    public AutonomousProgram() {
        super(Claw.INSTANCE, Lift.INSTANCE);
    }

    private final Pose startPose = new Pose(9.0, 60.0, Math.toRadians(0.0));
    private final Pose scorePose = new Pose(37.0, 50.0, Math.toRadians(180.0));
    private final Pose pickup1Pose = new Pose(0,0,Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(0,0,Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(0,0,Math.toRadians(0));

    private final Pose parkPose = new Pose(0,0,Math.toRadians(0));


    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, scorePreload, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public Command secondRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Arm.INSTANCE.toBasket(),
                        Lift.INSTANCE.toBasket(),
                        Claw.INSTANCE.closed()

                ),

                Claw.INSTANCE.open(),

                new ParallelGroup(
                        new FollowPath(grabPickup1),
                        Arm.INSTANCE.toPickup(),
                        Lift.INSTANCE.toPickup()
                ),

                Claw.INSTANCE.closed();

                new ParallelGroup(
                        new FollowPath(scorePickup1),
                        Arm.INSTANCE.toBasket(),
                        Lift.INSTANCE.toBasket()
                ),

                Claw.INSTANCE.open(),

                new ParallelGroup(
                        new FollowPath(grabPickup2),
                        Arm.INSTANCE.toPickup(),
                        Lift.INSTANCE.toPickup()
                ),

                Claw.INSTANCE.closed(),

                new ParallelGroup(
                        new FollowPath(scorePickup2),
                        Arm.INSTANCE.toBasket(),
                        Lift.INSTANCE.toBasket()
                ),

                Claw.INSTANCE.open(),

                new ParallelGroup(
                        new FollowPath(grabPickup3),
                        Arm.INSTANCE.toPickup(),
                        Lift.INSTANCE.toPickup()
                ),

                Claw.INSTANCE.closed(),

                new ParallelGroup(
                        new FollowPath(scorePickup3),
                        Arm.INSTANCE.toBasket(),
                        Lift.INSTANCE.toBasket()
                ),

                Claw.INSTANCE.open(),

                new ParallelGroup(
                        new FollowPath(Park),
                        Arm.INSTANCE.toLow(),
                        Lift.INSTANCE.toLow()
                );
        )
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine();
    }
}