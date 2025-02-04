package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
@Config
@TeleOp
public class TeleOpPID extends OpMode {

/*
    ≈3.434pulses/mm  MISUMI SLIDES

 */



    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public  static  double f = 0;

    public static int target = 0 ;

    private  final double ticks_in_degree = 2.77;


    private PIDController liftcontroller;
    public static double lp = 0, li = 0 , ld = 0;
    public static double lf = 0;
    public static int ltarget = 0;
    private final double ticks_in_mm = 3.20;

    private PIDController hang1pid;
    public static double h1p = 0, h1i = 0, h1d = 0;
    public  static  double h1f = 0;

    public static int h1target = 0 ;

    private  final double h1ticks_in_degree = 3.434;

    private PIDController hang2pid;
    public static double h2p = 0, h2i = 0, h2d = 0;
    public  static  double h2f = 0;

    public static int h2target = 0 ;

    private  final double h2ticks_in_degree = 3.434;

    private DcMotorEx  motor_stanga         = null; //the arm motor
    private DcMotorEx  motor_glisiere        = null; //
    private DcMotorEx fata_stanga = null;
    private DcMotorEx fata_dreapta = null;
    private DcMotorEx spate_dreapta = null ;
    private DcMotorEx spate_stanga = null;
    private  DcMotorEx hang1 = null;
    private DcMotorEx hang2 = null;
    private Servo cleste = null;
    private Servo servoRotire = null;
    @Override
    public void init() {

        GamepadEx toolOp = new GamepadEx(gamepad2);
        GamepadEx driverOp = new GamepadEx(gamepad1);

        controller = new PIDController(p, i, d);
        liftcontroller = new PIDController(lp, li, ld);
        hang1pid = new PIDController(h1p, h1i, h1d);
        hang2pid = new PIDController(h2p, h2i, h2d);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor_glisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");
        motor_stanga = hardwareMap.get(DcMotorEx.class, "motor_stanga");
        fata_dreapta =hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        fata_stanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        spate_dreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        spate_stanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");

        hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hang2 = hardwareMap.get(DcMotorEx.class, "hang2");

        servoRotire = hardwareMap.get(Servo.class, "servoRotire");
        cleste  = hardwareMap.get(Servo.class, "cleste");

        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_glisiere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // input motors exactly as shown below



        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        liftcontroller.setPID(lp,li,ld);

        int armPos = motor_stanga.getCurrentPosition();
        int liftPos = motor_glisiere.getCurrentPosition();
        int hang1Pos = hang1.getCurrentPosition();
        int hang2Pos = hang2.getCurrentPosition();

        hang1pid.setPID(h1p, h1i, h1d);
        hang2pid.setPID(h2p, h2i, h2d);

        double pid = controller.calculate(armPos, target);
        double liftpid = liftcontroller.calculate(liftPos, ltarget);
        double h1pid = controller.calculate(hang1Pos, h1target);
        double h2pid = controller.calculate(hang2Pos, h2target);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double liftff = Math.cos(Math.toRadians(ltarget / ticks_in_mm)) * lf;
        double h1ff = Math.cos(Math.toRadians(h1target / h1ticks_in_degree)) * h1f;
        double h2ff = Math.cos(Math.toRadians(h2target / h2ticks_in_degree)) * h2f;

        double power = pid + ff;
        double liftpower = liftpid + liftff;
        double h1power = h1pid + h1ff;
        double h2power = h2pid + h2ff;


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x ; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fata_stanga.setPower(frontLeftPower);
        spate_stanga.setPower(backLeftPower);
        fata_dreapta.setPower(frontRightPower);
        spate_dreapta.setPower(backRightPower);

        motor_stanga.setPower(power);
        motor_glisiere.setPower(liftpower);
        hang1.setPower(h1power);
        hang2.setPower(h2power);


        telemetry.addData("pos arm", armPos);
        telemetry.addData("lift pos", liftPos);
        telemetry.addData("h1 pos", hang1Pos);
        telemetry.addData("h2 pos", hang2Pos);

        telemetry.update();
    }
}
