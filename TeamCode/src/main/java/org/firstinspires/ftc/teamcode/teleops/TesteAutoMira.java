package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;          // se o seu Pedro 3 usar outro pacote, ajuste este import
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants; // sua classe Constants do Pedro
import org.firstinspires.ftc.teamcode.storage.TorretaConfig;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "Mira automática - teste", group = "Turret")
public class TesteAutoMira extends OpMode {
    private Follower follower;
    private final Turret Turret = new Turret();

    private boolean redAlliance = true;
    private boolean autoAim = true;
    private boolean lastY = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Em teleop depois do auto, use a pose final do auto aqui. Isto é só um exemplo:
        follower.setPose(new Pose(87.4, 133.8, Math.toRadians(90)));
        Turret.init(hardwareMap, true); // true = zera o encoder (Turret apontando para a frente). Use false após o auto.
    }

    @Override
    public void loop() {
        // ---- Drive (Pedro) ----
        // Pedro 3: sem startTeleOpDrive/setTeleOpDrive. Use ManualDrive (importe com Alt+Enter).
        DrivePowers stick = new DrivePowers(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        DrivePowers field = ManualDrive.fieldCentric(stick, follower.pose().heading());
        follower.manual(field);
        follower.update();
        com.pedropathing.math.Pose pose = follower.pose();

        // ---- Seleção de aliança / liga-desliga ----
        if (gamepad1.x) redAlliance = true;
        if (gamepad1.b) redAlliance = false;
        if (gamepad1.y && !lastY) autoAim = !autoAim;
        lastY = gamepad1.y;

        double tx = redAlliance ? TorretaConfig.HIVE_RED_X : TorretaConfig.HIVE_BLUE_X;
        double ty = redAlliance ? TorretaConfig.HIVE_RED_Y : TorretaConfig.HIVE_BLUE_Y;

        // ---- Turret ----
        if (autoAim) Turret.aimAt(pose.x(), pose.y(), pose.heading(), tx, ty);
        else Turret.setManualPower(gamepad2.right_stick_x * 0.5); // manual / calibração dos sinais

        telemetry.addData("Aliança", redAlliance ? "VERMELHA" : "AZUL");
        telemetry.addData("AutoAim", autoAim);
        telemetry.addData("Pose", "%.1f, %.1f, %.1f°", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Turret ângulo", "%.2f°", Turret.getAngleDeg());
        telemetry.addData("Turret alvo", "%.2f°", Turret.getTargetAngleDeg());
        telemetry.addData("Erro", "%.2f°", Turret.getErrorDeg());
        telemetry.addData("Potência", "%.2f", Turret.getPower());
        telemetry.update();
    }

    @Override
    public void stop() { Turret.stop(); }
}