package me.drton.jmavsim;

import java.util.Random;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import me.drton.jmavlib.geo.LatLonAlt;
import me.drton.jmavlib.mavlink.MAVLinkMessage;
import me.drton.jmavlib.mavlink.MAVLinkSchema;

/** Run with assertions enabled against a staged, compiled jMAVSim tree; no GUI required. */
public final class SensorModelTest {
    private static void check(boolean condition, String message) {
        if (!condition) {
            throw new AssertionError(message);
        }
    }

    private static void close(double actual, double expected, double tolerance, String message) {
        check(Math.abs(actual - expected) <= tolerance, message + ": " + actual + " != " + expected);
    }

    private static final class Body extends DynamicObject {
        Body(World world) {
            super(world, false);
        }
        protected Vector3d getForce() {
            return new Vector3d();
        }
        protected Vector3d getTorque() {
            return new Vector3d();
        }
        protected Vector3d getAirFlowForce(Vector3d airSpeed) {
            return new Vector3d();
        }
    }

    private static void geometry() {
        Matrix3d rotation = new Matrix3d();
        rotation.setIdentity();
        check(SensorModel.headingCentidegrees(rotation, 0.5, 0.05, 0) == 36000, "north encoding");
        rotation.rotZ(Math.PI / 2);
        check(SensorModel.headingCentidegrees(rotation, 0.5, 0.05, 0) == 9000, "east heading");
        rotation.rotY(Math.PI / 2);
        check(SensorModel.headingCentidegrees(rotation, 0.5, 0.05, 0) == 0, "vertical baseline invalid");
        rotation.setIdentity();
        close(SensorModel.groundRange(-10, 0, rotation, 0.1, 30, 0), 10, 1e-9, "vertical range");
        rotation.rotX(Math.PI / 3);
        close(SensorModel.groundRange(-10, 0, rotation, 0.1, 30, 0), 20, 1e-9, "slant range");
        check(Double.isNaN(SensorModel.groundRange(-20, 0, rotation, 0.1, 30, 0)), "overrange invalid");
        rotation.rotX(Math.PI);
        check(Double.isNaN(SensorModel.groundRange(-10, 0, rotation, 0.1, 30, 0)), "upward range invalid");
        rotation.setIdentity();
        check(Double.isNaN(SensorModel.groundRange(0, 0, rotation, 0.1, 30, 0)), "below minimum invalid");

        rotation.rotZ(Math.PI / 2);
        Float[] q = SensorModel.quaternion(rotation);
        close(q[0], Math.sqrt(0.5), 1e-7, "quaternion scalar first");
        close(q[3], Math.sqrt(0.5), 1e-7, "positive NED yaw");
        Matrix3d recovered = new Matrix3d();
        recovered.set(new Quat4d(q[1], q[2], q[3], q[0]));
        Vector3d forward = new Vector3d(1, 0, 0);
        recovered.transform(forward);
        close(forward.y, 1, 1e-6, "body forward points east");
        Float[] covariance = SensorModel.covariance(0.05, Math.toRadians(0.5));
        close(covariance[11], 0.0025, 1e-9, "position variance");
        check(covariance[15] > 0 && covariance[18] > 0 && covariance[20] > 0, "attitude variance nonzero");
    }

    private static void delayAndCorrelation() {
        SensorModel.Delay<Integer> delayed = new SensorModel.Delay<Integer>(40);
        delayed.add(100, 1);
        delayed.add(140, 2);
        check(delayed.poll(139) == null, "no early release");
        check(delayed.poll(140) == 1, "40 ms delayed sample");
        check(delayed.poll(144) == null, "sample not repeated");
        check(delayed.poll(180) == 2, "second delayed sample");
        Random random = new Random(91);
        double error = 0;
        double sumSquares = 0;
        double adjacentProducts = 0;
        int samples = 400000;
        for (int i = 0; i < samples; ++i) {
            double previous = error;
            error = SensorModel.correlatedError(previous, 0.02, 0.05, 10, random);
            sumSquares += error * error;
            adjacentProducts += error * previous;
        }
        close(Math.sqrt(sumSquares / samples), 0.02, 0.002, "stationary receiver error");
        check(adjacentProducts / sumSquares > 0.98, "receiver errors temporally correlated");
    }

    private static void productionModelSelection() {
        World world = new World();
        world.setGlobalReference(new LatLonAlt(47, 8, 488));
        world.addObject(new SimpleEnvironment(world));
        // Exercise the factory used by Simulator, rather than constructing only the new class.
        me.drton.jmavsim.vehicle.AbstractMulticopter vehicle =
            Simulator.buildMulticopter(world, "models/quadrotor/quadrotor.obj", false, 0);
        check(vehicle.getSensors() instanceof PositionSourceSensors,
              "production multicopter must select extended position-source sensor model");
    }

    private static void sensorStreams() {
        World world = new World();
        world.setGlobalReference(new LatLonAlt(47, 8, 488));
        world.addObject(new SimpleEnvironment(world));
        Body body = new Body(world);
        body.setPosition(new Vector3d(0, 0, -10));
        PositionSourceSensors sensors = new PositionSourceSensors();
        PositionSourceSensors replay = new PositionSourceSensors();
        sensors.setObject(body, 0);
        replay.setObject(body, 0);
        sensors.setGPSStartTime(0);
        replay.setGPSStartTime(0);
        Vector3d imu1 = sensors.getAcc(0);
        Vector3d imu2 = sensors.getAcc(1);
        check(!imu1.equals(imu2), "independent IMU noise");
        check(imu1.equals(replay.getAcc(0)), "IMU seed repeatable");
        check(imu2.equals(replay.getAcc(1)), "secondary IMU seed repeatable");
        sensors.getAcc(0); // Extra reads must not perturb other sensor streams.
        sensors.getGyro(1);
        int gnssCount = 0;
        int visionCount = 0;
        int rangeCount = 0;
        for (long time = 0; time < 1200; time += 4) {
            body.setPosition(new Vector3d(time * 0.001, 0, -10));
            body.setVelocity(new Vector3d(1, 0, 0));
            Matrix3d rotating = new Matrix3d();
            rotating.rotZ(time * 0.001);
            body.setRotation(rotating);
            sensors.update(time, false);
            replay.update(time, false);
            if (sensors.isGPSUpdated()) {
                ++gnssCount;
                GNSSReport gnss = sensors.getGNSS();
                check(time - gnss.time / 1000 == 200, "GNSS physical sample delayed exactly 200 ms");
                check(gnss.fix == 6, "RTK fixed status");
                double heading = Math.toRadians(gnss.heading * 0.01);
                double expectedHeading = gnss.time * 1e-6;
                double headingError = Math.atan2(Math.sin(heading - expectedHeading), Math.cos(heading - expectedHeading));
                check(Math.abs(headingError) < Math.toRadians(3), "GNSS heading uses the delayed position epoch");
                close(gnss.eph, 0.02, 1e-8, "RTK horizontal accuracy");
                close(gnss.epv, 0.03, 1e-8, "RTK vertical accuracy");
            }
            PositionSourceSensors.VisionReport vision = sensors.takeVision();
            PositionSourceSensors.VisionReport replayVision = replay.takeVision();
            if (vision != null) {
                ++visionCount;
                check(time - vision.time == 40, "vision delayed exactly 40 ms");
                check(vision.position.equals(replayVision.position), "vision stream independent of IMU draws");
                check(Math.abs(vision.position.x - vision.time * 0.001) < 0.2, "fixed NED vision coordinates");
                check(Math.abs(vision.position.z + 10) < 0.2, "vision altitude NED down");
            }
            if (sensors.takeRange() != null) {
                ++rangeCount;
            }
            close(sensors.getGlobalPosition().alt, 498, 1e-9, "global truth excludes GNSS error");
            close(sensors.getPressure(), SimpleEnvironment.alt2baro(sensors.getPressureAlt()), 1e-9,
                  "barometer pressure/altitude coherent");
        }
        check(gnssCount == 20, "20 Hz GNSS after initial 200 ms fill: " + gnssCount);
        check(visionCount == 29, "25 Hz vision after 40 ms fill: " + visionCount);
        check(rangeCount == 24, "20 Hz range: " + rangeCount);
    }

    private static void wireFormat(String schemaFile) throws Exception {
        MAVLinkSchema schema = new MAVLinkSchema(schemaFile);
        MAVLinkMessage imu = new MAVLinkMessage(schema, "HIL_SENSOR", 1, 51, 2);
        imu.set("id", 1);
        MAVLinkMessage decoded = new MAVLinkMessage(schema, imu.encode().order(java.nio.ByteOrder.BIG_ENDIAN));
        check(decoded.protocolVersion == 2 && decoded.getInt("id") == 1, "secondary IMU MAVLink 2 ID");
        MAVLinkMessage gnss = new MAVLinkMessage(schema, "HIL_GPS", 1, 51, 2);
        gnss.set("yaw", 36000);
        decoded = new MAVLinkMessage(schema, gnss.encode().order(java.nio.ByteOrder.BIG_ENDIAN));
        check(decoded.getInt("yaw") == 36000, "north heading wire roundtrip");
        MAVLinkMessage odometry = new MAVLinkMessage(schema, "ODOMETRY", 1, 51, 2);
        odometry.set("quality", 100);
        odometry.set("pose_covariance", SensorModel.covariance(0.05, 0.01));
        decoded = new MAVLinkMessage(schema, odometry.encode().order(java.nio.ByteOrder.BIG_ENDIAN));
        check(decoded.getInt("quality") == 100, "current-schema odometry quality");
        check(((Number)((Object[])decoded.get("pose_covariance"))[20]).floatValue() > 0,
              "covariance survives wire format");
    }

    public static void main(String[] args) throws Exception {
        geometry();
        delayAndCorrelation();
        productionModelSelection();
        sensorStreams();
        wireFormat(args[0]);
        System.out.println("SensorModelTest: production model selection, geometry, covariance, delays, rates, independent streams and MAVLink 2 passed");
    }
}
