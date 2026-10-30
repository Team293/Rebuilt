import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import frc.lib.FieldConstants;
import java.nio.file.*;
import java.util.*;
import org.json.simple.*;
import org.json.simple.parser.JSONParser;

/** Offline coordinate search over collision-free Bezier routes, using PathPlanner's dynamics.
 * Run with the project test runtime classpath and desktop native library directory.
 * Pickup anchors and shooting windows are fixed; only transit geometry is optimized.
 */
public class OptimizeDoublePass {
    static final Path PP = Path.of("src/main/deploy/pathplanner");
    static final double HALF = .8255 / 2, LANE = .639318;
    static final double START = FieldConstants.LinesVertical.starting - HALF;
    static final double BUMP_Y = (FieldConstants.LinesHorizontal.rightBumpEnd
            + FieldConstants.Hub.nearRightCorner.getY()) / 2;
    static RobotConfig config;
    static JSONObject firstTemplate, secondTemplate;
    static int evaluations;
    static final double[] LOW = {2.8, 5.70, .25, .08, .08, 5.70, .20, 0, .30, .25, .20, .20, .08, .08};
    static final double[] HIGH = {START, 6.4, 1.8, .70, .65, 6.4, 1.5, 1.3, 2.0, 1.3, 1.3, 1.8, .70, .8};
    static final double[] STEP = {.15, .15, .25, .12, .10, .15, .20, .20, .25, .20, .20, .25, .12, .1};
    record Result(JSONObject first, JSONObject second, double seconds, double firstSeconds, double distance) {}

    @SuppressWarnings("unchecked")
    static JSONObject obj(Object... args) {
        JSONObject out = new JSONObject();
        for (int i = 0; i < args.length; i += 2) out.put(args[i], args[i + 1]);
        return out;
    }
    static JSONObject point(double x, double y) { return obj("x", x, "y", y); }
    static JSONObject wp(double x, double y, JSONObject prev, JSONObject next) {
        return obj("anchor", point(x, y), "prevControl", prev, "nextControl", next,
                "isLocked", false, "linkedName", null);
    }
    @SuppressWarnings("unchecked")
    static JSONArray array(Object... entries) {
        JSONArray a = new JSONArray(); Collections.addAll(a, entries); return a;
    }
    static JSONObject rot(double pos, double degrees) { return obj("waypointRelativePos", pos, "rotationDegrees", degrees); }
    static JSONObject constraints(double v, double a, double omega, double alpha) {
        return obj("maxVelocity", v, "maxAcceleration", a, "maxAngularVelocity", omega,
                "maxAngularAcceleration", alpha, "nominalVoltage", 12.0, "unlimited", false);
    }
    static JSONObject fullConstraints() {
        double radius = Arrays.stream(config.modulePivotDistance).max().orElseThrow();
        double torque = Arrays.stream(config.modulePivotDistance).sum() * config.wheelFrictionForce;
        return constraints(config.moduleConfig.maxDriveVelocityMPS,
                config.wheelFrictionForce * config.numModules / config.massKG,
                Math.toDegrees(config.moduleConfig.maxDriveVelocityMPS / radius),
                Math.toDegrees(torque / config.MOI));
    }
    static JSONObject zone(String name, double from, double to, JSONObject c) {
        return obj("name", name, "minWaypointRelativePos", from, "maxWaypointRelativePos", to, "constraints", c);
    }
    static JSONObject event(String label, double pos, String command) {
        return obj("name", label, "waypointRelativePos", pos, "endWaypointRelativePos", null,
                "command", obj("type", "named", "data", obj("name", command)));
    }
    static JSONObject copy(JSONObject o) throws Exception { return (JSONObject) new JSONParser().parse(o.toJSONString()); }
    static double num(JSONObject o, String key) { return ((Number)o.get(key)).doubleValue(); }
    static Translation2d translation(JSONObject o) { return o == null ? null : new Translation2d(num(o, "x"), num(o, "y")); }
    static PathConstraints parseConstraints(JSONObject o) {
        return new PathConstraints(num(o,"maxVelocity"), num(o,"maxAcceleration"),
                Math.toRadians(num(o,"maxAngularVelocity")), Math.toRadians(num(o,"maxAngularAcceleration")),12,false);
    }
    static PathPlannerPath path(JSONObject o) {
        List<Waypoint> w = new ArrayList<>();
        for (Object item : (JSONArray)o.get("waypoints")) {
            JSONObject p = (JSONObject)item;
            w.add(new Waypoint(translation((JSONObject)p.get("prevControl")), translation((JSONObject)p.get("anchor")),
                    translation((JSONObject)p.get("nextControl"))));
        }
        List<RotationTarget> rotations = new ArrayList<>();
        for (Object item : (JSONArray)o.get("rotationTargets")) {
            JSONObject p = (JSONObject)item;
            rotations.add(new RotationTarget(num(p,"waypointRelativePos"), Rotation2d.fromDegrees(num(p,"rotationDegrees"))));
        }
        List<ConstraintsZone> zones = new ArrayList<>();
        for (Object item : (JSONArray)o.get("constraintZones")) {
            JSONObject p = (JSONObject)item;
            zones.add(new ConstraintsZone(num(p,"minWaypointRelativePos"),num(p,"maxWaypointRelativePos"),parseConstraints((JSONObject)p.get("constraints"))));
        }
        JSONObject goal = (JSONObject)o.get("goalEndState");
        return new PathPlannerPath(w, rotations, List.of(), zones, List.of(), parseConstraints((JSONObject)o.get("globalConstraints")),
                new IdealStartingState(0,Rotation2d.kZero),new GoalEndState(0,Rotation2d.fromDegrees(num(goal,"rotation"))),false);
    }
    // Solve monotonically decreasing return segment for a bumper-clear bump-zone boundary.
    static double positionAtX(double gate, double shot, double x) {
        if (x >= gate) return 4;
        if (x <= shot) return 5;
        double low = 0, high = 1;
        for (int i = 0; i < 40; i++) {
            double t = (low + high) / 2, u = 1 - t;
            double px = u*u*u*gate + 3*u*u*t*(gate-.8) + 3*u*t*t*(shot+.8) + t*t*t*shot;
            if (px > x) low = t; else high = t;
        }
        return 4 + (low + high) / 2;
    }
    @SuppressWarnings("unchecked")
    static Result evaluate(double[] p) throws Exception {
        evaluations++;
        JSONObject first = copy(firstTemplate), second = copy(secondTemplate);
        first.put("waypoints",array(
                wp(START,LANE,null,point(START+.8,LANE)),
                wp(p[1],LANE,point(p[1]-.8,LANE),point(p[1]+p[2],LANE)),
                wp(7.85,.9,point(7.85,.9-p[3]),point(7.85,1.15)),
                wp(7.85,2.34,point(7.85,2.09),point(7.85,2.34+p[4])),
                wp(p[5],BUMP_Y,point(p[5]+p[6],BUMP_Y),point(p[5]-.8,BUMP_Y)),
                wp(p[0],BUMP_Y,point(p[0]+.8,BUMP_Y),null)));
        first.put("rotationTargets", array(rot(1,0),rot(2,90),rot(3,90),rot(4,0)));
        first.put("constraintZones",array(zone("40% pickup speed",1.75,3.05,constraints(1.48,1,180,360)),
                zone("Bump crossing",positionAtX(p[5],p[0],5.70),positionAtX(p[5],p[0],3.52),constraints(1.5,1.5,180,360))));
        first.put("eventMarkers",array(event("Begin pickup",1.8,"enableIntake"),event("Batch collected",3.0,"disableIntake")));
        first.put("globalConstraints",fullConstraints());
        second.put("waypoints",array(
                wp(p[0],BUMP_Y,null,point(p[0]-p[7],BUMP_Y-p[8])),
                wp(3.55,LANE,point(3.55-p[9],LANE),point(3.55+p[10],LANE)),
                wp(p[1],LANE,point(p[1]-.8,LANE),point(p[1]+p[11],LANE)),
                wp(8.15,.9,point(8.15,.9-p[12]),point(8.15,1.15)),
                wp(8.15,3.12,point(8.15,2.87),point(8.15,3.12+p[13])),
                wp(p[5],BUMP_Y,point(p[5]+p[6],BUMP_Y),point(p[5]-.8,BUMP_Y)),
                wp(p[0],BUMP_Y,point(p[0]+.8,BUMP_Y),null)));
        second.put("rotationTargets",array(rot(1,0),rot(2,0),rot(3,90),rot(4,90),rot(5,0)));
        second.put("constraintZones",array(zone("40% pickup speed",2.75,4.05,constraints(1.48,1,180,360)),
                zone("Bump crossing",1+positionAtX(p[5],p[0],5.70),1+positionAtX(p[5],p[0],3.52),constraints(1.5,1.5,180,360))));
        second.put("eventMarkers",array(event("Begin second pickup",2.8,"enableIntake"),
                event("Second batch collected",4.0,"disableIntake")));
        second.put("goalEndState",obj("velocity",0,"rotation",0));
        second.put("globalConstraints",fullConstraints());
        PathPlannerPath a = path(first), b = path(second);
        PathPlannerTrajectory ta = a.getIdealTrajectory(config).orElseThrow(), tb = b.getIdealTrajectory(config).orElseThrow();
        if (!clear(ta) || !clear(tb)) return null;
        double seconds = ta.getTotalTimeSeconds() + tb.getTotalTimeSeconds();
        if (!Double.isFinite(seconds)) return null;
        double distance = 0;
        for (PathPlannerPath leg : List.of(a,b)) distance += leg.getPoint(leg.numPoints()-1).distanceAlongPath;
        return new Result(first,second,seconds,ta.getTotalTimeSeconds(),distance);
    }
    static boolean clear(PathPlannerTrajectory trajectory) {
        double prevX = trajectory.getInitialState().pose.getX();
        for (var sample : trajectory.getStates()) {
            Pose2d pose = sample.pose;
            double extent = HALF * (Math.abs(pose.getRotation().getCos())+Math.abs(pose.getRotation().getSin()));
            double x = pose.getX(), y = pose.getY();
            if (x-extent < 0 || x+extent > FieldConstants.fieldLength || y-extent < .015 || y+extent > FieldConstants.fieldWidth) return false;
            if (x+extent > FieldConstants.LinesVertical.starting+1e-6 && x-extent < FieldConstants.Hub.farRightCorner.getX()) {
                if (Math.abs(pose.getRotation().getRadians()) > 1e-6) return false;
                if (x >= prevX) {
                    if (y+extent > FieldConstants.RightTrench.openingWidth-.015) return false;
                } else {
                    if (y-extent < FieldConstants.LinesHorizontal.rightBumpEnd+.015
                            || y+extent > FieldConstants.Hub.nearRightCorner.getY()-.015 || sample.linearVelocity > 1.52) return false;
                }
            }
            prevX = x;
        }
        return true;
    }
    @SuppressWarnings("unchecked")
    static JSONObject reflect(JSONObject input) throws Exception {
        JSONObject o = copy(input);
        for (Object item : (JSONArray)o.get("waypoints")) {
            JSONObject w = (JSONObject)item;
            for (String key : List.of("anchor","prevControl","nextControl")) {
                JSONObject p = (JSONObject)w.get(key);
                if (p != null) p.put("y",FieldConstants.fieldWidth-num(p,"y"));
            }
        }
        for (Object item : (JSONArray)o.get("rotationTargets")) {
            JSONObject r = (JSONObject)item; r.put("rotationDegrees",-num(r,"rotationDegrees"));
        }
        for (String key : List.of("idealStartingState","goalEndState")) {
            JSONObject g = (JSONObject)o.get(key); g.put("rotation",-num(g,"rotation"));
        }
        return o;
    }
    public static void main(String[] args) throws Exception {
        HAL.initialize(500,0);
        config = RobotConfig.fromGUISettings();
        firstTemplate = (JSONObject)new JSONParser().parse(Files.readString(PP.resolve("paths/Outpost Double Pass - Bump Return.path")));
        secondTemplate = (JSONObject)new JSONParser().parse(Files.readString(PP.resolve("paths/Outpost Double Pass - Trench Harvest.path")));
        double[] bestP = {3.4125,6.0,.45625,.44,.35,5.9625,1.0,.4,.7875,.45,.65,1.4875,.32,.3};
        Result best = evaluate(bestP);
        if (best == null) throw new IllegalStateException("Initial candidate fails clearance");
        System.out.printf("Initial second shot begins: %.3f s%n",5+best.seconds);
        Random random = new Random(293);
        for (int seed = 0; seed < 6; seed++) {
            double[] currentP = bestP.clone();
            if (seed > 0) {
                for (int axis = 0; axis < currentP.length; axis++) {
                    currentP[axis] = Math.max(LOW[axis],Math.min(HIGH[axis],
                            currentP[axis] + (random.nextDouble()*2-1)*STEP[axis]*2));
                }
            }
            Result current = evaluate(currentP);
            if (current == null) { currentP = bestP.clone(); current = best; }
            for (double stepScale : new double[] {1,.5,.25,.125}) {
                for (int pass=0; pass<8; pass++) {
                    boolean improved = false;
                    for (int axis=0; axis<currentP.length; axis++) {
                        for (int sign : new int[] {-1,1}) {
                            double[] p = currentP.clone();
                            p[axis] = Math.max(LOW[axis],Math.min(HIGH[axis],p[axis]+sign*STEP[axis]*stepScale));
                            Result r = evaluate(p);
                            if (r != null && r.seconds < current.seconds-.0001) {
                                current=r;currentP=p;improved=true;
                            }
                        }
                    }
                    if (!improved) break;
                }
            }
            if (current.seconds < best.seconds) { best=current;bestP=currentP; }
            System.out.printf("Seed %d: best second shot begins %.3f s, first leg %.3f s%n",seed,5+best.seconds,best.firstSeconds);
        }
        for (String side : List.of("Outpost","Depot")) {
            Files.writeString(PP.resolve("paths/"+side+" Double Pass - Bump Return.path"),
                    (side.equals("Outpost")?best.first:reflect(best.first)).toJSONString()+"\n");
            Files.writeString(PP.resolve("paths/"+side+" Double Pass - Trench Harvest.path"),
                    (side.equals("Outpost")?best.second:reflect(best.second)).toJSONString()+"\n");
        }
        double secondShotBegins = 5+best.seconds;
        JSONObject report = obj("evaluatedCandidates",evaluations,"parameters",Arrays.toString(bestP),
                "firstPathSeconds",best.firstSeconds,"secondPathSeconds",best.seconds-best.firstSeconds,
                "secondShotBeginsSeconds",secondShotBegins,
                "secondShotAvailableSeconds",Math.max(0,Math.min(5,20-secondShotBegins)),
                "fullRouteSeconds",10+best.seconds,"pathDistanceMeters",best.distance,
                "constraints",fullConstraints(),"method","Coordinate descent over Bezier handles and transit anchors; fixed pickup sweeps and shooting windows. Best of six deterministic coordinate searches within the route family, using the configured dynamics.");
        Files.writeString(Path.of("docs/autos/double-pass-optimization.json"),report.toJSONString()+"\n");
        System.out.println(report.toJSONString());
    }
}
