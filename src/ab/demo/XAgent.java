package ab.demo;

import java.awt.Point;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import ab.demo.other.ActionRobot;
import ab.demo.other.Shot;
import ab.planner.TrajectoryPlanner;
import ab.utils.StateUtil;
import ab.vision.ABObject;
import ab.vision.GameStateExtractor.GameState;
import ab.vision.Vision;

public class XAgent implements Runnable {

    private ActionRobot aRobot;
    private Random randomGenerator;
    public int currentLevel = 1;
    public static int time_limit = 12;
    private Map<Integer,Integer> scores = new LinkedHashMap<Integer,Integer>();
    TrajectoryPlanner tp;
    private boolean firstShot;
    private Point prevTarget;

    public XAgent() {
        aRobot = new ActionRobot();
        tp = new TrajectoryPlanner();
        prevTarget = null;
        firstShot = true;
        randomGenerator = new Random();
        ActionRobot.GoFromMainMenuToLevelSelection();
    }

    public void run() {

    }

    private double distance(Point p1, Point p2) {
        return Math
                .sqrt((double) ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y)
                        * (p1.y - p2.y)));
    }

    public GameState solve() {
        GameState state = aRobot.getState();

        return state;
    }

    public static void main(String[] args) {
        XAgent xa = new XAgent();

        if(args.length > 0) {
            xa.currentLevel = Integer.parseInt(args[0]);
        }

        xa.run();
    }
}
