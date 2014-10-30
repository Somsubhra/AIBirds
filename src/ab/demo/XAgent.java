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
import ab.vision.ABShape;
import ab.vision.ABType;
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

        aRobot.loadLevel(currentLevel);

        while (true) {
            GameState state = solve();
            if (state == GameState.WON) {
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                int score = StateUtil.getScore(ActionRobot.proxy);
                if(!scores.containsKey(currentLevel))
                    scores.put(currentLevel, score);
                else
                {
                    if(scores.get(currentLevel) < score)
                        scores.put(currentLevel, score);
                }
                int totalScore = 0;
                for(Integer key: scores.keySet()){

                    totalScore += scores.get(key);
                    System.out.println(" Level " + key
                            + " Score: " + scores.get(key) + " ");
                }
                System.out.println("Total Score: " + totalScore);
                aRobot.loadLevel(++currentLevel);
                // make a new trajectory planner whenever a new level is entered
                tp = new TrajectoryPlanner();

                // first shot on this level, try high shot first
                firstShot = true;
            } else if (state == GameState.LOST) {
                System.out.println("Restart");
                aRobot.restartLevel();
            } else if (state == GameState.LEVEL_SELECTION) {
                System.out
                        .println("Unexpected level selection page, go to the last current level : "
                                + currentLevel);
                aRobot.loadLevel(currentLevel);
            } else if (state == GameState.MAIN_MENU) {
                System.out
                        .println("Unexpected main menu page, go to the last current level : "
                                + currentLevel);
                ActionRobot.GoFromMainMenuToLevelSelection();
                aRobot.loadLevel(currentLevel);
            } else if (state == GameState.EPISODE_MENU) {
                System.out
                        .println("Unexpected episode menu page, go to the last current level : "
                                + currentLevel);
                ActionRobot.GoFromMainMenuToLevelSelection();
                aRobot.loadLevel(currentLevel);
            }

        }
    }

    private double distance(Point p1, Point p2) {
        return Math
                .sqrt((double) ((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y)
                        * (p1.y - p2.y)));
    }

    /**
     * The class/structure representing a block
     */
    private class ABBlock {

        /**
         * The block number
         */
        public int number;

        /**
         * The shape of the block
         */
        public String shape;

        /**
         * The material of the block
         */
        public String material;

        /**
         * Dump the variables on screen
         */
        public void dumpVars() {
            System.out.println("Block number: " + number);
            System.out.println("Block shape: " + shape);
            System.out.println("Block material: " + material);
            System.out.println();
        }
    }

    /**
     * Get all the blocks in the scene
     * @param vision The vision object of the scene
     * @return A list of blocks
     */
    private List<ABBlock> blocks(Vision vision) {

        // Find all the blocks appearing in the scene
        List<ABObject> objectsList = vision.findBlocksMBR();

        List<ABBlock> blocksList = new ArrayList<ABBlock>();

        // Iterate through all the blocks
        for(ABObject object : objectsList) {

            ABBlock block = new ABBlock();

            // Set the block number
            block.number = object.id;

            // Set the block shape
            ABShape shape = object.shape;

            if(shape == ABShape.Circle) {
                block.shape = "Circle";
            } else if(shape == ABShape.Poly) {
                block.shape = "Polygon";
            } else if(shape == ABShape.Rect) {
                block.shape = "Rectangle";
            } else if(shape == ABShape.Triangle) {
                block.shape = "Triangle";
            } else {
                block.shape = "Unknown";
            }

            // Set the block material
            ABType material = object.type;

            if(material == ABType.Stone) {
                block.material = "Stone";
            } else if(material == ABType.Wood) {
                block.material = "Wood";
            } else if(material == ABType.Ice) {
                block.material = "Ice";
            } else {
                block.material = "Unknown";
            }

            // Append the block to the list
            blocksList.add(block);
        }

        return blocksList;
    }

    public GameState solve() {

        System.out.println("Solving");

        // Capture the image
        BufferedImage screenshot = ActionRobot.doScreenShot();

        // Process the image
        Vision vision = new Vision(screenshot);

        // Find the slingshot
        Rectangle sling = vision.findSlingshotMBR();

        while(sling == null && aRobot.getState() == GameState.PLAYING) {
            System.out.println("No slingshot detected. Please remove pop up or zoom out");
            ActionRobot.fullyZoomOut();
            screenshot = ActionRobot.doScreenShot();
            vision = new Vision(screenshot);
            sling = vision.findSlingshotMBR();
        }

        // Get all the blocks
        List<ABBlock> blocks = this.blocks(vision);

        // Get all the pigs
        List<ABObject> pigs = vision.findPigsMBR();

        GameState state = aRobot.getState();

        // If there is a sling, play else skip
        if(sling != null) {

            if(!pigs.isEmpty()) {

                Point releasePoint = null;
                Shot shot = new Shot();
                int dx, dy;

                {
                    // Pick up the topmost pig

                    int numberOfPigs = pigs.size();

                    ABObject topmostPig = null;

                    if(numberOfPigs > 0) {

                        topmostPig = pigs.get(0);

                        for (int index = 1; index < numberOfPigs; index++) {

                            ABObject currentPig = pigs.get(index);

                            // If the current top most pig has y co-ordinate greater than the other pig
                            // then the other pig is at a greater height
                            if(topmostPig.getCenter().getY() > currentPig.getCenter().getY()) {
                                topmostPig = currentPig;
                            }
                        }
                    }

                    Point _tpt = topmostPig.getCenter();

                    // If the target is very close to before, randomly choose a
                    // point near it
                    if (prevTarget != null && distance(prevTarget, _tpt) < 10) {
                        double _angle = randomGenerator.nextDouble() * Math.PI * 2;
                        _tpt.x = _tpt.x + (int) (Math.cos(_angle) * 10);
                        _tpt.y = _tpt.y + (int) (Math.sin(_angle) * 10);
                        System.out.println("Randomly changing to " + _tpt);
                    }

                    System.out.println("x: " + _tpt.x + ", y: " + _tpt.y);

                    prevTarget = new Point(_tpt.x, _tpt.y);

                    // Estimate the trajectory
                    ArrayList<Point> pts = tp.estimateLaunchPoint(sling, _tpt);

                    // Do a high shot when entering a level to find an accurate velocity
                    if (firstShot && pts.size() > 1)
                    {
                        releasePoint = pts.get(1);
                    }
                    else if (pts.size() == 1)
                        releasePoint = pts.get(0);
                    else if (pts.size() == 2)
                    {
                        // Choose the trajectory with the higher angle
                        releasePoint = pts.get(1);
                    }
                    else
                    if(pts.isEmpty())
                    {
                        System.out.println("No release point found for the target");
                        System.out.println("Try a shot with 45 degree");
                        releasePoint = tp.findReleasePoint(sling, Math.PI/4);
                    }

                    // Get the reference point
                    Point refPoint = tp.getReferencePoint(sling);


                    //Calculate the tapping time according the bird type
                    if (releasePoint != null) {
                        double releaseAngle = tp.getReleaseAngle(sling,
                                releasePoint);
                        System.out.println("Release Point: " + releasePoint);
                        System.out.println("Release Angle: "
                                + Math.toDegrees(releaseAngle));
                        int tapInterval = 0;
                        switch (aRobot.getBirdTypeOnSling())
                        {

                            case RedBird:
                                tapInterval = 0; break;               // start of trajectory
                            case YellowBird:
                                tapInterval = 65 + randomGenerator.nextInt(25);break; // 65-90% of the way
                            case WhiteBird:
                                tapInterval =  70 + randomGenerator.nextInt(20);break; // 70-90% of the way
                            case BlackBird:
                                tapInterval =  70 + randomGenerator.nextInt(20);break; // 70-90% of the way
                            case BlueBird:
                                tapInterval =  65 + randomGenerator.nextInt(20);break; // 65-85% of the way
                            default:
                                tapInterval =  60;
                        }

                        int tapTime = tp.getTapTime(sling, releasePoint, _tpt, tapInterval);
                        dx = (int)releasePoint.getX() - refPoint.x;
                        dy = (int)releasePoint.getY() - refPoint.y;
                        shot = new Shot(refPoint.x, refPoint.y, dx, dy, 0, tapTime);
                    }
                    else
                    {
                        System.err.println("No Release Point Found");
                        return state;
                    }
                }

                // check whether the slingshot is changed. the change of the slingshot indicates a change in the scale.
                {
                    ActionRobot.fullyZoomOut();
                    screenshot = ActionRobot.doScreenShot();
                    vision = new Vision(screenshot);
                    Rectangle _sling = vision.findSlingshotMBR();
                    if(_sling != null)
                    {
                        double scale_diff = Math.pow((sling.width - _sling.width),2) +  Math.pow((sling.height - _sling.height),2);
                        if(scale_diff < 25)
                        {
                            if(dx < 0)
                            {
                                aRobot.cshoot(shot);
                                state = aRobot.getState();
                                if ( state == GameState.PLAYING )
                                {
                                    screenshot = ActionRobot.doScreenShot();
                                    vision = new Vision(screenshot);
                                    List<Point> traj = vision.findTrajPoints();
                                    tp.adjustTrajectory(traj, sling, releasePoint);
                                    firstShot = false;
                                }
                            }
                        }
                        else
                            System.out.println("Scale is changed, can not execute the shot, will re-segement the image");
                    }
                    else
                        System.out.println("no sling detected, can not execute the shot, will re-segement the image");
                }

            }

        }

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
