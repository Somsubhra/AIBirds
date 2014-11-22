package ab.demo;

import java.awt.Point;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.util.*;

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

    public ArrayList<Point> getReleasePoint(Vision vision) {

        List<ABObject> pigs = vision.findPigsMBR();
        int numberOfPigs = pigs.size();

        // A list of release points with their respective ranks
        HashMap<Point, Set<Point>> rankList = new HashMap<Point, Set<Point>>();

        // Traverse the entire trajectory space based on the pigs
        for(int index = 0; index < numberOfPigs; index++) {
            ABObject pig = pigs.get(index);

            int pigX = pig.x;
            int pigY = pig.y;

            int pigWidth = pig.width;
            int pigHeight = pig.height;

            // Iterate over the width of the pig
            for(int iterX = pigX; iterX <= pigX + pigWidth; iterX++) {

                // Iterate over the height of the pig
                for(int iterY = pigY; iterY <= pigY + pigHeight; iterY++) {

                    Point targetPoint = new Point(iterX, iterY);

                    Set<Point> launchPoints = rankList.keySet();
                    
                    ArrayList<Point> newLaunchPoints = tp.estimateLaunchPoint(vision.findSlingshotMBR(), targetPoint);
                    int newLaunchPointNo = newLaunchPoints.size();

                    // Iterate over all the possible launch points calculated
                    for(int iterLPts = 0; iterLPts < newLaunchPointNo; iterLPts++) {

                        Point newLaunchPoint = newLaunchPoints.get(iterLPts);

                        boolean inRankList = launchPoints.contains(newLaunchPoint);

                        // If launch point does not appear in the key set then initialize it
                        if(!inRankList) {
                            Set<Point> s = new HashSet<Point>();
                            s.add(pig.getCenter());
                            rankList.put(newLaunchPoint, s);
                        } else {
                            rankList.get(newLaunchPoint).add(pig.getCenter());
                        }
                    }
                }
            }
        }

        // Find the maximum number of pigs lying on one particular trajectory
        Set<Point> launchPoints = rankList.keySet();

        int maxNoPigs = 0;

        for(Iterator<Point> it = launchPoints.iterator(); it.hasNext(); ) {
            Point launchPoint = it.next();

            int pigsOnTraj = rankList.get(launchPoint).size();

            if(pigsOnTraj > maxNoPigs) {
                maxNoPigs = pigsOnTraj;
            }
        }

        // Find the launch points with the maximum pigs lying on their path

        // List of results with launch point at index 0 and target point at index 1
        ArrayList<ArrayList<Point>> resLaunchPoints = new ArrayList<ArrayList<Point>>();

        // List of just the launch points
        ArrayList<Point> resLaunchPointsList = new ArrayList<Point>();

        // List of just the target points
        ArrayList<Point> resTargetPointsList = new ArrayList<Point>();

        for(Iterator<Point> it = launchPoints.iterator(); it.hasNext(); ) {
            Point launchPoint = it.next();

            int pigsOnTraj = rankList.get(launchPoint).size();

            if(pigsOnTraj == maxNoPigs) {

                resLaunchPointsList.add(launchPoint);

                ArrayList<Point> record = new ArrayList<Point>();
                record.add(launchPoint);

                // Find the farthest point and add that as the target point
                Set<Point> resTargetPoints = rankList.get(launchPoint);

                Point farthestPoint = resTargetPoints.iterator().next();

                for(Iterator<Point> it1 = resTargetPoints.iterator(); it1.hasNext(); ) {
                    Point nextPoint = it1.next();
                    if(nextPoint.x > farthestPoint.x) {
                        farthestPoint = nextPoint;
                    }
                }

                resTargetPointsList.add(farthestPoint);

                record.add(farthestPoint);

                // Add the record to the result
                resLaunchPoints.add(record);
            }
        }

        // If there is only one resultant launch point return that
        if(resLaunchPoints.size() == 1) {
            return resLaunchPoints.get(0);
        }

        // Calculate the weighted score of the resultant launch points

        /* Few rules about different colored birds' behavior
         *
         * 1. Blue bird works best against Ice
         * 2. Yellow and white birds work best against wood
         * 3. Black bird works best against stone
         *
         * Code:
         * Red - 0, Blue - 1, Yellow - 2, White - 3, Black - 5
         * Ice - 0, Wood - 1, Stone - 2
         * Pig is Before - 0, After - 1
        */

        // Define weights
        int[][][] weights = new int[5][3][2];
        weights[0][0][0] = 6;
        weights[0][0][1] = 3;
        weights[0][1][0] = 6;
        weights[0][1][1] = 3;
        weights[0][2][0] = 6;
        weights[0][2][1] = 3;
        weights[1][0][0] = 9;
        weights[1][0][1] = 6;
        weights[1][1][0] = 6;
        weights[1][1][1] = 3;
        weights[1][2][0] = 6;
        weights[1][2][1] = 3;
        weights[2][0][0] = 6;
        weights[2][0][1] = 3;
        weights[2][1][0] = 9;
        weights[2][1][1] = 6;
        weights[2][2][0] = 6;
        weights[2][2][1] = 3;
        weights[3][0][0] = 6;
        weights[3][0][1] = 3;
        weights[3][1][0] = 9;
        weights[3][1][1] = 6;
        weights[3][2][0] = 6;
        weights[3][2][1] = 3;
        weights[4][0][0] = 6;
        weights[4][0][1] = 3;
        weights[4][1][0] = 6;
        weights[4][1][1] = 3;
        weights[4][2][0] = 9;
        weights[4][2][1] = 6;

        List<ABObject> blocks = vision.findBlocksMBR();
        int numberOfBlocks = blocks.size();

        int cnt = 0;

        ABType birdType = aRobot.getBirdTypeOnSling();
        int birdCode = 0;

        switch (birdType) {
            case RedBird:
                birdCode = 0;
                break;
            case BlueBird:
                birdCode = 1;
                break;
            case YellowBird:
                birdCode = 2;
                break;
            case WhiteBird:
                birdCode = 3;
                break;
            case BlackBird:
                birdCode = 4;
                break;
            default:
                break;
        }

        // Contains weighted scores of trajectories
        HashMap<Point, Integer> LPScores = new HashMap<Point, Integer>();

        // Iterate over all the blocks
        for(int index = 0; index < numberOfBlocks; index++) {

            ABObject block = blocks.get(index);

            ABType type = block.getType();

            ABType blockType = block.getType();
            int blockCode = 0;

            switch (blockType) {
                case Ice:
                    blockCode = 0;
                    break;
                case Wood:
                    blockCode = 1;
                    break;
                case Stone:
                    blockCode = 2;
                    break;
                default:
                    break;
            }

            if(type != ABType.Ice && type != ABType.Wood && type != ABType.Stone) {
                continue;
            }

            int blockX = block.x;
            int blockY = block.y;

            int blockWidth = block.width;
            int blockHeight = block.height;

            boolean onTrajectory = false;

            Point targetPig = null;

            Set<Point> LPToIncr = new HashSet<Point>();

            // Iterate over the width of the block
            for(int iterX = blockX; iterX <= blockX + blockWidth; iterX++) {

                // Iterate over the height of the block
                for(int iterY = blockY; iterY <= blockY + blockHeight; iterY++) {

                    Point targetPoint = new Point(iterX, iterY);

                    ArrayList<Point> newLaunchPoints = tp.estimateLaunchPoint(vision.findSlingshotMBR(), targetPoint);
                    int newLaunchPointsNo = newLaunchPoints.size();

                    // Iterate over all the possible launch points calculated
                    for(int iterLPts = 0; iterLPts < newLaunchPointsNo; iterLPts++) {
                        Point newLaunchPoint = newLaunchPoints.get(iterLPts);

                        // If block is on trajectory
                        if(resLaunchPointsList.contains(newLaunchPoint)) {
                            LPToIncr.add(newLaunchPoint);
                            onTrajectory = true;
                            targetPig = resTargetPointsList.get(resLaunchPointsList.indexOf(newLaunchPoint));
                        }
                    }
                }
            }

            // Simple little hack to prevent hanging of the shot
            cnt++;

            if(cnt == 15) {
                break;
            }

            if(onTrajectory) {

                int locationCode = 0;
                if(targetPig.x < targetPig.x) {
                    locationCode = 0;
                } else {
                    locationCode = 1;
                }

                int weight = weights[birdCode][blockCode][locationCode];

                for(Iterator<Point> it = LPToIncr.iterator(); it.hasNext(); ) {
                    Point lp = it.next();

                    if(LPScores.containsKey(lp)) {
                        LPScores.put(lp, LPScores.get(lp) + weight);
                    } else {
                        LPScores.put(lp, weight);
                    }
                }
            }
        }

        System.out.println(LPScores);

        // Get the launch points with maximum weighted score
        Map.Entry<Point, Integer> maxEntry = null;

        for(Map.Entry<Point, Integer> entry : LPScores.entrySet()) {
            if(maxEntry == null || entry.getValue().compareTo(maxEntry.getValue()) > 0) {
                maxEntry = entry;
            }
        }

        Point finalLaunchPoint = maxEntry.getKey();
        Point finalTargetPoint = resTargetPointsList.get(resLaunchPointsList.indexOf(finalLaunchPoint));

        ArrayList<Point> result = new ArrayList<Point>();
        result.add(finalLaunchPoint);
        result.add(finalTargetPoint);

        System.out.println(result);

        return result;
    }

    public GameState solve() {

        // Capture the image
        BufferedImage screenshot = ActionRobot.doScreenShot();

        int sceneWidth = screenshot.getWidth();
        int sceneHeight = screenshot.getHeight();

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

        // Get all the pigs
        List<ABObject> pigs = vision.findPigsMBR();

        GameState state = aRobot.getState();

        // If there is a sling, play else skip
        if(sling != null) {

            if(!pigs.isEmpty()) {

                Point releasePoint = null;
                Point targetPoint = null;

                Shot shot = new Shot();
                int dx, dy;

                {
                    ArrayList<Point> result = this.getReleasePoint(vision);

                    if(result.size() == 2) {
                        releasePoint = result.get(0);
                        targetPoint = result.get(1);
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

                        int tapTime = tp.getTapTime(sling, releasePoint, targetPoint, tapInterval);
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
