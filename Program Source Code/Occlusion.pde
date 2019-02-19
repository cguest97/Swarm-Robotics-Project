import shiffman.box2d.*;
import org.jbox2d.common.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.callbacks.RayCastCallback;
import org.jbox2d.dynamics.contacts.*;
import org.jbox2d.callbacks.ContactListener;
import org.jbox2d.callbacks.ContactImpulse;
import org.jbox2d.collision.Manifold;

//System.out.println((crate.getAngularError() * 180) / PI);

//Variable containing the World.
Box2DProcessing box2d;
  
// Objects in the World.
ArrayList<SwarmRobot> robots;
ArrayList<Wall> walls;
PushableObj crate;
Goal goal;
GoalContactListener myListener;


/* Variables for users to change */
final static int   ROBOT_NUMBER = 10;
final static int   ROBOT_SIZE = 7;
final static float ROBOT_TORQUE = 50; 
final static float ROBOT_DENSITY = 3.9;
final static float ROBOT_FRICTION = 0.1;
final static float ROBOT_RESTITUTION = 0.3;
final static float ROBOT_MAX_SPEED = 5;
final static float ROBOT_LINEAR_DAMPING = 0.5;
final static float ROBOT_ANGULAR_DAMPING = 10;


final static String OBJECT_SHAPE = "circle";
final static int    OBJECT_SIZE = 40;
final static float  OBJECT_DENSITY = 50;
final static float  OBJECT_FRICTION = 0.1;
final static float  OBJECT_RESTITUTION = 0.3;
final static float  OBJECT_LINEAR_DAMPING = 1;
final static float  OBJECT_ANGULAR_DAMPING = 1;

final static float GOAL_SIZE = 5;

boolean simDone = false;
long lEndTime;
long lStartTime;


/* Setup the arena */
void setup(){  
  lStartTime = System.nanoTime();
  /* Setup world variables */
  size(500, 300); 
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  box2d.setGravity(0, 0);
  box2d.listenForCollisions();
  myListener = new GoalContactListener();
  box2d.world.setContactListener(myListener);
   
  walls = new ArrayList<Wall>();
  robots = new ArrayList<SwarmRobot>();
  goal = new Goal(width - 75, height/2, GOAL_SIZE); //(width * 4)/5
  
  // Add Static Walls to Contain Elements in World.
  walls.add(new Wall(width/2, 0       , width, 5));
  walls.add(new Wall(width/2, height  , width, 5));
  walls.add(new Wall(0      , height/2, 5    , height));
  walls.add(new Wall(width  , height/2, 5    , height));
  
  crate = new PushableObj(120.5, height/2, OBJECT_SHAPE, OBJECT_SIZE, OBJECT_DENSITY, OBJECT_FRICTION, OBJECT_RESTITUTION, OBJECT_LINEAR_DAMPING, OBJECT_ANGULAR_DAMPING, goal);
  
  // Add Robots to the Center of the Screen.
  for(int i = 0; i < ROBOT_NUMBER; i++){
    float rx = random((3*width)/5, (4*width)/5);
    float ry = random((height)/10, (8*height)/10);
    robots.add(new SwarmRobot(rx, ry, ROBOT_SIZE, ROBOT_MAX_SPEED, ROBOT_TORQUE, goal, ROBOT_DENSITY, ROBOT_FRICTION, ROBOT_RESTITUTION, ROBOT_LINEAR_DAMPING, ROBOT_ANGULAR_DAMPING, crate));
  }
}

/* At every step, do agent behaviours and update their location on the screen */
void draw(){
  if(!myListener.m_contact){
    box2d.step();
    background(255);
  
    goal.display();
    crate.display();
  
    for (Wall w: walls) w.display();
  
    for (SwarmRobot r: robots) {
      r.doAction();
      r.display();
    }

    crate.applyAngularError();
    crate.updateOurPathDistance();
    lEndTime = System.nanoTime();
  } else{
    simDone = true;
  }
}

void mouseClicked(){
  if(simDone){
    float fpathEff = crate.getMinPathDistance() / crate.getOurPathDistance();
    if(fpathEff > 1) fpathEff = 1;
    
    String angErr = "Angular Error: " + ((crate.getAngularError() * 180) / PI);
    String pathEff = "Path Efficiency: " + (fpathEff * 100) + "%";
    String timeElapsed = "Elapsed time in milliseconds: " + (lEndTime - lStartTime) / 1000000;
    
    background(255);
    fill(50);
    text(angErr, 10, 10);
    text(pathEff, 10, 50);
    text(timeElapsed, 10, 90);
  
  }
}