import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import shiffman.box2d.*; 
import org.jbox2d.common.*; 
import org.jbox2d.collision.shapes.*; 
import org.jbox2d.dynamics.*; 
import org.jbox2d.callbacks.RayCastCallback; 
import org.jbox2d.dynamics.contacts.*; 
import org.jbox2d.callbacks.ContactListener; 
import org.jbox2d.callbacks.ContactImpulse; 
import org.jbox2d.collision.Manifold; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Occlusion extends PApplet {











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
final static float ROBOT_DENSITY = 3.9f;
final static float ROBOT_FRICTION = 0.1f;
final static float ROBOT_RESTITUTION = 0.3f;
final static float ROBOT_MAX_SPEED = 5;
final static float ROBOT_LINEAR_DAMPING = 0.5f;
final static float ROBOT_ANGULAR_DAMPING = 10;


final static String OBJECT_SHAPE = "circle";
final static int    OBJECT_SIZE = 40;
final static float  OBJECT_DENSITY = 50;
final static float  OBJECT_FRICTION = 0.1f;
final static float  OBJECT_RESTITUTION = 0.3f;
final static float  OBJECT_LINEAR_DAMPING = 1;
final static float  OBJECT_ANGULAR_DAMPING = 1;

final static float GOAL_SIZE = 5;

boolean simDone = false;
long lEndTime;
long lStartTime;


/* Setup the arena */
public void setup(){  
  lStartTime = System.nanoTime();
  /* Setup world variables */
   
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
  
  crate = new PushableObj(120.5f, height/2, OBJECT_SHAPE, OBJECT_SIZE, OBJECT_DENSITY, OBJECT_FRICTION, OBJECT_RESTITUTION, OBJECT_LINEAR_DAMPING, OBJECT_ANGULAR_DAMPING, goal);
  
  // Add Robots to the Center of the Screen.
  for(int i = 0; i < ROBOT_NUMBER; i++){
    float rx = random((3*width)/5, (4*width)/5);
    float ry = random((height)/10, (8*height)/10);
    robots.add(new SwarmRobot(rx, ry, ROBOT_SIZE, ROBOT_MAX_SPEED, ROBOT_TORQUE, goal, ROBOT_DENSITY, ROBOT_FRICTION, ROBOT_RESTITUTION, ROBOT_LINEAR_DAMPING, ROBOT_ANGULAR_DAMPING, crate));
  }
}

/* At every step, do agent behaviours and update their location on the screen */
public void draw(){
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

public void mouseClicked(){
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
/* Class to create a solid static object that the pushable object must reach */
class Goal{  
  private Body body;
  private float size;
  
  public Goal(float x_, float y_, float size_){
    size = size_;
    makeBody(x_, y_);
  }
  
  public void display(){
    Vec2 position = box2d.getBodyPixelCoord(body);
    float angle = body.getAngle();
    
    pushMatrix();
    translate(position.x, position.y);
    rotate(-angle);
    
    fill(119,136,153);
    stroke(0);
    strokeWeight(1);
    ellipse(0, 0, size*2, size*2);
    popMatrix();
  }
  
  public Body getBody(){
    return body;
  }
  
  public float getSize(){
    return size;
  }
  
  public Vec2 getLocation(){
    return body.getWorldCenter();
  }
  
  public void makeBody(float x, float y){
    // Define a body
    BodyDef bd = new BodyDef();
    // Set its position
    bd.position = box2d.coordPixelsToWorld(x,y);
    bd.type = BodyType.STATIC;
    bd.userData = this;
    body = box2d.world.createBody(bd);
    
   /* Make the body a circle */
    CircleShape cs = new CircleShape();
    cs.m_radius = box2d.scalarPixelsToWorld(size);
    
    FixtureDef fd = new FixtureDef();
    fd.shape = cs;
    fd.friction = 0.01f;
    
    /* Attach the fixture to the body */
    body.createFixture(fd);
  
  }
  
}
/* Listen for when the object contacts the goal */
class GoalContactListener implements ContactListener{
  
  public boolean m_contact = false;

  public void beginContact(Contact cp){
    Fixture f1 = cp.getFixtureA();
    Fixture f2 = cp.getFixtureB();
 
    Body b1 = f1.getBody();
    Body b2 = f2.getBody();
    
    Object o1 = b1.getUserData();
    Object o2 = b2.getUserData();
    
    if(o1.getClass() == Goal.class && o2.getClass() == PushableObj.class) m_contact = true;
    if(o2.getClass() == Goal.class && o1.getClass() == PushableObj.class) m_contact = true;
  }
  
  public void endContact(Contact cp){}
  
  public void preSolve(Contact cp, Manifold m){}
  
  public void postSolve(Contact cp, ContactImpulse cpi){}
}
class GoalRayCast implements RayCastCallback{
  
  public boolean m_hit;
  
  /* Reset the variables for future use */
  public void init(){
    m_hit = false;
  }
  
  /* Report if the pushable object occludes the raycast */
  public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction){
    Body body = fixture.getBody();
    Object object = body.getUserData();
    
    if(object.getClass() == PushableObj.class){
      m_hit = true;
      return 0f;
    } else{
      return -1f;
    }
  }
}
class PushableObj{

  private Body body;
  private float size;
  private String shape;
  private PolygonShape objShape;
  private Goal goal;
  
  private float angularError = 0;
  private float lastAngle;
  
  private float minPathDistance;
  private float ourPathDistance;
  
  private Vec2 lastLocation;
  
  // Constructor
  public PushableObj(float x, float y, String s_, int size_, float density_, float friction_, float restitution_, float linDam_, float angDam_, Goal g_){
    shape = s_;
    goal = g_;
    size = size_;
    makeBody(x, y, density_, friction_, restitution_, linDam_, angDam_);
    lastAngle = body.getAngle();
    lastLocation = body.getWorldCenter().clone();
    calculateMinPathDistance();
  }
  
  public Vec2 getLocation(){
    return body.getWorldCenter();
  }
  
  public float getSize(){
    return size;
  }
  
  public float getAngularError(){
    return angularError;
  }
  
  public float getMinPathDistance(){
    return minPathDistance;
  }
  
  public float getOurPathDistance(){
    return ourPathDistance;
  }
  
  /* Calculate the most efficient path to the goal */
  public void calculateMinPathDistance(){
    Vec2 vecToGoal = goal.getLocation().sub(body.getWorldCenter());
    float minDistance = vecToGoal.length() - (box2d.scalarPixelsToWorld(goal.getSize() + size));
    minPathDistance = minDistance;
  }
  
  /* Calculate the distance of our current path */
  public void updateOurPathDistance(){
    Vec2 vecTravelled = body.getWorldCenter().sub(lastLocation);
    
    float lengthTravelled = vecTravelled.length();
    ourPathDistance = ourPathDistance + lengthTravelled;
    
    lastLocation = body.getWorldCenter().clone();
  }
  
  /* Calculate the unnecessary rotation applied to the object in this time step */
  public void applyAngularError(){
    float currentAngle = body.getAngle();
    float stepAngularError = Math.abs(currentAngle - lastAngle);
    angularError = angularError + stepAngularError;
    
    lastAngle = body.getAngle();
  }
   
  public void display(){
    Vec2 position = box2d.getBodyPixelCoord(body);
    float angle = body.getAngle();
    
    pushMatrix();
    translate(position.x, position.y);
    rotate(-angle);
    
    fill(119,136,153);
    stroke(0);
    strokeWeight(1);
    
    switch(shape){
      case "circle":
        ellipse(0, 0, size*2, size*2);
        break;
      case "square": 
        rectMode(CENTER);
        rect(0, 0, size*2, size*2);
        break;
      case "triangle": case "pentagon": case "scalene": case "rectangle":
        beginShape();
        for(int i = 0; i < objShape.getVertexCount(); i++){
          Vec2 v = box2d.vectorWorldToPixels(objShape.getVertex(i));
          vertex(v.x,v.y);
        }
        endShape(CLOSE);
        break;
    }
    line(0, 0, 0, -size);
    noFill();
    ellipse(0, 0, size * 6, size * 6);
    popMatrix();
  }
  
  /* Construct a circular body shape */
  public Shape makeCircle(){
    CircleShape circle = new CircleShape();
    circle.m_radius = box2d.scalarPixelsToWorld(size);
    
    return circle;
  }
  
   /* Construct a triangular body shape */
  public Shape makeTriangle(){
    Vec2[] vertices = new Vec2[3];
    vertices[0] = box2d.vectorPixelsToWorld(new Vec2(0, size));
    vertices[1] = box2d.vectorPixelsToWorld(new Vec2(size, -size));
    vertices[2] = box2d.vectorPixelsToWorld(new Vec2(-size, -size));
    PolygonShape triangle = new PolygonShape();
    triangle.set(vertices, vertices.length);
    objShape = triangle;
    return triangle;
  }
  
   /* Construct a square body shape */
  public Shape makeSquare(){
    PolygonShape square = new PolygonShape();
    float shapeWidth = box2d.scalarPixelsToWorld(size);
    float shapeHeight = box2d.scalarPixelsToWorld(size);
    
    square.setAsBox(shapeWidth, shapeHeight);
    return square;
  }
  
  /* Construct a pentagonal body shape */
  public Shape makePentagon(){
    Vec2[] vertices = new Vec2[5];
    vertices[0] = box2d.vectorPixelsToWorld(new Vec2(0, size));
    vertices[1] = box2d.vectorPixelsToWorld(new Vec2(size, size/3));
    vertices[2] = box2d.vectorPixelsToWorld(new Vec2((2*size)/3, -size));
    vertices[3] = box2d.vectorPixelsToWorld(new Vec2((-2*size)/3, -size));
    vertices[4] = box2d.vectorPixelsToWorld(new Vec2(-size, size/3));
    PolygonShape pentagon = new PolygonShape();
    pentagon.set(vertices, vertices.length);
    objShape = pentagon;
    return pentagon;
  }
  
  /* Construct a scalene triangle body shape */
  public Shape makeScalene(){
    Vec2[] vertices = new Vec2[3];
    vertices[0] = box2d.vectorPixelsToWorld(new Vec2(-size, -size));
    vertices[1] = box2d.vectorPixelsToWorld(new Vec2(size, -size));
    vertices[2] = box2d.vectorPixelsToWorld(new Vec2(size, size));
    
    PolygonShape scalene = new PolygonShape();
    scalene.set(vertices, vertices.length);
    objShape = scalene;
    return scalene;
  }
  
  /* Construct a square body shape */
  public Shape makeRectangle(){
    Vec2[] vertices = new Vec2[4];
    vertices[0] = box2d.vectorPixelsToWorld(new Vec2(-size / 4, size));
    vertices[1] = box2d.vectorPixelsToWorld(new Vec2(-size / 4, -size));
    vertices[2] = box2d.vectorPixelsToWorld(new Vec2(size / 4, -size));
    vertices[3] = box2d.vectorPixelsToWorld(new Vec2(size / 4, size));
    
    PolygonShape rect = new PolygonShape();
    rect.set(vertices, vertices.length);
    objShape = rect;
    return rect;
  }
 
  
  /* This Method Creates The Body Of The Object For Physics To Act On */
  public void makeBody(float x, float y, float d, float f, float r, float ld, float ad) {
    // Define a body
    BodyDef bd = new BodyDef();
    // Set its position
    bd.position = box2d.coordPixelsToWorld(x,y);
    bd.type = BodyType.DYNAMIC;
    bd.userData = this;
    bd.linearDamping = ld;
    bd.angularDamping = ad;
    body = box2d.world.createBody(bd);
    
    Shape bodyShape = makeCircle();
    
    /* Create the shape based on user selection */
    switch(shape){
      case "circle": 
        break;
      case "square": 
        bodyShape = makeSquare();
        break;
      case "triangle":
        bodyShape = makeTriangle();
        break;
      case "pentagon":
        bodyShape = makePentagon();
        break;
      case "scalene":
        bodyShape = makeScalene();
        break;
      case "rectangle":
        bodyShape = makeRectangle();
        break;
    }
    
    FixtureDef fd = new FixtureDef();
    fd.shape = bodyShape;
    // Parameters that affect physics
    fd.density = d;
    fd.friction = f;
    fd.restitution = r;
    
    // Attach fixture to body
    body.createFixture(fd);
    
  }
}
class RobotRayCast implements RayCastCallback{
  
  public boolean m_hit;
  public Vec2 m_point, m_normal;
  public int m_num;
  
  public Object m_object;
  
  /* Reset the variables for future use */
  public void init(){
    m_hit = false;
    m_point = new Vec2(0, 0);
    m_normal = new Vec2(0, 0);
    m_num = 0;
    m_object = null;
  }
  
  /* Report any objects seen by the Raycast */
  public float reportFixture(Fixture fixture, Vec2 point, Vec2 normal, float fraction){
    Body body = fixture.getBody();
    Object object = body.getUserData();
    m_hit = true;
    m_point = point;
    m_normal = normal;
    
    if( object.getClass() == Wall.class ){
      m_num = 1;
      return 0f;
    } else if( object.getClass() == SwarmRobot.class ){
      m_num = 2;
      return 0f;  
    }else if( object.getClass() == Goal.class){
      m_num = 3;
      return 0f;  
    } else if( object.getClass() == PushableObj.class ){
      m_object = body.getUserData();
      m_num = 4;
      return 0f;
    }
    return -1f;
  }

}
class SwarmRobot{
  private Vec2 location;
  
  /* Robot Properties */
  private float radius;
  private float maxspeed;
  private float maxTorque;
  private float sightRange = 4;
  
  /* Setup raycasts to be used for line of sight */
  private RobotRayCast leftRay   = new RobotRayCast();
  private RobotRayCast middleRay = new RobotRayCast();
  private RobotRayCast rightRay  = new RobotRayCast();
  private GoalRayCast goalRay = new GoalRayCast();
  
  private static final float DEGREES_TO_RADIANS = (float)(Math.PI/180);
  
  /* Bodies Robot is able to detect */
  private Body body;
  private PushableObj crate;
  private Goal goal;
  
  private int col;
  private boolean objectFound = false;
  private boolean headTowardsObject = false;
  
  /* Class Constructor */
  public SwarmRobot(float x, float y, float r, float ms, float mt, Goal goal_, float density_, float friction_, float restitution_, float linDam_, float angDam_, PushableObj cr){
    location = new Vec2(x, y);
    crate = cr;
    goal = goal_;
    maxTorque = mt;
    col = (200);
    radius = r;
    maxspeed = ms;
    makeBody(density_, friction_, restitution_, linDam_, angDam_);
    location = box2d.getBodyPixelCoord(body);
  }
  
  /* Check if objects ahead via raycasting */
  public int checkAhead(){
    Vec2 currentPosition = body.getPosition();
    
    /* Calculate the end point of the raycasts */
    float leftX = MathUtils.cos(body.getAngle() - 0.3f) * sightRange;
    float leftY = MathUtils.sin(body.getAngle() - 0.3f) * sightRange;
    float middleX = MathUtils.cos(body.getAngle()) * sightRange;
    float middleY = MathUtils.sin(body.getAngle()) * sightRange;
    float rightX = MathUtils.cos(body.getAngle() + 0.3f) * sightRange;
    float rightY = MathUtils.sin(body.getAngle() + 0.3f) * sightRange;
    
    /* Setup the lines the raycasts will use */
    Vec2 leftLine = new Vec2(currentPosition.x + leftX, currentPosition.y + leftY);
    Vec2 middleLine = new Vec2(currentPosition.x + middleX, currentPosition.y + middleY);
    Vec2 rightLine = new Vec2(currentPosition.x + rightX, currentPosition.y + rightY);
    
    /* Initialize Raycasts to refresh variables from previous runs */
    rightRay.init();
    middleRay.init();
    leftRay.init(); 
    
    /* Do the Raycast */
    box2d.world.raycast(leftRay, currentPosition, leftLine);
    box2d.world.raycast(middleRay, currentPosition, middleLine);
    box2d.world.raycast(rightRay, currentPosition, rightLine);
    
    /* Draw a visual representation of the raycasts */
    Vec2 startPoint = box2d.coordWorldToPixels(currentPosition);
    Vec2 leftPoint = box2d.coordWorldToPixels(leftLine);
    Vec2 middlePoint = box2d.coordWorldToPixels(middleLine);
    Vec2 rightPoint = box2d.coordWorldToPixels(rightLine);
    
    line(startPoint.x, startPoint.y, leftPoint.x, leftPoint.y);
    line(startPoint.x, startPoint.y, middlePoint.x, middlePoint.y);
    line(startPoint.x, startPoint.y, rightPoint.x, rightPoint.y);
    
    /* Return 0 if no object ahead, -1 if the pushable object is ahead, or the object ID */
    if( leftRay.m_hit || middleRay.m_hit || rightRay.m_hit) {
      if( leftRay.m_num == 4 || middleRay.m_num == 4  || rightRay.m_num == 4 ){ return -1; } 
      else if( leftRay.m_num == 1 || middleRay.m_num == 1  || rightRay.m_num == 1 ){ return 1; } 
      else if( leftRay.m_num == 2 || middleRay.m_num == 2  || rightRay.m_num == 2 ){ return 2; }
      else if( leftRay.m_num == 3 || middleRay.m_num == 3  || rightRay.m_num == 3 ){ return 3; }
    }  
    return 0;
  }
  
  /* Decide which agent behaviour to do */
  public void doAction(){
    
    int action = checkAhead();
    float random = random(0, 100);
    if(random > 97) headTowardsObject = true;
    
    /* Do a raycast to see if goal is occluded */
    goalRay.init();
    box2d.world.raycast(goalRay, body.getPosition(), goal.getLocation());
    
    /* Check distance from pushable object */
    Vec2 toCrate = crate.getLocation().sub(body.getWorldCenter());
    
    /* If goal occluded head towards it */
    if(goalRay.m_hit && headTowardsObject){ headTowardsGoal(); col = color(0, 255, 0);}
    /* Else if we have detected the object but are very far away, approach the object */ 
    else if(action == -1 && toCrate.length() > box2d.scalarPixelsToWorld(crate.getSize() * 4)){ approachObject(); objectFound = true; col = color(255, 255, 0); }
    /* Else if we have detected the object and are close to it we begin circling it */
    else if(action == -1 || objectFound){ foundObject(); objectFound = true; col = color(255, 255, 0); }
    /* Else if there is an object ahead avoid it */
    else if(action == 1 || action == 2 || action == 3){ explore(true); col = color(255, 0, 0);}
    /* Else if there is no object ahead explore the arena */
    else if(action == 0){ explore(false); col = color(255, 0, 0);}
    
    /* If too far from the object consider it lost */
    if(toCrate.length() > box2d.scalarPixelsToWorld(crate.getSize() * 5)) objectFound = false;
    if(!goalRay.m_hit) headTowardsObject = false;
  }
  
  /* Circle object after it's been found */
  public void foundObject(){
    int action = checkAhead();
    Vec2 currentPosition = body.getPosition();
    float middleX = MathUtils.cos(body.getAngle()) * sightRange / 2;
    float middleY = MathUtils.sin(body.getAngle()) * sightRange / 2;
    Vec2 middleLine = new Vec2(currentPosition.x + middleX, currentPosition.y + middleY);
    Vec2 toCrate = crate.getLocation().sub(middleLine);
    
    /* If too near or too far away from object turn in required direction, otherwise go forward */
    if(action == -1){body.applyTorque(-50);}
    else if(toCrate.length() > box2d.scalarPixelsToWorld(crate.getSize() * 3) || action == 1 || action == 3){
      body.applyTorque(50);
    } else{ goForward(); }
  }
  
  /* Randomly walk around the arena, if obstacle detected avoid it */
  public void explore(boolean objectAhead){
    float chosenAngle = 0;
    if( objectAhead ){
      chosenAngle = maxTorque * 2;
    } else{
      chosenAngle = random(-maxTorque, maxTorque);
    }   
    body.applyTorque(chosenAngle);
    goForward();   
  }
  
  /* Turn to the desired angle */
  public void turnToAngle(float desired){
    float nextAngle = body.getAngle() + body.getAngularVelocity() / 20;
    float totalRotation = desired - nextAngle;
    while(totalRotation < -180 * DEGREES_TO_RADIANS) totalRotation += 360 * DEGREES_TO_RADIANS;
    while(totalRotation >  180 * DEGREES_TO_RADIANS) totalRotation -= 360 * DEGREES_TO_RADIANS;
    
    float desiredAngularVelocity = totalRotation * 20;
    float torque = body.getInertia() * desiredAngularVelocity / (1/20.0f);
    body.applyTorque( torque );
  }
  
  /* Travel towards the goal */
  public void headTowardsGoal(){
    Vec2 toGoal = goal.getLocation().sub(body.getWorldCenter());
    float angle = (float)Math.atan2(toGoal.y, toGoal.x);
    turnToAngle(angle);
    goForward();
    
  }
  
  /* Face object and travel towards it */
  public void approachObject(){
    Vec2 toObject = crate.getLocation().sub(body.getWorldCenter());
    toObject = crate.getLocation().sub(body.getWorldCenter());
    float angle = (float)Math.atan2(toObject.y, toObject.x);
    turnToAngle(angle);
    goForward();
    
  }
  
  /* Method to travel in direction robot is facing */
  public void goForward(){
    float velX = MathUtils.cos(body.getAngle()) * maxspeed;
    float velY = MathUtils.sin(body.getAngle()) * maxspeed;
    Vec2 newVelocity = new Vec2(velX, velY);
    newVelocity.normalize();
    newVelocity.mulLocal(maxspeed);
    
    body.setLinearVelocity(newVelocity);    
  }
    
  public void setColor(int c){
    col = c;
  }
    
  /* Method to display the object on the screen */
  public void display(){
    location = box2d.getBodyPixelCoord(body);
    float angle = body.getAngle();
    
    pushMatrix();
    translate(location.x, location.y);
    rotate(-angle);    
   
    fill(col);
    stroke(0);
    strokeWeight(1);
    ellipse(0, 0,radius*2,radius*2);
    line(0, 0, radius, 0);
    
    strokeWeight(1);    
    popMatrix();
  }
  
  /* Setup the body for the physics engine to use */
  public void makeBody(float d, float f, float r, float ld, float ad) {
    /* Setup the body */
    BodyDef bd = new BodyDef();
    bd.position = box2d.coordPixelsToWorld(location.x, location.y);
    bd.type = BodyType.DYNAMIC;
    bd.userData = this;
    bd.linearDamping = ld;
    bd.angularDamping = ad;
    body = box2d.world.createBody(bd);

    /* Make the body a circle */
    CircleShape cs = new CircleShape();
    cs.m_radius = box2d.scalarPixelsToWorld(radius);
    
    FixtureDef fd = new FixtureDef();
    fd.shape = cs;
    fd.density = d;
    fd.friction = f;
    fd.restitution = r;
    
    /* Attach the fixture to the body */
    body.createFixture(fd);
  }
}
  
/* Class to create a solid static object that encloses the arena */
class Wall{

  private Body body;
  private float x, y, w, h;
  
  public Wall(float x_, float y_, float w_, float h_){
    x = x_;
    y = y_;
    w = w_;
    h = h_;
    
    BodyDef bd = new BodyDef();
    bd.position.set(box2d.coordPixelsToWorld(x, y));
    bd.type = BodyType.STATIC;
    bd.userData = this;
    body = box2d.createBody(bd);
    
    float box2dW = box2d.scalarPixelsToWorld(w/2);
    float box2dH = box2d.scalarPixelsToWorld(h/2);
    PolygonShape ps = new PolygonShape();
    ps.setAsBox(box2dW, box2dH);
    
    body.createFixture(ps, 1);
 
  }
  
  public void display(){
    fill(0);
    stroke(0);
    rectMode(CENTER);
    rect(x,y,w,h);
  }
}
  public void settings() {  size(500, 300); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Occlusion" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
