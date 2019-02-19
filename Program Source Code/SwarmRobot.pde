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
  
  private color col;
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
    float leftX = MathUtils.cos(body.getAngle() - 0.3) * sightRange;
    float leftY = MathUtils.sin(body.getAngle() - 0.3) * sightRange;
    float middleX = MathUtils.cos(body.getAngle()) * sightRange;
    float middleY = MathUtils.sin(body.getAngle()) * sightRange;
    float rightX = MathUtils.cos(body.getAngle() + 0.3) * sightRange;
    float rightY = MathUtils.sin(body.getAngle() + 0.3) * sightRange;
    
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
    float torque = body.getInertia() * desiredAngularVelocity / (1/20.0);
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
    
  public void setColor(color c){
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
  