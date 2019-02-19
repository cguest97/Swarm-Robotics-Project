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
  void makeBody(float x, float y, float d, float f, float r, float ld, float ad) {
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