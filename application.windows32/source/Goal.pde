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
    fd.friction = 0.01;
    
    /* Attach the fixture to the body */
    body.createFixture(fd);
  
  }
  
}