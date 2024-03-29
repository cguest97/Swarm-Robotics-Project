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