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