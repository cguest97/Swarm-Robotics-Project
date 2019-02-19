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