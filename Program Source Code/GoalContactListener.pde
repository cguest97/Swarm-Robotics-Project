/* Listen for when the object contacts the goal */
class GoalContactListener implements ContactListener{
  
  public boolean m_contact = false;

  void beginContact(Contact cp){
    Fixture f1 = cp.getFixtureA();
    Fixture f2 = cp.getFixtureB();
 
    Body b1 = f1.getBody();
    Body b2 = f2.getBody();
    
    Object o1 = b1.getUserData();
    Object o2 = b2.getUserData();
    
    if(o1.getClass() == Goal.class && o2.getClass() == PushableObj.class) m_contact = true;
    if(o2.getClass() == Goal.class && o1.getClass() == PushableObj.class) m_contact = true;
  }
  
  void endContact(Contact cp){}
  
  void preSolve(Contact cp, Manifold m){}
  
  void postSolve(Contact cp, ContactImpulse cpi){}
}