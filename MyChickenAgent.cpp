#include "MyChickenAgent.h"
#include "MyGame.h"
#include "MyDragonAgent.h"

using namespace mathtool;

namespace GMUCS425
{
  //DO NOT CHANGE THIS FUNCTION
  //this function updates this->force (from MyPartcile)
  //implement: 3 flocking force, 1 obstacle avoiding force
  //           1 following force if this chicken is a leader
  void MyChickenAgent::compute_force()
  {
    //DONT CHANGE ANYTHING HERE
    Vector2d boid_force=compute_flocking_force();
    Vector2d obst_force=compute_obstacle_avoiding_force();

    this->force = boid_force+obst_force;

    if(this->leader)
    {
      Vector2d follow_force=compute_following_force();
      this->force += follow_force;
    }

    //add drag force
    this->force += this->vel*(-0.1);
    //DONT CHANGE ANYTHING HERE
  }

  Vector2d MyChickenAgent::distance(MyChickenAgent* chicken_one, MyChickenAgent* chicken_two)
  {
      float x = chicken_two->pos[0] - chicken_one->pos[0];
      float y = chicken_two->pos[1] - chicken_one->pos[1];
      return Vector2d(x,y);
  }

  //compute each force
  Vector2d MyChickenAgent::compute_flocking_force()
  {
    Vector2d force_sep, force_align, force_coherent;
    force_sep = force_align = force_coherent = Vector2d(0,0);

    list<MyChickenAgent*> neighbors;
    this->get_neighbors(neighbors);
    int nsize=neighbors.size();
    if(nsize==0) return Vector2d();

    //1. compute separation force
    //https://www.youtube.com/watch?v=mZ1zCo-acuQ&feature=youtu.be @ 8:40
    // force_sep = sum(weight * (vector_to_x/magnitude_to_x^2))
    // weight  = 1 / magnitude_to_xi^2
    float total_weight = 0;
    for (auto neighbor = neighbors.begin(); neighbor != neighbors.end(); neighbor++)
    {
        if ((*neighbor) == this) continue;
        Vector2d chicken_distance = distance(this, (*neighbor));
        float w = 1 / chicken_distance.normsqr();
        w = 750;
        force_sep += (chicken_distance / chicken_distance.normsqr()) * w;
    }
    force_sep *= -1;
    
    //2. compute alignment force
    //https://www.youtube.com/watch?v=mZ1zCo-acuQ&feature=youtu.be @ 14:57
    // force_align = sum(weight * velocity_of_neighbor) / sum(weight)
    // weight  = 1 / magnitude_of_vel_of_xi^2
    total_weight = 0;
    for (auto neighbor = neighbors.begin(); neighbor != neighbors.end(); neighbor++)
    {
        Vector2d chicken_distance = distance((*neighbor), this);
        float w = 1 / chicken_distance.normsqr();
        w = 5;
        force_align += (*neighbor)->vel * w;
        total_weight += w;
    }
    force_align = force_align / total_weight;

    //3. compute coherent force
    //https://www.youtube.com/watch?v=mZ1zCo-acuQ&feature=youtu.be @ 16:30
    // force_coherent = x_cm - x
    // x_cm = sum(weight * pos_of_neighbor) / sum(weight)
    // weight = mass
    Vector2d x_cm = Vector2d(0, 0);
    total_weight = 0;
    for (auto neighbor = neighbors.begin(); neighbor != neighbors.end(); neighbor++)
    {
        if ((*neighbor) == this) continue;
        Vector2d chicken_distance = distance((*neighbor), this);
        float w =  1 / chicken_distance.normsqr();
        x_cm += Vector2d((*neighbor)->pos[0], (*neighbor)->pos[1]) * (*neighbor)->mass;
        total_weight += (*neighbor)->mass;
    }
    x_cm = x_cm / total_weight;
    force_coherent = x_cm - Vector2d(this->pos[0], this->pos[1]);

    //add the forces together
    return force_sep*k_sep + force_align*k_align + force_coherent*k_coherent;
  }

  Vector2d MyChickenAgent::compute_obstacle_avoiding_force()
  {
    Vector2d force;
    Point2d predict_pos = this->pos + this->vel.normalize()*this->view_radius;

    //check if there is collision between this->pos and predict_pos
    //if so, compute the point of collision.
    HalfPlane hp;
    if( checkCollision(this->pos, predict_pos, hp) )
    {
        //TODO:
        //use hp to determine a force that pushes this chicken
        //away from the obstacle
        //remember to scale the force by k_obst
        //https://www.youtube.com/watch?v=7s5O67QDlMs&feature=youtu.be @ 13:30, formula @16:43
        // force = k * (w * n)
        Vector2d perp_vec = hp.n * Vector2d(this->pos - hp.p); // Should only have either the x or the y, going to be used for the weight
        float w = 1 / perp_vec.normsqr();
        force = (hp.n * w) * k_obst;
    }
    //return Vector2d(0, 0);
    return force;
  }

  //DO NOT CHANGE THIS FUNCTION:
  //detect collision between line segment ab and the static obstacles
  //if so, return true and store the point of collision in hp
  bool MyChickenAgent::checkCollision(const Point2d& a, const Point2d& b, HalfPlane& hp)
  {
    //DONT CHANGE ANYTHING HERE
    const std::list<MyAgent * > &  agents=getMyGame()->getSceneManager()->get_active_scene()->get_agents();
    mathtool::Box2d box;
    for(auto obst : agents)
    {
      if( obst->is_movable() ) continue;
      box.x=obst->getX();
      box.y=obst->getY();
      box.width=obst->getSprite()->getWidth(obst->getScale());
      box.height=obst->getSprite()->getHeight(obst->getScale());

      //check if ab intersects box
      if(checkCollision(box, a, b, hp)) return true;
    }//end obst

    //bounding box of the entire level, prevent the chicken going out of screen
    box.x=0; box.y=-5;
    box.width=getMyGame()->getScreenWidth(); box.height=5;
    if(checkCollision(box, a, b, hp)) return true; //TOP

    box.x=-5; box.y=0;
    box.width=5; box.height=getMyGame()->getScreenHeight();
    if(checkCollision(box, a, b, hp)) return true; //LEFT

    box.x=0; box.y=getMyGame()->getScreenHeight();
    box.width=getMyGame()->getScreenWidth(); box.height=5;
    if(checkCollision(box, a, b, hp)) return true; //BOTTOM

    box.x=getMyGame()->getScreenWidth(); box.y=0;
    box.width=5; box.height=getMyGame()->getScreenHeight();
    if(checkCollision(box, a, b, hp)) return true; //RIGHT

    return false;
    //DONT CHANGE ANYTHING HERE
  }


  bool MyChickenAgent::checkCollision(const mathtool::Box2d& box,
                                      const Point2d& a, const Point2d& b,
                                      HalfPlane& hp)
  {
    //TODO: WORK ON THIS FUNCTION
    //https://www.youtube.com/watch?v=7s5O67QDlMs&feature=youtu.be @ 6:00
    Point2d v1 = Point2d(box.x, box.y); //top left corner
    Point2d v2 = Point2d(box.x, box.y + box.height); //bottom left corner
    Point2d v3 = Point2d(box.x + box.width, box.y + box.height); //bottom right corner
    Point2d v4 = Point2d(box.x + box.width, box.y); //top right corner
    
    Vector2d result;

    Vector2d L12_normal = Vector2d(-1, 0);
    Vector2d Va = Vector2d(a - v1);
    Vector2d Vb = Vector2d(b - v2);
    if ((Va * L12_normal) * (Vb * L12_normal) < 0)
    {
        // r1 = V1 + t ( V2 - V1 )
        // r2 =  a + s (  b -  a )
        
        //       --                      --
        // r1 = | V1.x + t ( V2.x - V1.x ) |
        //      | V1.y + t ( V2.y - V1.y ) |
        //       --                      --
        //       --                   --
        // r2 = | a.x + s ( b.x - a.x ) |
        //      | a.y + s ( b.y - a.y ) |
        //       --                   --

        // V1.x + t ( V2.x - V1.x ) = a.x + s ( b.x - a.x )
        // V1.y + t ( V2.y - V1.y ) = a.y + s ( b.y - a.y )
        
        // V1.x - a.x + t ( V2.x - V1.x ) = s ( b.x - a.x )
        // V1.y - a.y + s ( V2.y - V1.y ) = s ( b.y - a.y )

        //  --        --       --             --
        // | V1.x - a.x |     | ( V2.x - V1.x ) |
        //  --        --       --             --
        // -------------- + t ------------------- = s
        // ( b.x - a.x )         ( b.x - a.x )
        //  --        --       --             --
        // | V1.y - a.y |     | ( V2.y - V1.y ) |
        //  --        --       --             --
        // -------------- + t ------------------- = s
        // ( b.y - a.y )         ( b.y - a.y )
        // subtract the two to get t

        // ( V1.x - a.x )   ( V1.y - a.y )       ( V2.x - V1.x )         ( V2.y - V1.y ) 
        // -------------- - -------------- + t ------------------- - t ------------------- = 0
        // ( b.x - a.x )    ( b.y - a.y )         ( b.x - a.x )           ( b.y - a.y )

        //     ( V2.x - V1.x )         ( V2.y - V1.y )     ( V1.x - a.x )   ( V1.y - a.y )
        // t ------------------- - t ------------------- = -------------- - --------------
        //      ( b.x - a.x )           ( b.y - a.y )      ( b.x - a.x )    ( b.y - a.y )

        //      ( V1.x - a.x )   ( V1.y - a.y )
        //      -------------- - --------------
        //      ( b.x - a.x )    ( b.y - a.y )
        // ----------------------------------------- = t
        //   ( V2.x - V1.x )       ( V2.y - V1.y )
        // ------------------- - -------------------
        //    ( b.x - a.x )         ( b.y - a.y )  

        // Using the t that has been found to get s
        // ( V1.x - a.x )     ( V2.x - V1.x )
        // -------------- + t --------------- = s
        // ( b.x - a.x )       ( b.x - a.x )

        float t = ((v1[0] - a[0]) / (b[0] - a[0]) - (v1[1] - a[1]) / (b[1] - a[1])) / ((v2[0] - v1[0]) / (b[0] - a[0]) - (v2[1] - v1[1]) / (b[1] - a[1]));
        float s = ((v1[0] - a[0]) / (b[0] - a[0])) + t * ((v2[0] - v1[0]) / (b[0] - a[0]));
        
        if(0 <= t && t <= 1 && 0 <= s && s <= 1)
        {
            hp.n = L12_normal;
            hp.p = Point2d(v1[0] + t * (v2[0] - v1[0]), v1[1] + t * (v2[1] - v1[1]));
            return true;
        }
    }

    Vector2d L23_normal = Vector2d(0, 1);
    Va = Vector2d(a - v2);
    Vb = Vector2d(b - v3);
    if ((Va * L23_normal) * (Vb * L23_normal) < 0)
    {
        float t = ((v2[0] - a[0]) / (b[0] - a[0]) - (v2[1] - a[1]) / (b[1] - a[1])) / ((v3[0] - v2[0]) / (b[0] - a[0]) - (v3[1] - v2[1]) / (b[1] - a[1]));
        float s = ((v2[0] - a[0]) / (b[0] - a[0])) + t * ((v3[0] - v2[0]) / (b[0] - a[0]));

        if (0 <= t && t <= 1 && 0 <= s && s <= 1)
        {
            hp.n = L23_normal;
            hp.p = Point2d(v2[0] + t * (v3[0] - v2[0]), v2[1] + t * (v3[1] - v2[1]));
            return true;
        }
    }

    Vector2d L34_normal = Vector2d(1, 0);
    Va = Vector2d(a - v3);
    Vb = Vector2d(b - v4);
    if ((Va * L34_normal) * (Vb * L34_normal) < 0)
    {
        float t = ((v3[0] - a[0]) / (b[0] - a[0]) - (v3[1] - a[1]) / (b[1] - a[1])) / ((v4[0] - v3[0]) / (b[0] - a[0]) - (v4[1] - v3[1]) / (b[1] - a[1]));
        float s = ((v3[0] - a[0]) / (b[0] - a[0])) + t * ((v4[0] - v3[0]) / (b[0] - a[0]));

        if (0 <= t && t <= 1 && 0 <= s && s <= 1)
        {
            hp.n = L34_normal;
            hp.p = Point2d(v3[0] + t * (v4[0] - v3[0]), v3[1] + t * (v4[1] - v3[1]));
            return true;
        }
    }

    Vector2d L41_normal = Vector2d(0, -1);
    Va = Vector2d(a - v4);
    Vb = Vector2d(b - v1);
    if ((Va * L41_normal) * (Vb * L41_normal) < 0)
    {
        float t = ((v4[0] - a[0]) / (b[0] - a[0]) - (v4[1] - a[1]) / (b[1] - a[1])) / ((v1[0] - v4[0]) / (b[0] - a[0]) - (v1[1] - v4[1]) / (b[1] - a[1]));
        float s = ((v4[0] - a[0]) / (b[0] - a[0])) + t * ((v1[0] - v4[0]) / (b[0] - a[0]));

        if (0 <= t && t <= 1 && 0 <= s && s <= 1)
        {
            hp.n = L41_normal;
            hp.p = Point2d(v4[0] + t * (v1[0] - v4[0]), v4[1] + t * (v1[1] - v4[1]));
            return true;
        }
    }

    return false;
  }

  //if this chicken is a leader, find a force to drag the chicken to the dragon
  Vector2d MyChickenAgent::compute_following_force()
  {
    //https://www.youtube.com/watch?v=rP04OAGd-Nk&feature=youtu.be @ 1:23
    // F = (l / |l|) * -(ks * (|l| - r) + kd * (v * (l / |l|)))   
    MyDragonAgent * dragon = get_dragon();
    assert(dragon);
    Vector2d force;
    //NOTE: DON"T USE MOTION PLANNER;
    //      Simply use the vector from this chiken to dragon to determine the force
    //      Remember to use k_follow to scale the force
    Vector2d chicken_distance = Vector2d(dragon->getX() - this->pos[0], dragon->getY() - this->pos[1]); // This is l
    double mag_l = chicken_distance.norm(); // This is |l|
    Vector2d v = vel;
    double ks = 0.03;
    double kd = 0.025;
    double r = 20;
    if (chicken_distance.norm() > 100) kd = 0;
    force = (chicken_distance / mag_l)  * (ks * (mag_l - r) + kd * (v * (chicken_distance / mag_l)));
    return force * k_follow;
  }
  
  //DO NOT CHANGE ANYTHING BELOW

  void MyChickenAgent::display()
  {
    if(!this->visible) return; //not visible...
    //setup positions and ask sprite to draw something
    this->sprite->display(x, y, scale, degree, NULL, this->vel[0]<0?SDL_FLIP_HORIZONTAL:SDL_FLIP_NONE);

    SDL_Renderer * renderer=getMyGame()->getRenderer();
    if(this->leader) SDL_SetRenderDrawColor(renderer,255,0,0,100);
    else SDL_SetRenderDrawColor(renderer,200,200,200,100);

    draw_bounding_box();

    if(this->leader) SDL_SetRenderDrawColor(renderer,255,0,0,100);
    else SDL_SetRenderDrawColor(renderer,200,200,200,100);
    draw_circle(x,y,view_radius);
  }

  void MyChickenAgent::handle_event(SDL_Event & e)
  {

  }

  //TODO: get a list of nearby neighbors
  void MyChickenAgent::get_neighbors(list<MyChickenAgent*>& neighbors)
  {
    const std::list<MyAgent * > &  agents=getMyGame()->getSceneManager()->get_active_scene()->get_agents();
    for( MyAgent * agent : agents)
    {
      if(agent==this) continue;
      MyChickenAgent * other = dynamic_cast<MyChickenAgent *>(agent);
      if( other==NULL ) continue; //not a chicken

      if( (this->pos-other->pos).norm() < this->view_radius )
      {
        neighbors.push_back(other);
      }
    }//end for agent
  }

  MyDragonAgent * MyChickenAgent::get_dragon()
  {
    MyDragonAgent* dragon=NULL;

    //find the dragon agent and get its position
    const std::list<MyAgent * > &  agents=getMyGame()->getSceneManager()->get_active_scene()->get_agents();
    for( MyAgent * agent : agents)
    {
      if( dynamic_cast<MyDragonAgent*>(agent)==NULL ) //not a dragon
        continue;
      dragon=dynamic_cast<MyDragonAgent*>(agent);
      break;
    }

    return dragon;
  }


  void MyChickenAgent::update()
  {
    //update agent position using particle position
    x=this->pos[0];
    y=this->pos[1];
    degree=atan2(this->vel[1],this->vel[0])* 180 / PI;
    if(this->vel[0]<0) degree-=180;
  }

}//end namespace
