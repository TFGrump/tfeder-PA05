#include "MyPhysicsEngine.h"

namespace GMUCS425
{

void MyPhysicsEngine::step(float h)
{
  vector<State> states, new_states;
  particles_to_states(states);
  new_states.resize(states.size());
  step(h, states, new_states);
  states_to_particles(new_states);
}

//step forward by h secs
void MyPhysicsEngine::step(float h, const vector<State>& states, vector<State>& new_states)
{
    vector<Vector2d> forces;
    vector<dState> dxdv;

    //1. update forces
    for(MyParticle * p : m_particles)
    {
      p->compute_force();
      forces.push_back(p->force);
    }

    //2. compute derivatives
    dxdv.resize(states.size());
    derive(states, forces, dxdv);

    //3. solve ode to get new states
    ode(h, states, dxdv, new_states);
}

//TODO: compute the derivatives
void MyPhysicsEngine::derive(const vector<MyPhysicsEngine::State>& states,
                             const vector<Vector2d>& forces,
                             vector<MyPhysicsEngine::dState>& dxdv)
{
    // BBCU 11/11 @ 59:00
    int n = states.size();
    auto particle = this->m_particles.begin();
    for (int i = 0; i < n; i++) 
    {
        dxdv[i].dx = states[i].v;
        dxdv[i].dv = forces[i] / (*particle)->mass;
        particle++;
    }
}

//TODO: implement midpoint method
void MyPhysicsEngine::ode(float h,
                          const vector<MyPhysicsEngine::State>& states,
                          const vector<MyPhysicsEngine::dState>& dxdv,
                          vector<MyPhysicsEngine::State>& new_states)
{
    // BBCU 11/11 @ 1:02:00
    int n = states.size();
    vector<MyPhysicsEngine::State> states_mid;
    states_mid.resize(n);
    vector<Vector2d> F_mid;
    vector<MyPhysicsEngine::dState> dxdv_mid;
    dxdv_mid.resize(n);

    euler(h/2,states,dxdv,states_mid); 

    states_to_particles(states_mid);
    for (MyParticle* p : m_particles)
    {
        p->compute_force();
        F_mid.push_back(p->force);
    }

    derive(states_mid, F_mid, dxdv_mid);
    euler(h, states, dxdv_mid, new_states);
    //euler(h, states, dxdv, new_states);
}

//TODO: implement Euler's method
void MyPhysicsEngine::euler(float h,
                            const vector<MyPhysicsEngine::State>& states,
                            const vector<MyPhysicsEngine::dState>& dxdv,
                            vector<MyPhysicsEngine::State>& new_states)
{
    // BBCU 11/11 @ 1:00:00
    int n = states.size();
    for (int i = 0; i < n; i++)
    {
        new_states[i].x = states[i].x + dxdv[i].dx * h;
        new_states[i].v = states[i].v + dxdv[i].dv * h;
    }
}

}//end namespace GMUCS425
