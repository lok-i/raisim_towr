#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include <xpp_inv/hyqleg_inverse_kinematics.h>
#include <xpp_inv/cartesian_declarations.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "leg.cpp"
#include <math.h>
#define ON_RACK 0 //Means no physics, just movement like animation
#define RAIBERT 0 
#define RAIBERT2 1 
using Vector3d = Eigen::Vector3d;

// Global Pointers -- Currently best solution to divide code into functions, returning and passing pointers of different classes through is too convoluted
raisim::World world;
raisim::Ground *ground;
raisim::ArticulatedSystem *monoped;
auto vis = raisim::OgreVis::get();
std::vector<raisim::GraphicObject> *groundVis;
std::vector<raisim::GraphicObject> *monopedVis;
void setupCallback() {
  auto vis = raisim::OgreVis::get();

  /// light
  vis->getLight()->setDiffuseColour(1, 1, 1);
  vis->getLight()->setCastShadows(true);
  Ogre::Vector3 lightdir(-3, -3, -0.5);
  lightdir.normalise();
  vis->getLightNode()->setDirection({lightdir});

  /// load  textures
  vis->addResourceDirectory(vis->getResourceDir() + "/material/checkerboard");
  vis->loadMaterialFile("checkerboard.material");

  /// shdow setting
  vis->getSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE);
  vis->getSceneManager()->setShadowTextureSettings(2048, 3);

  /// scale related settings!! Please adapt it depending on your map size
  // beyond this distance, shadow disappears
  vis->getSceneManager()->setShadowFarDistance(30);
  // size of contact points and contact forces
  vis->setContactVisObjectSize(0.06, .6);
  // speed of camera motion in freelook mode
  vis->getCameraMan()->setTopSpeed(5);
}


void InitWorld()
{
    /* Initializes the world
    global pointers used ground, monoped, world
    */
  ground = world.addGround();
  monoped = world.addArticulatedSystem(raisim::loadResource("monoped/monoped.urdf"));
  world.setGravity({0,0,-9.82}); // by default gravity is set to {0,0,g}
  world.setTimeStep(0.0001);
  monoped->setGeneralizedCoordinate({   0, 0, 0.6, //base coordinates 0.5 is appropriate height
          1, 0, 0, 0,  //orientation 
          0,1.09542,-2.3269});

  Eigen::VectorXd jointNominalConfig(monoped->getDOF()+1), jointVelocityTarget(monoped->getDOF());
  Eigen::VectorXd jointState(monoped->getDOF()), jointForce(monoped->getDOF()), jointPgain(monoped->getDOF()), jointDgain(monoped->getDOF());
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  // P and D gains for the leg actuators alone
  jointPgain.tail(3).setConstant(200.0);
  jointDgain.tail(3).setConstant(10.0);
  monoped->setGeneralizedForce(Eigen::VectorXd::Zero(monoped->getDOF()));
  //monoped->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
  monoped->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  monoped->setPdGains(jointPgain, jointDgain);
  monoped->setName("monoped");
  // monoped->printOutBodyNamesInOrder();
  // std::vector<std::string> names = monoped->getMovableJointNames();
  // std::cout<<"dof: "<<monoped->getDOF()<<std::endl;
  // for(auto x: names)
  // {
  //   std::cout<<x<<std::endl;
  // }

}

void InitGraphics()
{   
    /*
    Initializes graphics. Global Pointers used are ground, monoped, vis, monopedVis, groundVis
    */
    /// these method must be called before initApp
    vis->setWorld(&world);
    vis->setWindowSize(2600, 1200);
    vis->setImguiSetupCallback(imguiSetupCallback);
    vis->setImguiRenderCallback(imguiRenderCallBack);
    vis->setKeyboardCallback(raisimKeyboardCallback);
    vis->setSetUpCallback(setupCallback);
    vis->setAntiAliasing(2);
    vis->setDesiredFPS(25);
    //simulation is automatically stepped, if is false
    raisim::gui::manualStepping = true; 
    //raisim::gui::Collisionbodies = true; 
    /// starts visualizer thread
    vis->initApp();

    /// create raisim objects
    /// create visualizer objects
    monopedVis = vis->createGraphicalObject(monoped, "monoped");
    groundVis = vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

}

void getJointAngles(Vector3d &jointAngles, raisim::ArticulatedSystem *monoped)
{
    // The order of the angles is abduction, hip, joint
    auto temp = monoped->getGeneralizedCoordinate();
    jointAngles[0]=temp[7];
    jointAngles[1]=temp[8];
    jointAngles[2]=temp[9];
}

float magnitude(Vector3d &vec)
{
    // The order of the angles is abduction, hip, joint
    return sqrt(pow(vec[0],2)+pow(vec[1],2)+pow(vec[2],2));
}

void convert_spherical(Vector3d &spherical, Vector3d &xyz)
{
  spherical[0] = sqrt(pow(xyz[0],2)+pow(xyz[1],2)+pow(xyz[2],2));
  spherical[1] = atan2(xyz[1],xyz[0]);
  spherical[2] = atan2(sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1]), xyz[2]);
}

void convert_cartesian_vec(Vector3d &xyz, Vector3d &spherical, Vector3d &joint_angles)
{
  //Complex equation for PID in spherical coordinates
  Vector3d end_eff_pos;
  forward_kinematics(end_eff_pos, joint_angles);
  float th1 = joint_angles[0];
  float th2 = joint_angles[1];
  float th3 = joint_angles[2];
  end_eff_pos = end_eff_pos/(magnitude(end_eff_pos));
  xyz[0] = spherical[0]*end_eff_pos[0]+spherical[2]*-1*(L2*sin(th2+PI/2)+L3*sin(th2+th3+PI/2));
  xyz[1] = spherical[0]*end_eff_pos[1] + spherical[1]*cos(th1)*(L2*sin(th2+PI/2)+L3*sin(th2+th3+PI/2)) + spherical[2]*sin(th1)*(L2*cos(th2+PI/2)+L3*cos(th2+th3+PI/2));
  xyz[2] = spherical[0]*end_eff_pos[2] + spherical[1]*-sin(th1)*(L2*sin(th2+PI/2)+L3*sin(th2+th3+PI/2))+ spherical[2]*cos(th1)*(L2*cos(th2+PI/2)+L3*cos(th2+th3+PI/2));
}



int main(int argc, char **argv) 
{

    xpp::HyqlegInverseKinematics leg;
    InitWorld();
    InitGraphics();
    
    // lambda function for the controller
    

    /// set camera
    vis->select(groundVis->at(0));
    vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);

    /// run the app
    double t = 0.0;
    auto controller = [&monoped, &world, &t]() 
    {
      float amp = 0.05;
      float omega = 10;
      float t_elapsed;
      if(ON_RACK)
      {
        // Vector3d end_effector_pos( amp*sin(t*omega),0.1,  amp*cos(t*omega)+0.4);
        // Vector3d end_effector_pos(0.0,0.1,0.4);
        // Vector3d joint_pos;
        Vector3d fwd_kin_pos;
        // inverse_kinematics(joint_pos,end_effector_pos);
        // monoped->setGeneralizedCoordinate({0, 0, 1, 1,0,0,0, joint_pos[0], joint_pos[1], joint_pos[2]});
        Vector3d joint_pos( 0,1.09542,-2.3269);
        forward_kinematics(fwd_kin_pos, joint_pos);
        // fwd_kin_pos -= end_effector_pos;
        std::cout<<fwd_kin_pos[0]<<","<<fwd_kin_pos[1]<<","<<fwd_kin_pos[2]<<std::endl; 
        // Vector3d end_effector_force(0.,0.0,0.001);
        // Vector3d joint_pos;
        // auto temp = monoped->getGeneralizedCoordinate();
        // joint_pos[0] = temp[7];
        // joint_pos[1] = temp[8];
        // joint_pos[2] = temp[9];
        // Vector3d torques;
        // inverse_dynamics(torques, end_effector_force, joint_pos);
        // monoped->setGeneralizedForce({0,0,0,0,0,0,torques[0],torques[1],torques[2]}); //One less value than generalized coordinates
        // monoped->setGeneralizedForce({0,0,0,0,0,0,0.0,0.0,0.01});
        // monoped->setGeneralizedCoordinate({0, 0, 1, 1,0,0,0, joint_pos[0], 1.5, joint_pos[2]});
        // std::cout<<torques[0]<<","<<torques[1]<<","<<torques[2]<<std::endl;
      }
      if(RAIBERT)
      {
        //0,0,0.35
        //First focus on getting spring like behaviour, spring like working but loses balance easily
        //Now should try to get more balanced behaviour
        Vector3d current_joint_pos;
        Vector3d old_values(0,0,0);
        Vector3d diff(0,0,0);
        Vector3d xyz_end_eff_pos;
        Vector3d spherical_end_eff_pos;
        Vector3d spherical_force;
        Vector3d Pgains(1, 200, 0.1);
        Vector3d Dgains(0.0001, 10, 0.000001);
        Vector3d end_eff_force;
        Vector3d torques;
        Vector3d equilibrium(0,0,0.3);
        float k;
        float mag;
        float force;
        float eq_mag;
        getJointAngles(current_joint_pos, monoped);
        forward_kinematics(xyz_end_eff_pos, current_joint_pos);
        convert_spherical(spherical_end_eff_pos, xyz_end_eff_pos);
        if(t != 0.0)
        {
          if(abs(t_elapsed)>0.001)
          {
            diff[0]=(spherical_end_eff_pos[0] - old_values[0])/t_elapsed;
            diff[1]=(current_joint_pos[0]-old_values[1])/t_elapsed;
          }
        }
        //0.35 3.14 0
        spherical_force[0] = Pgains[0]*(0.35 - spherical_end_eff_pos[0]) + Dgains[0]*(0-diff[0]) ;
        spherical_force[1] = Pgains[1]*(0.1 - current_joint_pos[0]);
        // spherical_force[1] = 0.0;
        //equations are wrong
        // spherical_force[0]=1.0;
        spherical_force[0]=0.0;
        spherical_force[1]=0.001;
        spherical_force[2] = 0.0;

        convert_cartesian_vec(end_eff_force, spherical_force, current_joint_pos);
        // std::cout<<diff[0]<<std::endl;
        // std::cout<<"sphere coord: "<<spherical_end_eff_pos[0]<<","<<spherical_end_eff_pos[1]<<","<<spherical_end_eff_pos[2]<<std::endl;
        // std::cout<<"sphere force: "<<spherical_force[0]<<","<<spherical_force[1]<<","<<spherical_force[2]<<std::endl;
        // std::cout<<"end force: "<<end_eff_force[0]<<","<<end_eff_force[1]<<","<<end_eff_force[2]<<std::endl;
        // k = 2.0;
        // force = -k*(magnitude(current_end_eff_pos) - magnitude(equilibrium));
        // diff = current_end_eff_pos;

        // if(magnitude(diff)>0.01)
        // {
        //   spring_force = force*diff /(magnitude(diff));
        // }
        inverse_dynamics(torques,end_eff_force, current_joint_pos);        
        monoped->setGeneralizedForce({0,0,0,0,0,0,spherical_force[1],torques[1],torques[2]}); //One less value than generalized coordinates
        old_values[0]=spherical_end_eff_pos[0];
        old_values[1]=current_joint_pos[0];
      }
      if(RAIBERT2)
      {
        Eigen::VectorXd jointPgain(monoped->getDOF()), jointDgain(monoped->getDOF());
        jointPgain[6]=20;
        jointPgain[7]=20;
        jointPgain[8]=20;
        jointDgain[6]=0.1;
        jointDgain[7]=0.1;
        jointDgain[8]=0.01;        
        monoped->setPdGains(jointPgain, jointDgain);

        Vector3d end_eff_pos(0.0,0.0,0.35);
        Vector3d des_joint_pos;
        Eigen::VectorXd des_vel_target(9);
        Eigen::VectorXd des_pos_target(10);
        des_vel_target.setZero();
        des_pos_target.setZero();
        inverse_kinematics(des_joint_pos, end_eff_pos);
        des_pos_target[7]=des_joint_pos[0];
        des_pos_target[8]=des_joint_pos[1];
        des_pos_target[9]=des_joint_pos[2];
        monoped->setPdTarget(des_pos_target, des_vel_target);
        auto coords= monoped->getGeneralizedCoordinate();
        // std::cout<<"coord: "<<coords[7]<<","<<coords[8]<<","<<coords[9]<<std::endl;
      }

      t_elapsed = world.getTimeStep();
      t += t_elapsed;
    };

  vis->setControlCallback(controller);

    vis->run();

    /// terminate
    vis->closeApp();



  return 0;
}
