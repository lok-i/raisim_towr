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
#define ON_RACK 1 //Means no physics, just movement like animation
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
  world.setTimeStep(0.0025);
  monoped->setGeneralizedCoordinate({   0, 0, 1, //base coordinates 0.5 is appropriate height
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
  monoped->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  monoped->setPdGains(jointPgain, jointDgain);
  monoped->setName("monoped");
  // monoped->printOutBodyNamesInOrder();
  // std::vector<std::string> names = monoped->getMovableJointNames();
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

// void getJointAngles(raisim::ArticulatedSystem *monoped, std::vector<float,3> &jointAngles)
// {
//     // The order of the angles is abduction, hip, joint
//     auto temp = monoped->getGeneralizedCoordinate();
//     jointAngles[0]=temp[7];
//     jointAngles[1]=temp[8];
//     jointAngles[2]=temp[9];
// }


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
    auto controller = [&monoped]() 
    {
      
    };

  vis->setControlCallback(controller);

    vis->run();

    /// terminate
    vis->closeApp();



  return 0;
}
