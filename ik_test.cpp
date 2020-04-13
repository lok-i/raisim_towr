#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include <xpp_inv/hyqleg_inverse_kinematics.h>
#include <xpp_inv/cartesian_declarations.h>
#include <Eigen/Dense>

#define ON_RACK 1
using Vector3d = Eigen::Vector3d;

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

void getJointAngles(Vector3d end_effector_pos, Vector3d &joint_pos , xpp::HyqlegInverseKinematics &leg)
{
  /*
  Function takes in desired coordinates as x,y,z values and outputs the abduction, hip, knee angles.
  */ 
  joint_pos =leg.GetJointAngles(end_effector_pos);
}
int main(int argc, char **argv) 
{

  xpp::HyqlegInverseKinematics leg;
  raisim::World world;
    
  world.setGravity({0,0,-9.82}); // by default gravity is set to {0,0,g}
  world.setTimeStep(0.0025);

  auto vis = raisim::OgreVis::get();

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
  auto ground = world.addGround();

  /// create visualizer objects
  auto groundVis = vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

  const size_t N = 100;
  auto monoped = world.addArticulatedSystem(raisim::loadResource("monoped/monoped.urdf"));
  auto monopedVis = vis->createGraphicalObject(monoped, "monoped");
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

  //to take random samples
  monoped->printOutBodyNamesInOrder();
  std::vector<std::string> names = monoped->getMovableJointNames();
  for(auto x: names)
  {
    std::cout<<x<<std::endl;
  }

// lambda function for the controller
double time_ = 0.;
auto controller = [&monoped ,&leg, &world, &time_]()
{
  float amp = 0.05;
  float omega = 10;
  static size_t controlDecimation = 0;

  if (controlDecimation++ % 10000 == 0)
  {
    std::cout<<"Reset"<<controlDecimation<<std::endl;
    monoped->setGeneralizedCoordinate({0, 0, 0.5, 1,0,0,0, 0,1.09542,-2.3269});
  }
  /// laikago joint PD controller
  Eigen::VectorXd jointNominalConfig(monoped->getDOF()+1), jointVelocityTarget(monoped->getDOF());
  jointVelocityTarget.setZero();
  
  if(ON_RACK)
  {
    Vector3d end_effector_pos(amp*sin(time_*omega), 0., amp*cos(time_*omega) - 0.4);
    Vector3d joint_pos(0.,0.,0.);
    getJointAngles(end_effector_pos, joint_pos, leg);
    monoped->setGeneralizedCoordinate( {0, 0, 2, 1,0,0,0, joint_pos[0],joint_pos[1],joint_pos[2] });
  }
  else
  {
    jointNominalConfig << 0, 0, 2,  1, 0, 0, 0,  1,1,1;  //abduction, knee, hip joint is the order of the data
    monoped->setPdTarget(jointNominalConfig, jointVelocityTarget);
  }
  time_ += world.getTimeStep();
};

  vis->setControlCallback(controller);

  /// set camera
  vis->select(groundVis->at(0));
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);

  /// run the app

  vis->run();

  /// terminate
  vis->closeApp();



  return 0;
}
