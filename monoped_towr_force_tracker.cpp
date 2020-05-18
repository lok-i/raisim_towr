//need to update the grnd_to_base function



#include <raisim/OgreVis.hpp>
#include "raisimBasicImguiPanel.hpp"
#include "raisimKeyboardCallback.hpp"
#include "helper.hpp"
#include <xpp_inv/hyqleg_inverse_kinematics.h>
#include <xpp_inv/cartesian_declarations.h>
#include "matplotlibcpp.h"

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
#include <typeinfo>
#include <leg.cpp>

#define base_height_initial 0.5
#define gravity true
#define actuators_only true
#define PD_tuning_mode false
#define towr_initial_output false

using namespace towr;
namespace plt = matplotlibcpp;

static std::string VectortoString(Eigen::Vector3d alpha)
{
    std::stringstream ss;
    for (int i =0;i<3;i++)
     ss << alpha[i]<<',';
    return ss.str();
}


//converts end effector from ground reference to base refernce such
//that axes are parallel
Eigen::Vector3d grnd_ref_to_base_ref(Eigen::Vector3d ee,Eigen::Vector3d base)
{
 Eigen::Vector3d offset_base_to_hip(0.0, 0.0, 0.15);
 base = base -offset_base_to_hip;//base to hip offset

 return(ee-base);
}

//towr optimization
void towr_trajectory(NlpFormulation &formulation,SplineHolder &solution,Eigen::Vector3d target)
{


  // terrain
  formulation.terrain_ = std::make_shared<FlatGround>(0.0);

  // Kinematic limits and dynamic parameters of the hopper
  formulation.model_ = RobotModel(RobotModel::Monoped);

  // set the initial position of the hopper
  formulation.initial_base_.lin.at(kPos).z() = base_height_initial;
  formulation.initial_ee_W_.push_back(Eigen::Vector3d::Zero());

  // define the desired goal state of the hopper
  formulation.final_base_.lin.at(towr::kPos) =target;

  // Parameters that define the motion. See c'tor for default values or
  // other values that can be modified.
  // First we define the initial phase durations, that can however be changed
  // by the optimizer. The number of swing and stance phases however is fixed.
  // alternating stance and swing:     ____-----_____-----_____-----_____
  formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
  formulation.params_.ee_in_contact_at_start_.push_back(true);

  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;

  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);

  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);

  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);

  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  cout.precision(2);
  nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  

  if(towr_initial_output)
  {
  cout << "\n====================\nMonoped trajectory:\n====================\n";

  double t = 0.0;
  while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

    
    xpp::HyqlegInverseKinematics leg1;
    Eigen::Vector3d end_factor_pt( solution.ee_motion_.at(0)->GetPoint(t).p().transpose());
    Eigen::VectorXd q0 =leg1.GetJointAngles(end_factor_pt);
    
    cout << "Foot position x,y,z:          \t";
    cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;
    cout << "Angle for actuator:"<< q0 <<endl;

    cout << "Contact force x,y,z:          \t";
    cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
    std::string foot_in_contact = contact? "yes" : "no";
    cout << "Foot in contact:              \t" + foot_in_contact << endl;

    cout << endl;

    t += 0.2;
  }}
}

//rendering callback
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

//function to calculate magnitude of a vector
double mag_vector(Eigen::Vector3d x)
{
return(sqrt(pow(x(0),2)+pow(x(1),2)+pow(x(2),2)));
}

/*function to calcute the force exerted
  in the tip using Jacobian and genralized
  force(tau)*/
Eigen::Vector3d calc_force_tip(Eigen::Vector3d tau,Eigen::Vector3d q0)
{
  
    
    Eigen::Matrix<double, 3, 3> jacob;
    jacobian_2(jacob, q0);//Jacobian wrt to local leg frame
  
    
    
    Eigen::Matrix<double,3,3> Rot_legframe_to_hipframe;
    Rot_legframe_to_hipframe << 0, 0,-1,
                                 0,-1, 0,
                                -1, 0, 0;
    //Jh = Rhl*Jl
    Eigen::Matrix<double, 3, 3> jacob_inv = (Rot_legframe_to_hipframe*jacob).inverse();



    std::cout<<std::endl<<"Ftip = ((R*J)^-1)^T * tau"<<std::endl<<jacob_inv.transpose()*tau<<"\n\n"; 
   
   return(Eigen::Vector3d(jacob_inv.transpose()*tau));

}

/*
f_trip should be wrt to the hip frame
*/
Eigen::Vector3d calc_tau_(Eigen::Vector3d f_tip,Eigen::Vector3d q0)
{
  
     
    Eigen::Matrix<double, 3, 3> jacob;
    jacobian_2(jacob, q0);//Jacobian wrt to local leg frame
    
    
    Eigen::Matrix<double,3,3> Rot_legframe_to_hipframe;
    Rot_legframe_to_hipframe << 0, 0,-1,
                                 0,-1, 0,
                                -1, 0, 0;
    //Jh = Rhl*Jl
    Eigen::Matrix<double, 3, 3> jacob_rot = Rot_legframe_to_hipframe*jacob;

 
    
    Eigen::Vector3d tau;
    tau = jacob_rot.transpose() * f_tip ;
     
   std::cout<<std::endl<<"tau = (R*J)^T * Ftip"<<std::endl<<tau<<"\n\n"; 
   
    return(tau);

}
/*
R = Rz(2)Ry(1)Rx(0)
=


cos 1 cos 2  //00

sin 0 sin 1 cos 2 − cos 0 sin 2 //01

cos 0 sin 1 cos 2 + sin 0 sin 2 //02

cos 1 sin 2 //10

sin 0 sin 1 sin 2 + cos 0 cos 2 //11

cos 0 sin 1 sin 2 − sin 0 cos 2 //12
− sin 1 //20
sin 0 cos 1 //21
cos 0 cos 1 //22


*/

Eigen::Vector3d Transform_Vector_to_hip_frame_frm_grnd(Eigen::Vector3d base_co,Eigen::Vector3d b_a,Eigen::Vector3d v_g)
{

 Eigen::Matrix<double,4,4> Transform;
//rotation part
 Transform(0,0) = cos(b_a[1])*cos(b_a[2]);

 Transform(0,1) = sin(b_a[0])*sin(b_a[1])*cos(b_a[2]) - cos(b_a[0])*sin(b_a[2]);
 
 Transform(0,2) = cos(b_a[0])*sin(b_a[1])*cos(b_a[2]) + sin(b_a[0])*sin(b_a[2]);

 
 Transform(1,0) = cos(b_a[1])*sin(b_a[2]);

 Transform(1,1) = sin(b_a[0])*sin(b_a[1])*sin(b_a[2]) + cos(b_a[0])*cos(b_a[2]);

 Transform(1,2) = cos(b_a[0])*sin(b_a[1])*sin(b_a[2]) - sin(b_a[0])*cos(b_a[2]);

 
 Transform(2,0) = -1*sin(b_a[1]);

 Transform(2,1) = sin(b_a[0])*cos(b_a[1]);

 Transform(2,2) = cos(b_a[0])*cos(b_a[1]);


//linear distance from gound origin
 Transform(0,3) = base_co[0];
 Transform(1,3) = base_co[1];
 Transform(2,3) = base_co[2];
// last dummy row 
 for (int i =0 ;i <4;i++)
 {
if(i==3)
Transform(3,i) = 1;
else
Transform(3,i) = 0;

 }

//Tgb is ready need to convert to Tgh
//need to consider an offset along the z axis of the new frame (base to hip)
Eigen::VectorXd Hip_wrt_g(4);
Eigen::VectorXd Hip_in_base(4);
Hip_in_base << 0, 0, -0.15, 1;
Hip_wrt_g = Transform*Hip_in_base;
 
 for (int i =0 ;i <4;i++)
 {

  Transform(i,3) = Hip_wrt_g(i);

 }

 /////Tgh is ready..but we need Thg = Tgh^-1
Eigen::Matrix<double,4,4> Transform_inv = Transform.inverse();
//vec3 to vec4 for homogeneous co ordinate system
Eigen::VectorXd Input(4),Output(4);

 for (int i =0 ;i <4;i++)
 	if(i==3)
		Input(i)= 1;
	else
		Input(i) = v_g(i);

Output = Transform_inv*Input ;

return(Eigen::Vector3d(Output[0],Output[1],Output[2])); 


}


int main(int argc, char **argv) 
{

//in order to match the speed of cin & cout to scanf anf printf
 std::ios_base::sync_with_stdio(false);
 std::cin.tie(0);
 std::cout.tie(0);

 //towr variables
 NlpFormulation formulation; 
 SplineHolder solution;
 
 /*finale Target base position
   that for wich towr should deduce 
   a trajectory*/
 Eigen::Vector3d Target_base;   
 std::cout<<"Enter target base co_ordinates:"<<std::endl;
 for (int i =0;i<3;i++)
    std::cin>>Target_base(i);

 //defaut P and D gains for the actuators
 float P_gain = 200.0,D_gain =10.0;

 //input PD gains if u need are tuning
 if(PD_tuning_mode)
  {
    std::cout<<"Enter P and D gains:"<<std::endl;
    std::cin>>P_gain>>D_gain;
  }
  

  //calculate the required trajectory from towr
  towr_trajectory(formulation,solution,Target_base);
  
  

  //initialize rasim world
  raisim::World world;
  
  if(!gravity)
   world.setGravity({0,0,0}); // by default gravity is set to {0,0,g}
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
  raisim::gui::manualStepping = false; 
  //raisim::gui::Collisionbodies = true; 
  /// starts visualizer thread
  vis->initApp();

  /// create raisim objects
  auto ground = world.addGround();
  

  /// create visualizer objects
  auto groundVis = vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

   

 


  //initializes the monoped
  auto monoped = world.addArticulatedSystem(raisim::loadResource("monoped/monoped.urdf"));
  auto monopedVis = vis->createGraphicalObject(monoped, "monoped");
  monoped->setGeneralizedCoordinate({0, 0, base_height_initial, //base co ordinates 
                                     1, 0, 0, 0,  //orientation 
                                     0,1.09542,-2.3269});



  //initializes the generalized co ordinates,velocities , torques and gains
  Eigen::VectorXd jointNominalConfig(monoped->getDOF()+1), jointVelocityTarget(monoped->getDOF());
  Eigen::VectorXd jointState(monoped->getDOF()), jointForce(monoped->getDOF()), jointPgain(monoped->getDOF()), jointDgain(monoped->getDOF());
  jointPgain.setZero();
  jointDgain.setZero();
  jointVelocityTarget.setZero();
  
 // P and D gains for depending on the program mode
  if (actuators_only)
  {jointPgain.tail(3).setConstant(P_gain);
   jointDgain.tail(3).setConstant(D_gain);}

  else
  {

     for(int k =0;k<9;k++)
     {
      if(k<=2)//for base linear posn
      {
        jointPgain(k)=200.0;
        jointDgain(k)=10.0;

      }
      else
      {

        if(k<=5) //for base orientation
        {
          jointPgain(k)=200.0;
          jointDgain(k)=10.0;

        }
        else //for actuators
        {
        jointPgain(k)=P_gain;
        jointDgain(k)=D_gain;
        }

     
      }}}

  monoped->setGeneralizedForce(Eigen::VectorXd::Zero(monoped->getDOF()));
  monoped->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
  monoped->setPdGains(jointPgain, jointDgain);
  monoped->setName("monoped");
  //monoped->printOutBodyNamesInOrder();
  
  //local time variable to sample from the spline genrated by towr
  double t = 0.0;
  bool reverse = true;
 
  //variable to accumulate data for plots  
  std::vector<double> time;
  std::vector<double> F_towr;//Force tip of towr
  std::vector<double> F_raisim;//Force tip of towr

  // lambda function for the controller
  auto controller = [&monoped ,&solution,&t,&reverse,&time,&F_towr,&F_raisim]()
   {
    
    //variable to take care of stable simulation rendering
    static size_t controlDecimation = 0;


    //resets the orientation once in every 10000 iterations
    if (controlDecimation++ % 10000 == 0)
    monoped->setGeneralizedCoordinate({0, 0, base_height_initial,1, 0, 0,0,0,1.09542,-2.3269});
      
    //keeps the controller dead for 50 steps between 2 different updates,for propper renderin
    if (controlDecimation % 50 != 0)
    return;
    

    /// monoped joint PD controller
    if(t<2)
        {
          
       
        //inverse kinematics calculations along the spline
        xpp::HyqlegInverseKinematics leg;
        Eigen::Vector3d Base_co(solution.base_linear_->GetPoint(t).p().transpose());
        
        //Base orientaton angles       
        Eigen::Vector3d Base_a(solution.base_angular_->GetPoint(t).p());
        Base_a = Base_a/M_PI*180;


        Eigen::Vector3d ee_grnd(solution.ee_motion_.at(0)->GetPoint(t).p().transpose()); //end effector wrt ground 

        Eigen::Vector3d Force_towr(solution.ee_force_.at(0)->GetPoint(t).p().transpose());



        Eigen::Vector3d ee_H = Transform_Vector_to_hip_frame_frm_grnd(Base_co,Base_a,ee_grnd);
        Eigen::Vector3d q0   = leg.GetJointAngles(ee_H);
        
        
        //sets force according to towr
        Eigen::VectorXd tau_9(9);
        

        Eigen::Vector3d tau_3_temp = calc_tau_(Transform_Vector_to_hip_frame_frm_grnd(Base_co,Base_a,Force_towr),q0);
        
        for(int i=0;i<9;i++)
        {

          if(i>=6)
            tau_9(i)=tau_3_temp(i-6);
          else
            tau_9(i)=0;
        }

        //genarlized feed forward force
        monoped->setGeneralizedForce(tau_9);

        std::cout<<"\n"<<"Generalized Force:"<<monoped->getGeneralizedForce();
        
        std::cout<<std::endl<<"t:"<<t<<std::endl;
        std::cout<<std::endl<<"Contact force of ee @ time t(towr):"<<std::endl;
        
        std::cout<<Force_towr<<std::endl;

        //data collection for plots
        time.push_back(t);
        F_towr.push_back(mag_vector(Force_towr));
        F_raisim.push_back(mag_vector(calc_force_tip(tau_3_temp,q0)));
        t+=0.01;
        }

  };
  
 vis->setControlCallback(controller);

  /// set camera
  vis->select(groundVis->at(0));
  vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), -Ogre::Radian(M_PI_4), 2);

  /// run the app

  vis->run();

  /// terminate
  vis->closeApp();
  


  //finally plot the results
  // plt::figure_size(1200, 780);
  // plt::plot(time, F_towr,{{"label", "towr"}});
  // plt::plot(time,F_raisim,{{"label", "raisim"}});
  // std::string filename = VectortoString(Target_base);
  // std::string plot_path = "../Plots/force_input_frm_towr->raisim:Target:("+filename+").png";
  // std::cout << "Saving result to " << plot_path << std::endl;;
  // plt::save(plot_path);

  return 0;
}
