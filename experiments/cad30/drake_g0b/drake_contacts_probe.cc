// Independent G0b geometry/contact and actuation audit, no CAD meshes embedded.
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/query_object.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/diagram.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>
#include <tuple>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/plant/contact_results.h>
#include <drake/systems/analysis/simulator.h>
#include <array>

int main(int argc, char** argv) {
  if (argc != 3 && argc != 4) {std::cerr << "usage: drake_contacts_probe scene.xml qD.txt\n"; return 1;}
  try {
    using namespace drake;
    systems::DiagramBuilder<double> builder;
    auto [plant, graph] = multibody::AddMultibodyPlantSceneGraph(&builder, 0.002);
    multibody::Parser parser(&plant, &graph);
    parser.AddModels(argv[1]);
    std::vector<std::string> names;
    for(int i=0;i<plant.num_actuators();i++) names.push_back(plant.get_joint_actuator(multibody::JointActuatorIndex(i)).name());
    const bool repair=(argc==4 && std::string(argv[3])=="--repair-actuators");
    if(repair) {
      struct A{std::string name;std::string joint;double cap;};
      std::vector<A> acts;
      for(int i=0;i<plant.num_actuators();i++) {
        const auto& a=plant.get_joint_actuator(multibody::JointActuatorIndex(i));
        const bool head=(a.name()=="motor0_ctrl" || a.name()=="motor1_ctrl");
        acts.push_back({a.name(),a.joint().name(),head?0.2:0.7});
      }
      for(const auto& x:acts) plant.RemoveJointActuator(plant.GetJointActuatorByName(x.name));
      for(const auto& x:acts) {
        plant.AddJointActuator(x.name,plant.GetJointByName(x.joint),x.cap);
        auto& a=plant.get_mutable_joint_actuator(plant.GetJointActuatorByName(x.name).index());
        a.set_default_rotor_inertia(0.002);
        a.set_default_gear_ratio(1.0);
      }
    }
    plant.Finalize();
    auto diagram=builder.Build();
    auto context=diagram->CreateDefaultContext();
    auto& plant_ctx=diagram->GetMutableSubsystemContext(plant, context.get());
    auto& graph_ctx=diagram->GetSubsystemContext(graph, *context);
    std::ifstream fs(argv[2]);
    if(!fs) throw std::runtime_error("Cannot open input qD");
    Eigen::VectorXd q(plant.num_positions());
    for(int i=0;i<q.size();i++) if(!(fs>>q[i])) throw std::runtime_error("Incomplete qD");
    plant.SetPositions(&plant_ctx, q);
    Eigen::VectorXd v(plant.num_velocities()), u(plant.num_actuators());
    bool has_dynamic_input=bool(fs >> v[0]);
    if(has_dynamic_input) {
      for(int i=1;i<v.size();i++) if(!(fs>>v[i])) throw std::runtime_error("Invalid v");
      for(int i=0;i<u.size();i++) if(!(fs>>u[i])) throw std::runtime_error("Invalid u");
      plant.SetVelocities(&plant_ctx,v);
      plant.get_actuation_input_port().FixValue(&plant_ctx,u);
    }
    std::cout << std::setprecision(15);
    std::cout << "meta " << plant.num_positions() << " " << plant.num_velocities() << " " << plant.num_actuators() << "\n";
    for(const auto& name:names) {
      auto& actuator=plant.GetJointActuatorByName(name);
      std::cout << "actuator " << actuator.name() << " " << actuator.effort_limit() << " " << actuator.default_rotor_inertia() << "\n";
    }
    for(int i=0;i<plant.num_joints();i++) {
      const auto& j=plant.get_joint(multibody::JointIndex(i));
      if (j.num_velocities()==1) {
        std::cout << "joint_damping " << j.name() << " " << j.default_damping_vector()[0] << "\n";
      }
    }
    Eigen::MatrixXd mass(plant.num_velocities(),plant.num_velocities());
    plant.CalcMassMatrix(plant_ctx,&mass);
    std::cout << "mass_diagonal ";for(int i=0;i<mass.rows();i++) std::cout << mass(i,i) << " ";std::cout << "\n";
    const auto& query=graph.get_query_output_port().Eval<geometry::QueryObject<double>>(graph_ctx);
    const auto pairs=query.ComputeSignedDistancePairwiseClosestPoints(0.020);
    const auto& inspector=graph.model_inspector();
    int n=0;
    for(const auto& p:pairs){
      auto an=inspector.GetName(p.id_A), bn=inspector.GetName(p.id_B);
      if (an.find("floor")==std::string::npos && bn.find("floor")==std::string::npos) continue;
      std::cout << "ground_pair " << an << " " << bn << " " << p.distance << "\n";
      n++;
    }
    std::cout << "ground_pair_count " << n << "\n";
    if(has_dynamic_input) {
      systems::Simulator<double> sim(*diagram,std::move(context));
      sim.Initialize();
      sim.AdvanceTo(0.002);
      auto& pc=diagram->GetSubsystemContext(plant,sim.get_context());
      const auto& results=plant.get_contact_results_output_port().Eval<multibody::ContactResults<double>>(pc);
      std::cout << "point_contact_count " << results.num_point_pair_contacts() << "\n";
      std::array<std::string,6> legs={"tibiaml","tibiamr","tibiabl","tibiabr","tibiafl","tibiafr"};
      std::array<double,6> Fz={0.,0.,0.,0.,0.,0.};
      for(int i=0;i<results.num_point_pair_contacts();i++){
        const auto& c=results.point_pair_contact_info(i);
        const auto an=plant.get_body(c.bodyA_index()).name();
        const auto bn=plant.get_body(c.bodyB_index()).name();
        for(int leg=0;leg<6;leg++) {
          if(bn==legs[leg]) Fz[leg]+=c.contact_force()[2];
          if(an==legs[leg]) Fz[leg]-=c.contact_force()[2];
        }
      }
      for(int leg=0;leg<6;leg++)std::cout << "drake_fz " << legs[leg] << " " << Fz[leg] << "\n";
    }
  } catch(const std::exception& e){std::cerr<<"DR_G0B_ERROR: "<<e.what()<<"\n"; return 2;}
}
