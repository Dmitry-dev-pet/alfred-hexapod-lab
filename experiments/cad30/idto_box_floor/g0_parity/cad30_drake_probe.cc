// Compare a native Drake-imported MJCF CAD30 with MuJoCo, without embedding STL/OBJ.
// Usage: cad30_drake_probe path/to/scene_local30_nominal.xml > drake_probe.txt 2>parser_warnings.log
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/scene_graph.h>
#include <exception>
#include <iomanip>
#include <iostream>

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: cad30_drake_probe path/to/model.xml\n";
    return 1;
  }
  try {
    drake::multibody::MultibodyPlant<double> plant(0.0);
    drake::geometry::SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
    drake::multibody::Parser parser(&plant, &scene_graph);
    parser.AddModels(argv[1]);
    plant.Finalize();
    auto context = plant.CreateDefaultContext();
    std::cout << std::setprecision(17);
    std::cout << "meta " << plant.num_positions() << " " << plant.num_velocities()
              << " " << plant.num_bodies() << " " << plant.num_actuators()
              << " " << plant.CalcTotalMass(*context) << "\n";
    auto com = plant.CalcCenterOfMassPositionInWorld(*context);
    std::cout << "com " << com.x() << " " << com.y() << " " << com.z() << "\n";
    for (int i=0; i<plant.num_joints(); i++) {
      const auto& joint=plant.get_joint(drake::multibody::JointIndex(i));
      if (joint.num_positions() == 1)
        std::cout << "joint " << joint.name() << " " << joint.position_start()
                  << " " << joint.velocity_start() << "\n";
    }
    for (int i=0; i<plant.num_actuators(); i++) {
      const auto& a=plant.get_joint_actuator(drake::multibody::JointActuatorIndex(i));
      std::cout << "actuator " << a.name() << " " << a.joint().name()
                << " " << a.effort_limit() << "\n";
    }
    auto q=plant.GetPositions(*context);
    std::cout << "q0 "; for (int i=0;i<q.size();i++) std::cout << q[i] << " "; std::cout << "\n";
    Eigen::MatrixXd M(plant.num_velocities(), plant.num_velocities());
    plant.CalcMassMatrix(*context, &M);
    for (int i=0; i<M.rows(); i++) {
      std::cout << "mrow " << i << " ";
      for (int j=0;j<M.cols();j++) std::cout << M(i,j) << " ";
      std::cout << "\n";
    }
    auto gf=plant.CalcGravityGeneralizedForces(*context);
    std::cout << "gravity "; for (int i=0;i<gf.size();i++) std::cout << gf[i] << " ";
    std::cout << "\n";
    for (const char* name : {"tibiaml", "tibiamr", "tibiabl", "tibiabr", "tibiafl", "tibiafr"}) {
      if (!plant.HasBodyNamed(name)) continue;
      const auto& body=plant.GetBodyByName(name);
      const auto xyz=plant.CalcRelativeTransform(*context, plant.world_frame(), body.body_frame()).translation();
      std::cout << "bodyxyz " << name << " " << xyz.x() << " " << xyz.y() << " " << xyz.z() << "\n";
    }
    std::cout << "geometry " << scene_graph.model_inspector().num_geometries()
              << " " << scene_graph.model_inspector().GetAllGeometryIds(drake::geometry::Role::kProximity).size() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "PARSER_FAIL: " << e.what() << "\n";
    return 2;
  }
  return 0;
}
