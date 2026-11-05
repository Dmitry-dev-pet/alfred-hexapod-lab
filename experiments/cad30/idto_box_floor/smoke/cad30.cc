// An isolated IDTO smoke adapter; not a gait controller or a certified walk.
// The private CAD30 meshes and MJCF never belong in the public source tree.
#include "examples/example_base.h"
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>
#include <cstdlib>
#include <stdexcept>
#include <string>

DEFINE_bool(test, false, "Run one short trajectory solve (no MPC)");

class Cad30Example : public idto::examples::TrajOptExample {
 private:
  void CreatePlantModel(drake::multibody::MultibodyPlant<double>* plant) const override {
    const char* path=std::getenv("CAD30_DRAKE_MJCF");
    if (path==nullptr || path[0]=='\0') throw std::runtime_error("Missing CAD30_DRAKE_MJCF env");
    drake::multibody::Parser(plant).AddModels(path);
  }
};

int main(int argc,char** argv){
  gflags::ParseCommandLineFlags(&argc,&argv,true);
  Cad30Example().RunExample("idto/examples/cad30/cad30.yaml",FLAGS_test);
}