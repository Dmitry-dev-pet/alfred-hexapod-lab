"""Patch only the *local pinned source copy* of TRI IDTO's examples/example_base.cc.

Adds an optional externally supplied nominal trajectory and initial guess,
plus a measurement-only state export. Does not change the optimizer algorithms.
Never run against the owner's upstream source unconditionally: explicit --source.
"""
import argparse
import shutil
from pathlib import Path

HELPER = r'''
// Optional research-mode time-varying reference for CAD30 only.
// Each row holds nq finite q coordinates, one row for each optimizer node.
static std::vector<VectorXd> Cad30LoadWholeCycleCsv(
    const char* env_key, int expected_rows, int nq) {
  const char* value = std::getenv(env_key);
  if (value == nullptr || value[0] == '\0') return {};
  std::ifstream in(value);
  if (!in) throw std::runtime_error(std::string("Cannot read ") + env_key);
  std::vector<VectorXd> result;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);
    std::string field;
    VectorXd row(nq);
    int col = 0;
    while (std::getline(ss, field, ',')) {
      if (col >= nq) throw std::runtime_error("Too many CSV columns");
      size_t consumed = 0;
      double q = std::stod(field, &consumed);
      if (consumed == 0 || !std::isfinite(q))
        throw std::runtime_error("Nonfinite IDTO reference CSV value");
      row[col++] = q;
    }
    if (col != nq) throw std::runtime_error("Bad IDTO reference CSV columns");
    result.push_back(row);
  }
  if (static_cast<int>(result.size()) != expected_rows)
    throw std::runtime_error("Bad IDTO reference CSV row count");
  return result;
}

static void Cad30WriteCsv(const char* path, const std::vector<VectorXd>& data) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("Cannot write IDTO trajectory CSV");
  out << std::setprecision(16);
  for (const auto& row : data) {
    for (int j=0;j<row.size();++j) {
      if (j) out << ',';
      out << row[j];
    }
    out << '\n';
  }
}
'''

def patch(source):
    p=Path(source)
    s=p.read_text()
    if 'Cad30LoadWholeCycleCsv' in s:
        raise RuntimeError('Already patched')
    inc='#include <chrono>\n'
    if s.count(inc)!=1:raise ValueError('include anchor mismatch')
    s=s.replace(inc,inc+'#include <cstdlib>\n#include <cmath>\n#include <fstream>\n#include <stdexcept>\n')
    anchor='void TrajOptExample::RunExample(const std::string options_file,'
    if s.count(anchor)!=1:raise ValueError('helper anchor mismatch')
    s=s.replace(anchor,HELPER+'\n'+anchor)
    anchor='  NormalizeQuaternions(plant, &q_guess);\n'
    addition='''  auto external_guess = Cad30LoadWholeCycleCsv("CAD30_G1_NOMINAL_CSV", opt_prob.num_steps+1, nq);
  if (!external_guess.empty()) {
    q_guess = external_guess;
    NormalizeQuaternions(plant, &q_guess);
  }
'''
    if s.count(anchor)!=1:raise ValueError('guess anchor mismatch')
    s=s.replace(anchor,anchor+addition)
    anchor='  // Report maximum torques on all DoFs\n'
    addition='''  if (const char* path = std::getenv("CAD30_G1_OUTPUT_PREFIX")) {
    Cad30WriteCsv((std::string(path)+"_q.csv").c_str(), solution.q);
    Cad30WriteCsv((std::string(path)+"_v.csv").c_str(), solution.v);
    Cad30WriteCsv((std::string(path)+"_tau.csv").c_str(), solution.tau);
  }
'''
    if s.count(anchor)!=1:raise ValueError('output anchor mismatch')
    s=s.replace(anchor,addition+anchor)
    anchor='  opt_prob->q_nom =\n      MakeLinearInterpolation(q_nom_start, q_nom_end, options.num_steps + 1);\n'
    addition='''  auto external_nominal = Cad30LoadWholeCycleCsv("CAD30_G1_NOMINAL_CSV",options.num_steps+1, options.q_init.size());
  if (!external_nominal.empty()) {
    if ((external_nominal[0]-options.q_init).norm()>1e-8)
       throw std::runtime_error("IDTO external reference initial q != initial checkpoint");
    opt_prob->q_nom = external_nominal;
  }
'''
    if s.count(anchor)!=1:raise ValueError('nominal anchor mismatch')
    s=s.replace(anchor,anchor+addition)
    backup=p.with_suffix(p.suffix + ".before_cad30_g1")
    if backup.exists():
        raise RuntimeError(f"Existing backup {backup}; refusing to overwrite")
    shutil.copy2(p,backup)
    p.write_text(s)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--source',required=True);x=parser.parse_args()
    patch(x.source)