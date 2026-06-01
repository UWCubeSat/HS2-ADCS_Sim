#include "adcs/adcs_core.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace {

adcs::Vec3 as_vec3(const std::vector<double>& v, const char* name) {
  if (v.size() != 3) {
    throw std::runtime_error(std::string(name) + " must have length 3");
  }
  return adcs::Vec3{v[0], v[1], v[2]};
}

py::dict step_py(double time_s,
                 const std::vector<double>& omega_B_rad_s,
                 const std::vector<double>& mag_B_T,
                 py::dict config_dict = py::dict()) {
  adcs::ControllerConfig cfg{};
  if (config_dict.contains("dipoleCommandGain")) cfg.dipoleCommandGain = config_dict["dipoleCommandGain"].cast<double>();
  if (config_dict.contains("mtqDipoleGain_Am2_A")) cfg.mtqDipoleGain_Am2_A = as_vec3(config_dict["mtqDipoleGain_Am2_A"].cast<std::vector<double>>(), "mtqDipoleGain_Am2_A");
  if (config_dict.contains("mtqResistance_Ohm")) cfg.mtqResistance_Ohm = as_vec3(config_dict["mtqResistance_Ohm"].cast<std::vector<double>>(), "mtqResistance_Ohm");
  if (config_dict.contains("mtqCurrentLimit_A")) cfg.mtqCurrentLimit_A = as_vec3(config_dict["mtqCurrentLimit_A"].cast<std::vector<double>>(), "mtqCurrentLimit_A");
  if (config_dict.contains("mtqDipoleLimit_Am2")) cfg.mtqDipoleLimit_Am2 = as_vec3(config_dict["mtqDipoleLimit_Am2"].cast<std::vector<double>>(), "mtqDipoleLimit_Am2");
  if (config_dict.contains("minMagField_T")) cfg.minMagField_T = config_dict["minMagField_T"].cast<double>();

  adcs::StepInput input{};
  input.time_s = time_s;
  input.omega_B_rad_s = as_vec3(omega_B_rad_s, "omega_B_rad_s");
  input.mag_B_T = as_vec3(mag_B_T, "mag_B_T");

  const adcs::StepOutput out = adcs::step(input, cfg);
  py::dict d;
  d["commanded_magnetic_dipole_B_Am2"] = out.commandedDipole_B_Am2;
  d["commanded_coil_current_A"] = out.commandedCurrent_A;
  d["commanded_control_torque_B_Nm"] = out.commandedTorque_B_Nm;
  d["coil_power_W"] = out.coilPower_W;
  d["coil_power_total_W"] = out.coilPowerTotal_W;
  d["saturation_flags"] = out.saturation;
  d["valid"] = out.valid;
  return d;
}

}  // namespace

PYBIND11_MODULE(adcs_core, m) {
  m.doc() = "Small standalone ADCS controller core for Basilisk adapter use.";
  m.def("step", &step_py, py::arg("time_s"), py::arg("omega_B_rad_s"), py::arg("mag_B_T"), py::arg("config") = py::dict());
}
