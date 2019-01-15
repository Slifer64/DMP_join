#ifndef DMP_CONTROLLER_H
#define DMP_CONTROLLER_H

#include <cstdlib>
#include <vector>
#include <memory>
#include <exception>

#include <dmp_lib/dmp_lib.h>
#include <io_lib/io_lib.h>
#include <math_lib/math_lib.h>

#include <dmp_join/utils.h>
#include <dmp_join/Robot/Robot.h>
#include <dmp_join/GUI/GUI.h>

#include <dmp_join/LogData.h>


class Controller
{
public:

  Controller(std::shared_ptr<Robot> &robot, std::shared_ptr<GUI> &gui);
  ~Controller();

  bool initExecution();
  void execute();
  void checkGoalChange();
  bool saveExecutionData();

  bool loadTrainingData();
  bool train();

  bool loadTrainedModel();
  bool saveTrainedModel();

  bool readControllerParams(const char *params_file=NULL);
  void readTrainingParams(const char *params_file=NULL);

  bool finished() const { return reached_target; }

  void setStartPose();

  std::string getErrMsg() const { return err_msg; }


  double Ts;

  bool is_trained;
  bool  loaded_training_data;
  bool reached_target;

  arma::vec q_start; ///< starting pose
  arma::vec Yg; ///< goal position
  double tau;

  double t; // current timestamp during controller execution
  double x; // phase variable
  arma::vec Y0; // initial position
  arma::vec Y, dY, ddY; // smoothed DMP output
  arma::vec Y_ref, dY_ref, ddY_ref; // DMP output

  int N_kernels;
  double a_z;
  double b_z;
  std::string train_method;
  std::shared_ptr<as64_::CanonicalClock> can_clock_ptr;
  std::shared_ptr<as64_::GatingFunction> shape_attr_gating_ptr;
  std::vector<std::vector<std::shared_ptr<as64_::DMP>>> dmp_vec;
  std::vector<std::shared_ptr<as64_::DMP>> dmp;

  std::vector<KinematicData> train_data;
  std::vector<arma::vec> goals;
  double T_total;
  arma::vec goal_switch_times;
  arma::uvec goal_switch_ind;
  arma::uvec dmp_switch_ind;
  int i_switch;

  ExecutionData exec_data;

  // DMP-filt params
  double as;
  double as0;
  double kas;
  double as_max;

  double k_click;

protected:

  void setErrMsg(const std::string &msg) { err_msg = msg; }
  std::string err_msg;

  std::shared_ptr<Robot> robot;
  std::shared_ptr<GUI> gui;
};

#endif // DMP_CONTROLLER_H
