#include <dmp_join/Controller/Controller.h>
#include <ros/package.h>
#include <io_lib/io_lib.h>

Controller::Controller(std::shared_ptr<Robot> &robot, std::shared_ptr<GUI> &gui)
{
  this->robot = robot;
  this->gui = gui;
  is_trained = false;
  loaded_training_data = false;
  setErrMsg("");

  setStartPose();

  readControllerParams();
  readTrainingParams();

  can_clock_ptr.reset(new as64_::CanonicalClock(1.0));
  shape_attr_gating_ptr.reset(new as64_::SigmoidGatingFunction(1.0, 0.97));

  robot->update();
  q_start = robot->getJointPosition();
}

Controller::~Controller()
{

}

void Controller::readTrainingParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Controller_config.yml";
  as64_::io_::Parser parser(path_to_config_file);

  // train data params
  if (!parser.getParam("N_kernels", N_kernels)) N_kernels = 30;
  if (!parser.getParam("train_method", train_method)) train_method = "LWR";
  if (!parser.getParam("a_z", a_z)) a_z = 15;
  if (!parser.getParam("b_z", b_z)) b_z = a_z/4;
}

bool Controller::readControllerParams(const char *params_file)
{
  std::string path_to_config_file;
  if (params_file != NULL) path_to_config_file = *params_file;
  else path_to_config_file = ros::package::getPath(PACKAGE_NAME)+ "/config/Controller_config.yml";
  as64_::io_::Parser parser(path_to_config_file);

  if (!parser.getParam("T_total", T_total)) T_total = 10.0;

  arma::mat goals_mat;
  if (!parser.getParam("goals", goals_mat))
  {
    setErrMsg("readControllerParams: Couldn't read \"goals\" parameter");
    return false;
  }
  for (int i=0; i<goals_mat.n_cols; i++) goals.push_back(goals_mat.col(i));

  if (!parser.getParam("goal_switch_times", goal_switch_times))
  {
    setErrMsg("readControllerParams: Couldn't read \"goal_switch_times\" parameter");
    return false;
  }

  if (!parser.getParam("goal_switch_ind", goal_switch_ind))
  {
    setErrMsg("readControllerParams: Couldn't read \"goal_switch_ind\" parameter");
    return false;
  }

  if (!parser.getParam("dmp_switch_ind", dmp_switch_ind))
  {
    setErrMsg("readControllerParams: Couldn't read \"dmp_switch_ind\" parameter");
    return false;
  }

  if (!parser.getParam("as0", as0)) as0 = 4;
  if (!parser.getParam("kas", kas)) kas = 4.5;
  if (!parser.getParam("as_max", as_max)) as_max = 150;

  if (!parser.getParam("k_click", k_click)) k_click = 1.0;

  return true;
}

bool Controller::loadTrainingData()
{
  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/training_data.bin";
  bool binary = true;

  std::ifstream in(data_file.c_str(), std::ios::binary);
  if (!in)
  {
    setErrMsg("Error loading training data:\nCouldn't open file: \"" + data_file + "\"");
    return false;
  }

  long N_dmps;
  as64_::io_::read_scalar(N_dmps, in, binary);

  train_data.resize(N_dmps);

  for (int i=0; i<N_dmps; i++)
  {
    as64_::io_::read_mat(q_start, in, binary);
    as64_::io_::read_mat(train_data[i].Time, in, binary);
    as64_::io_::read_mat(train_data[i].Y_data, in, binary);
    as64_::io_::read_mat(train_data[i].dY_data, in, binary);
    as64_::io_::read_mat(train_data[i].ddY_data, in, binary);
  }

  in.close();

  loaded_training_data = true;

  return true;
}

bool Controller::train()
{
  if (!loaded_training_data)
  {
    loaded_training_data = loadTrainingData();
    if (loaded_training_data == false) return false;
  }

  readTrainingParams();

  dmp_vec.clear();
  int N_dmps = train_data.size();
  dmp_vec.resize(N_dmps);



  for (int k=0; k<N_dmps; k++)
  {
    if (train_data[k].isempty())
    {
      std::ostringstream dmp_num;
      dmp_num << k+1;
      setErrMsg("Error training model: The training data for dmp " + dmp_num.str() + " are empty...");
      return false;
    }

    int dim = train_data[k].Y_data.n_rows;
    dmp_vec[k].resize(dim);
    arma::vec train_err(dim);
    for (int i=0; i<dim; i++)
    {
      dmp_vec[k][i].reset(new as64_::DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr));
      train_err(i) = dmp_vec[k][i]->train(train_method, train_data[k].Time, train_data[k].Y_data.row(i), train_data[k].dY_data.row(i), train_data[k].ddY_data.row(i), true);
    }
    std::ostringstream out;
    out << "Training error:\n" << train_err;
    gui->printMsg(out.str().c_str(), Ui::MSG_TYPE::INFO);
  }

  is_trained = true;

  return true;
}

void Controller::setStartPose()
{
  robot->update();
  q_start = robot->getJointPosition();
}

bool Controller::saveExecutionData()
{
  if (exec_data.isempty())
  {
    setErrMsg("Error saving execution data: The data are empty...");
    return false;
  }

  std::string file_name = "execution_data.bin";

  std::string err_msg;
  bool save_success = exec_data.save(file_name, err_msg);
  setErrMsg(err_msg);
  return save_success;
}

bool Controller::initExecution()
{
  if (!is_trained)
  {
    setErrMsg("The model has not been trained");
    return false;
  }

  if (!readControllerParams())
  {
    return false;
  }

  if (gui->logControllerData()) exec_data.clear();

  this->robot->init();

  arma::vec p = this->robot->getTaskPosition();

  // controller variables
  Y0 = p;
  Y = p;
  dY.zeros(3);
  ddY.zeros(3);

  Y_ref = Y;
  dY_ref = dY;
  ddY_ref = ddY;

  t = 0.0;
  double x = 0.0;
  tau = T_total;
  can_clock_ptr->setTau(T_total);

  i_switch = 0;
  // Yg = goals[i_switch];

  Ts = robot->getControlCycle();

  reached_target = false;

  return true;
}

void Controller::execute()
{
  // Stopping criteria
  if (arma::norm(Y-Yg)<5e-3 && arma::norm(dY)<5e-3)
  {
    reached_target = true;
    // return;
  }

  // check for goal change
  checkGoalChange();

  // log data
  if (gui->logControllerData()) exec_data.log(t, Y, dY, ddY, Y_ref, dY_ref, ddY_ref);

  // DMP2 simulation
  for (int i=0; i<dmp.size(); i++)
  {
    ddY_ref(i) = dmp[i]->getAccel(Y_ref(i), dY_ref(i), Y0(i), 0.0, 0.0, x, Yg(i), tau);
  }

  // DMP-filt simulation
  double Ms = 3*as;
  double Ds = 3*std::pow(as,2);
  double Ks = std::pow(as,3);
  arma::vec dddY = Ms*(ddY_ref - ddY) + Ds*(dY_ref - dY) + Ks*(Y_ref - Y);

  // Update phase variable
  double dx = can_clock_ptr->getPhaseDot(x);

  // command robot
  arma::vec Y_robot = this->robot->getTaskPosition();
  arma::vec V_cmd = arma::vec().zeros(6);
  V_cmd.subvec(0,2) = dY + k_click*(Y-Y_robot);
  robot->setTaskVelocity(V_cmd);

  // ========  numerical integration  ========
  t = t + Ts;
  x = x + dx*Ts;

  Y_ref = Y_ref + dY_ref*Ts;
  dY_ref = dY_ref + ddY_ref*Ts;

  Y = Y + dY*Ts;
  dY = dY + ddY*Ts;
  ddY = ddY + dddY*Ts;

  if (as < as_max) as = as + kas*as*Ts;
}

void Controller::checkGoalChange()
{
  if ( ( i_switch<goal_switch_times.size() && std::abs(t-goal_switch_times[i_switch])<Ts/2 ) ||
       ( std::abs(t-goal_switch_times[i_switch])==Ts/2 && t>goal_switch_times[i_switch] )
     )
  {
        // move to next goal
        Yg = goals[i_switch];

        int Dim = dmp_vec[i_switch].size();
        dmp.resize(Dim);
        for (int i=0; i<Dim; i++) dmp[i] = dmp_vec[i_switch][i];

        // reset init conditions
        Y0 = Y;
        x = 0.0;
        double t_rem = T_total - t;
        tau = t_rem;
        can_clock_ptr->setTau(tau);

        // switch for DMP-filt
        as = as0;

        // switch for DMP3
        // for i=1:Dim
        //    g3(i) = ( tau^2*ddy(i) + dmp{i}.a_z*tau*dy(i) - dmp{i}.shapeAttractor(x, y0(i), g(i)) )/(dmp{i}.a_z*dmp{i}.b_z) + y(i);
        // end

        // switch for DMP2a
        // for i=1:Dim
        //     dmp2a{i} = dmp{i}.copy();
        //     psi = dmp2a{i}.kernelFunction(x);
        //     psi = psi/sum(psi);
        //     psi = dmp2a{i}.shapeAttrGating(x)*dmp2a{i}.forcingTermScaling(y0(i),g(i)) * psi;
        //     f = dmp2a{i}.shapeAttractor(x, y0(i), g(i));
        //     fd = tau^2*ddy(i) + tau*dmp2a{i}.a_z*dy(i) - dmp2a{i}.a_z*dmp2a{i}.b_z*(g(i)-y(i));
        //     dw = pinv(psi) * ( fd - f );
        //     dmp2a{i}.w = dmp2a{i}.w + dw(:);
        // end

        i_switch++;
    }
}

bool Controller::saveTrainedModel()
{
  if (!is_trained)
  {
    setErrMsg("Error saving the model:\nThe model hasn't been trained...");
    return false;
  }

  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/model_data.bin";
  bool binary = true;

  std::ofstream out(data_file.c_str(), std::ios::binary);
  if (!out)
  {
    setErrMsg("Error saving the model:\nCouldn't create file \"" + data_file + "\"...");
    return false;
  }

  long N_dmps = dmp_vec.size();

  as64_::io_::write_scalar((long)N_dmps, out, binary);

  for (int k=0; k<N_dmps; k++)
  {
    int dim = dmp.size();
    as64_::io_::write_scalar((long)dim, out, binary);
    for (int i=0; i<dim; i++)
    {
      arma::vec dmp_params(3);
      dmp_params(0) = dmp[i]->N_kernels;
      dmp_params(1) = dmp[i]->a_z;
      dmp_params(2) = dmp[i]->b_z;
      dmp_params = arma::join_vert(dmp_params, dmp[i]->w);
      as64_::io_::write_mat(dmp_params, out, binary);
    }
  }

  out.close();

  return true;
}

bool Controller::loadTrainedModel()
{
  std::string data_file = ros::package::getPath(PACKAGE_NAME)+ "/data/model_data.bin";
  bool binary = true;

  std::ifstream in(data_file.c_str(), std::ios::binary);
  if (!in)
  {
    setErrMsg("Error loading the model:\nCouldn't open file: \"" + data_file + "\"");
    return false;
  }

  long N_dmps;

  as64_::io_::read_scalar(N_dmps, in, binary);

  for (int k=0; k<N_dmps; k++)
  {
    long dim;
    as64_::io_::read_scalar(dim, in, binary);
    dmp.resize(dim);
    for (int i=0; i<dim; i++)
    {
      arma::vec dmp_params;
      as64_::io_::read_mat(dmp_params, in, binary);
      int i_end = dmp_params.size()-1;
      int N_kernels = dmp_params(0);
      double a_z = dmp_params(1);
      double b_z = dmp_params(2);

      dmp[i].reset(new as64_::DMP(N_kernels, a_z, b_z, can_clock_ptr, shape_attr_gating_ptr));
      dmp[i]->w = dmp_params.subvec(3,i_end);
    }
  }

  in.close();

  is_trained = true;

  return true;
}
