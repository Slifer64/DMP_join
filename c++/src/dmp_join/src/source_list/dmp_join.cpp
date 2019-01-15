#include <dmp_join/dmp_join.h>

#include <dmp_join/Robot/LWR4p_Robot.h>
#include <dmp_join/Robot/Sim_Robot.h>
#include <dmp_join/Controller/Controller.h>

using namespace as64_;


DmpJoin::DmpJoin()
{
  ros::NodeHandle nh("~");

  gui.reset(new GUI());

  std::string robot_type;
  if (!nh.getParam("robot_type", robot_type)) robot_type="lwr4p";
  if (!robot_type.compare("lwr4p")) robot.reset(new LWR4p_Robot());
  else if (!robot_type.compare("robot_sim")) robot.reset(new Sim_Robot());

  controller.reset(new Controller(robot, gui));
}

DmpJoin::~DmpJoin()
{

}

void DmpJoin::execute()
{
  robot->init();
  robot->setMode(Robot::Mode::IDLE_MODE);

  robot->update();

  while (ros::ok() && !gui->finished())
  {

    if (!robot->isOk())
    {
      gui->enforceState(Ui::ProgramState::PAUSE_PROGRAM);
      gui->printMsg(robot->getErrMsg(), Ui::MSG_TYPE::ERROR);
      robot->enable();
    }
    else robot->command();

    switch (gui->getState())
    {
      case Ui::ProgramState::RUN_CONTROLLER:
        if (robot->getMode() != Robot::Mode::VELOCITY_CONTROL)
        {
          gui->printMsg("Entering RUN_CONTROLLER mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::VELOCITY_CONTROL);
          gui->printModeMsg("== MODE: RUN_CONTROLLER ==");
          gui->printMsg("Mode changed to RUN_CONTROLLER!",Ui::MSG_TYPE::SUCCESS);
          gui->printMsg("Initializing controller...",Ui::MSG_TYPE::INFO);
          if (! controller->initExecution())
          {
            gui->setState(Ui::ProgramState::PAUSE_PROGRAM);
            gui->printMsg(std::string("Failed to initialize the controller: ") + controller->getErrMsg(),Ui::MSG_TYPE::WARNING);
            continue;
          }
          gui->printMsg("Initialized controller successfully! Controller is running...",Ui::MSG_TYPE::INFO);
        }
        controller->execute();
        if (controller->finished()) gui->setState(Ui::ProgramState::PAUSE_PROGRAM);
        break;
      case Ui::ProgramState::FREEDRIVE_MODE:
        if (robot->getMode() != Robot::Mode::FREEDRIVE_MODE)
        {
          gui->printMsg("Entering FREEDRIVE mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::FREEDRIVE_MODE);
          gui->printModeMsg("== MODE: FREEDRIVE ==");
          gui->printMsg("Mode changed to FREEDRIVE!",Ui::MSG_TYPE::SUCCESS);
        }
        break;
      case Ui::ProgramState::PAUSE_PROGRAM:

        if (robot->getMode() != Robot::Mode::IDLE_MODE)
        {
          gui->printMsg("Entering PAUSED mode...",Ui::MSG_TYPE::INFO);
          robot->setMode(Robot::Mode::IDLE_MODE);
          gui->printModeMsg("== MODE: PAUSED ==");
          gui->printMsg("Mode changed to PAUSED!",Ui::MSG_TYPE::SUCCESS);
        }

        if (gui->gotoStartPose()) this->gotoStartPose();

        if (gui->saveControllerData())
        {
          gui->printMsg("Saving execution data...",Ui::MSG_TYPE::INFO);
          if (controller->saveExecutionData()) gui->printMsg("Saved execution data successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(controller->getErrMsg() ,Ui::MSG_TYPE::WARNING);
          gui->resetSaveControllerData();
        }

        if (gui->saveTrainedModel())
        {
          gui->printMsg("Saving trained model...",Ui::MSG_TYPE::INFO);
          if (controller->saveTrainedModel()) gui->printMsg("Trained model saved successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(controller->getErrMsg() ,Ui::MSG_TYPE::WARNING);
          gui->resetSaveTrainedModel();
        }

        if (gui->loadTrainedModel())
        {
          gui->printMsg("Loading trained model...",Ui::MSG_TYPE::INFO);
          if (controller->loadTrainedModel()) gui->printMsg("Trained model loaded successfully!",Ui::MSG_TYPE::SUCCESS);
          else gui->printMsg(controller->getErrMsg() ,Ui::MSG_TYPE::WARNING);
          gui->resetLoadTrainedModel();
        }

        break;
    }

    if (gui->currentPoseAsStart())
    {
      controller->setStartPose();
      gui->printMsg("Registered current pose as start!", Ui::MSG_TYPE::SUCCESS);
      gui->resetCurrentPoseAsStart(); // reset gui flag
    }

    if (gui->printPosition())
    {
      arma::vec p = robot->getTaskPosition();
      std::ostringstream out;
      out << "End-effector position:\n" << p;
      gui->printMsg(out.str().c_str(), Ui::MSG_TYPE::INFO);
      gui->resetPrintPosition();
    }

    robot->update();
  }

}

void DmpJoin::gotoStartPose()
{
  PRINT_INFO_MSG("Moving to start pose...\n");
  gui->printMsg("Moving to start pose...", Ui::MSG_TYPE::INFO);

  robot->update();
  arma::vec q_current = robot->getJointPosition();
  double duration = std::max(arma::max(arma::abs(controller->q_start-q_current))*7.0/arma::datum::pi,2.0);
  robot->setJointTrajectory(controller->q_start, duration);
  robot->update();

  gui->resetGotoStartPose(); // reset gui flag

  q_current = robot->getJointPosition();
  if (arma::norm(q_current-controller->q_start) < 5e-3)
  {
    PRINT_CONFIRM_MSG("Reached start pose!\n");
    gui->printMsg("Reached start pose!", Ui::MSG_TYPE::SUCCESS);
  }
  else
  {
    PRINT_WARNING_MSG("Failed to reach start pose...\n");
    gui->printMsg("Failed to reach start pose...\n", Ui::MSG_TYPE::WARNING);
  }

}
