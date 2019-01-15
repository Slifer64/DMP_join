#ifndef OL_2D_RUP_GUI_H
#define OL_2D_RUP_GUI_H

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <vector>
#include <cstring>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <mainwindow.h>

#include <armadillo>

class GUI
{
public:

  GUI();
  ~GUI();

  int guiThread();

  bool printPosition() const { return gui_obj->print_pos; }
  void resetPrintPosition() { gui_obj->print_pos = false; }

  bool trainModel() const { return gui_obj->train_model; }
  bool resetTrainModel() { gui_obj->train_model = false; }

  bool saveControllerData() const { return gui_obj->save_controller_data; }
  void resetSaveControllerData() { gui_obj->save_controller_data = false; }
  bool logControllerData() const { return gui_obj->log_controller; }

  bool gotoStartPose() const { return gui_obj->goto_start_pose; }
  void resetGotoStartPose() { gui_obj->goto_start_pose = false; }

  bool currentPoseAsStart() const { return gui_obj->current_pose_as_start; }
  void resetCurrentPoseAsStart() { gui_obj->current_pose_as_start = false; }

  bool saveTrainedModel() const { return gui_obj->save_trained_model; }
  void resetSaveTrainedModel() { gui_obj->save_trained_model = false; }

  bool loadTrainedModel() const { return gui_obj->load_trained_model; }
  void resetLoadTrainedModel() { gui_obj->load_trained_model = false; }

  bool loadTrainingData() const { return gui_obj->load_training_data; }
  void resetLoadTrainingData() { gui_obj->load_training_data = false; }

  Ui::ProgramState getState() const { return gui_obj->state; }
  void setState(const Ui::ProgramState &state) { std::thread(&MainWindow::setState, gui_obj.get(), state).detach(); }
  void enforceState(const Ui::ProgramState &state) { gui_obj->enforceState(state); }

  void printMsg(const std::string &msg, Ui::MSG_TYPE msg_type=Ui::MSG_TYPE::INFO) { std::thread(&MainWindow::setMsg, gui_obj.get(), msg, msg_type).detach(); }
  void printModeMsg(const std::string &msg) { std::thread(&MainWindow::setModeMsg, gui_obj.get(), msg).detach(); }

  std::shared_ptr<MainWindow> gui_obj;

  bool finished() const { return gui_finished; }
private:
  std::shared_ptr<std::thread> gui_thread;


  std::shared_ptr<QApplication> q_app;

  bool gui_finished;
  std::condition_variable start_cond;
};

#endif // OL_2D_RUP_GUI_H
