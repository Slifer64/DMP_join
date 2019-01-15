#include <dmp_join/GUI/GUI.h>
#include <QApplication>

GUI::GUI()
{
  gui_finished = false;

  gui_thread.reset(new std::thread(&GUI::guiThread,this));
  gui_thread->detach(); // otherwise qt issues warnings

  std::mutex mtx;
  std::unique_lock<std::mutex> lck(mtx);
  start_cond.wait(lck); // wait for gui to start
}

GUI::~GUI()
{
  q_app->exit(0);
  // if (gui_thread->joinable()) gui_thread->join();
}

int GUI::guiThread()
{
  int argc = 0;
  char **argv = NULL;
  q_app.reset(new QApplication (argc, argv));
  gui_obj.reset(new MainWindow);
  gui_obj->show();

  gui_obj->setMsg("MODE initialized to IDLE", Ui::MSG_TYPE::INFO);
  gui_obj->setModeMsg("== MODE set to IDLE ==");

  start_cond.notify_one(); // the gui has started! so unblock the main thread

  int ret_val = q_app->exec();

  gui_finished = true;

  return ret_val;
}
