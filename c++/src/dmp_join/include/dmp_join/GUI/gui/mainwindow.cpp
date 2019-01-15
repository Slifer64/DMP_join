#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  this->parent = parent;

  app_name = "Robot Controller GUI";

  stateName.resize(3);
  stateName[0] = "RUN_CONTROLLER";
  stateName[1] = "FREEDRIVE_MODE";
  stateName[2] = "PAUSE_PROGRAM";

  info_color = QColor(0, 0, 255); // blue
  warn_color = QColor(200, 100, 0); // yellow-orange
  err_color = QColor(255, 0, 0); // red
  success_color = QColor(0, 125, 0); // green
  label_bg_clolor = QColor(238, 238, 236);

  msg_label_font = QFont("DejaVu Serif", 14);

  mode_label_font = QFont("DejaVu Serif", 16);
  mode_label_font.setBold(true);

  init();

  std::string cmd_btn_stylesheet = "font: 75 14pt \"Ubuntu Mono\";";
  std::string push_btn_stylesheet = "font: 75 14pt \"Ubuntu Mono\";";
  ui->freedrive_mode_button->setStyleSheet(cmd_btn_stylesheet.c_str());
  ui->pause_program_button->setStyleSheet(cmd_btn_stylesheet.c_str());
  ui->run_controller_button->setStyleSheet(cmd_btn_stylesheet.c_str());
  ui->load_trained_model_btn->setStyleSheet(push_btn_stylesheet.c_str());
  ui->save_trained_model_btn->setStyleSheet(push_btn_stylesheet.c_str());
  ui->save_controller_data_btn->setStyleSheet(push_btn_stylesheet.c_str());
  ui->load_training_data_btn->setStyleSheet(push_btn_stylesheet.c_str());
  ui->train_model_btn->setStyleSheet(push_btn_stylesheet.c_str());
  ui->move_to_start_btn->setStyleSheet(push_btn_stylesheet.c_str());
  ui->register_startPose_btn->setStyleSheet(push_btn_stylesheet.c_str());
  ui->print_pos_btn->setStyleSheet(push_btn_stylesheet.c_str());
  ui->controller_log_checkbox->setStyleSheet(push_btn_stylesheet.c_str());
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::init()
{
  this->setWindowTitle(QApplication::translate("Robot Control", "Robot Control", 0));

  state = Ui::ProgramState::PAUSE_PROGRAM;

  print_pos = false;

  save_trained_model = false;
  load_trained_model = false;
  load_training_data = false;

  log_controller = false;
  save_controller_data = false;

  current_pose_as_start = false;
  goto_start_pose = false;

  train_model = false;
  model_trained = false;

  ui->msg_label->setWordWrap(true);
  ui->msg_label->setAutoFillBackground(true);
  ui->msg_label->setFont(msg_label_font);

  ui->msg_label2->setWordWrap(true);
  ui->msg_label2->setAutoFillBackground(true);
  ui->msg_label2->setFont(msg_label_font);

  ui->msg_label3->setWordWrap(true);
  ui->msg_label3->setAutoFillBackground(true);
  ui->msg_label3->setFont(msg_label_font);

  ui->mode_msg->setWordWrap(true);
  ui->mode_msg->setAutoFillBackground(true);
  ui->mode_msg->setFont(mode_label_font);

  setStyleSheet(ui->mode_msg, "QLabel { background-color : rgb(238, 238, 236)}");
  setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236)}"); //; color : blue; font: 75 12pt \"DejaVu Serif\"}");
  setStyleSheet(ui->msg_label2, "QLabel { background-color : rgb(238, 238, 236)}"); //; color : blue; font: 75 12pt \"DejaVu Serif\"}");
  setStyleSheet(ui->msg_label3, "QLabel { background-color : rgb(238, 238, 236)}"); //; color : blue; font: 75 12pt \"DejaVu Serif\"}");

  setMsg("", Ui::MSG_TYPE::INFO);
  setMsg("", Ui::MSG_TYPE::INFO);
  setMsg("", Ui::MSG_TYPE::INFO);
  setModeMsg("MODE: " + getModeName());
}

void MainWindow::setStyleSheet(QAbstractButton *btn, const std::string &style_sheet)
{
  btn->setStyleSheet(style_sheet.c_str());
  btn->style()->unpolish(btn);
  btn->style()->polish(btn);
  btn->update();
}

void MainWindow::setStyleSheet(QLabel *label, const QString &style_sheet)
{
  label->setStyleSheet(style_sheet);
  label->style()->unpolish(label);
  label->style()->polish(label);
  label->update();
}

void MainWindow::enforceState(const Ui::ProgramState &new_state)
{
  state = new_state;
}

void MainWindow::setState(const Ui::ProgramState &new_state)
{
  std::unique_lock<std::mutex> lck(state_mtx);

  if (getState() == new_state)
  {
    displayWarningMsg("Mode is already in \"" + getModeName(new_state) + "\"");
    return;
  }

  if (getState()==Ui::ProgramState::PAUSE_PROGRAM || new_state==Ui::ProgramState::PAUSE_PROGRAM)
  {
    state = new_state;
  }
  else
  {
    std::string msg = std::string("Current mode is \"") + getModeName() + "\"\nDo you wish to change mode to \"" + getModeName(Ui::ProgramState::RUN_CONTROLLER) + "\"?";
    if (switchModeQuestionBox(msg)) state = new_state;
  }
}

Ui::ProgramState MainWindow::getState()
{
  return state;
}

std::string MainWindow::getModeName(Ui::ProgramState mode)
{
    if ((int)mode < 0) return stateName[(int)getState()];
    else return stateName[(int)mode];
}

void MainWindow::on_freedrive_mode_button_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);
  setState(Ui::ProgramState::FREEDRIVE_MODE);
}

void MainWindow::on_pause_program_button_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);
  setState(Ui::ProgramState::PAUSE_PROGRAM);
}

void MainWindow::on_run_controller_button_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);
  setState(Ui::ProgramState::RUN_CONTROLLER);
}

void MainWindow::on_move_to_start_btn_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
  {
    displayWarningMsg("The program must be in \"" + getModeName(Ui::PAUSE_PROGRAM) + "\" mode to go to the start pose.");
  }
  else
  {
    goto_start_pose = true;
  }
}

void MainWindow::on_register_startPose_btn_clicked()
{
  std::unique_lock<std::mutex> lck(btn_click_mtx);

  current_pose_as_start = true;

  // setMsg("Registered current pose as start.", Ui::MSG_TYPE::INFO);
}

void MainWindow::on_train_model_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      displayWarningMsg("The program must be in \"" + getModeName(Ui::PAUSE_PROGRAM) + "\" mode to train the model.");
    }
    else
    {
      train_model = true;
      model_trained = true;
    }
}

void MainWindow::on_save_trained_model_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      displayWarningMsg("The program must be in \"" + getModeName(Ui::PAUSE_PROGRAM) + "\" mode to save the trained model.");
    }
    else
    {
      save_trained_model = true;
    }
}

void MainWindow::on_load_trained_model_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      displayWarningMsg("The program must be in \"" + getModeName(Ui::PAUSE_PROGRAM) + "\" mode to load the trained model.");
    }
    else
    {
      load_trained_model = true;
      model_trained = true;
    }
}

void MainWindow::on_load_training_data_btn_clicked()
{
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      displayWarningMsg("The program must be in \"" + getModeName(Ui::PAUSE_PROGRAM) + "\" mode to load the training data.");
    }
    else
    {
      load_training_data = true;
    }
}

void MainWindow::on_controller_log_checkbox_toggled(bool checked)
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);

    log_controller = checked;
    std::string msg = (checked?"Enabled":"Disabled");
    msg += " controller data logging.";
    setMsg(msg.c_str(), Ui::MSG_TYPE::INFO);
}

void MainWindow::on_save_controller_data_btn_clicked()
{
    std::unique_lock<std::mutex> lck(btn_click_mtx);
    if (getState() != Ui::ProgramState::PAUSE_PROGRAM)
    {
      displayWarningMsg("The program must be in \"" + getModeName(Ui::PAUSE_PROGRAM) + "\" mode to save the execution data.");
    }
    else
    {
      save_controller_data = true;
    }
}

void MainWindow::on_print_pos_btn_clicked()
{
    print_pos = true;
}

void MainWindow::PRINT_INFO_MSG(const std::string &msg)
{
  // setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236); color : blue; font: 75 12pt \"DejaVu Serif\"}");
  QPalette palette = ui->msg_label->palette();
  // palette.setColor(ui->msg_label->backgroundRole(), label_bg_clolor);
  palette.setColor(ui->msg_label->foregroundRole(), info_color);
  ui->msg_label->setPalette(palette);

  ui->msg_label->setText(msg.c_str());
}

void MainWindow::PRINT_WARN_MSG(const std::string &msg)
{
  // setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236); color : rgb(245, 121, 0); font: 75 12pt \"DejaVu Serif\"}");
  QPalette palette = ui->msg_label->palette();
  // palette.setColor(ui->msg_label->backgroundRole(), label_bg_clolor);
  palette.setColor(ui->msg_label->foregroundRole(), warn_color);
  ui->msg_label->setPalette(palette);

  ui->msg_label->setText(msg.c_str());
}

void MainWindow::PRINT_ERR_MSG(const std::string &msg)
{
  // setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236); color : red; font: 75 12pt \"DejaVu Serif\"}");

  QPalette palette = ui->msg_label->palette();
  // palette.setColor(ui->msg_label->backgroundRole(), label_bg_clolor);
  palette.setColor(ui->msg_label->foregroundRole(), err_color);
  ui->msg_label->setPalette(palette);

  ui->msg_label->setText(msg.c_str());
}

void MainWindow::PRINT_SUCCESS_MSG(const std::string &msg)
{
  // setStyleSheet(ui->msg_label, "QLabel { background-color : rgb(238, 238, 236); color : red; font: 75 12pt \"DejaVu Serif\"}");

  QPalette palette = ui->msg_label->palette();
  // palette.setColor(ui->msg_label->backgroundRole(), label_bg_clolor);
  palette.setColor(ui->msg_label->foregroundRole(), success_color);
  ui->msg_label->setPalette(palette);

  ui->msg_label->setText(msg.c_str());
}

void MainWindow::setMsg(const std::string &msg, Ui::MSG_TYPE msg_type)
{
  std::unique_lock<std::mutex> lck(msg_mtx);

  // setStyleSheet(ui->msg_label3, ui->msg_label2->styleSheet());
  ui->msg_label3->setPalette(ui->msg_label2->palette());
  ui->msg_label3->setText(ui->msg_label2->text());

  // setStyleSheet(ui->msg_label2, ui->msg_label->styleSheet());
  ui->msg_label2->setPalette(ui->msg_label->palette());
  ui->msg_label2->setText(ui->msg_label->text());

  switch (msg_type)
  {
    case Ui::MSG_TYPE::INFO:
      PRINT_INFO_MSG(msg.c_str());
      break;
    case Ui::MSG_TYPE::WARNING:
      PRINT_WARN_MSG(msg.c_str());
      break;
    case Ui::MSG_TYPE::ERROR:
      PRINT_ERR_MSG(msg.c_str());
      break;
    case Ui::MSG_TYPE::SUCCESS:
      PRINT_SUCCESS_MSG(msg.c_str());
      break;
  }
}

void MainWindow::setModeMsg(const std::string &msg)
{
  std::unique_lock<std::mutex> lck(mode_msg_mtx);
  // setStyleSheet(ui->mode_msg, "QLabel { background-color : rgb(238, 238, 236); color : blue; font: 75 13pt \"DejaVu Serif\"}");
  ui->mode_msg->setText(msg.c_str());
}

void MainWindow::displayWarningMsg(const std::string &msg)
{
    QMessageBox msg_box(QMessageBox::Warning, app_name.c_str(), tr(msg.c_str()), QMessageBox::Ok, this);
    msg_box.setDefaultButton(QMessageBox::Ok);
    msg_box.setStyleSheet("font: 75 14pt \"Ubuntu Mono\";");
    msg_box.setModal(true);
    msg_box.exec();
    // QMessageBox::StandardButton clicked_btn = (QMessageBox::StandardButton)(msg_box.exec());
    // if (clicked_btn != QMessageBox::Ok) ...
}

bool MainWindow::switchModeQuestionBox(const std::string &msg)
{
    QMessageBox msg_box(QMessageBox::Question, app_name.c_str(), tr(msg.c_str()), QMessageBox::Yes | QMessageBox::No, this);
    msg_box.setDefaultButton(QMessageBox::No);
    msg_box.setStyleSheet("font: 75 14pt \"Ubuntu Mono\";");
    msg_box.setModal(true);
    QMessageBox::StandardButton clicked_btn = (QMessageBox::StandardButton)(msg_box.exec());
    if (clicked_btn == QMessageBox::Yes) return true;
    else return false;
}

void MainWindow::closeEvent (QCloseEvent *event)
{
    QMessageBox msg_box(QMessageBox::Question, app_name.c_str(), tr("Are you sure you want to exit?\n"), QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel, this);
    msg_box.setDefaultButton(QMessageBox::Yes);
    msg_box.setStyleSheet("font: 75 14pt \"Ubuntu Mono\";");
    msg_box.setModal(true);
    QMessageBox::StandardButton clicked_btn = (QMessageBox::StandardButton)(msg_box.exec());

    if (clicked_btn != QMessageBox::Yes) event->ignore();
    else event->accept();
}
