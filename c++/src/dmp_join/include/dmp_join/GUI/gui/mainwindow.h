#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAbstractButton>
#include <QLabel>
#include <QString>
#include <QColor>
#include <QPalette>
#include <QFont>
#include <QString>
#include <QSlider>
#include <QCloseEvent>

#include <iostream>
#include <cstdlib>
#include <vector>
#include <thread>
#include <mutex>
#include <cstring>
#include <memory>
#include <sstream>
#include <condition_variable>
#include <iomanip>

#include <QLineEdit>
#include <QSlider>

namespace Ui {

enum ProgramState
{
  VOID = -1,
  RUN_CONTROLLER,
  FREEDRIVE_MODE,
  PAUSE_PROGRAM,

};

enum MSG_TYPE
{
  INFO,
  WARNING,
  ERROR,
  SUCCESS
};

class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void init();

    Ui::ProgramState state;

    bool print_pos;

    bool save_trained_model;
    bool load_trained_model;
    bool load_training_data;

    bool train_model;
    bool model_trained;

    bool log_controller;
    bool save_controller_data;

    bool goto_start_pose;
    bool current_pose_as_start;

    std::vector<std::string> stateName;

    void setMsg(const std::string &msg, Ui::MSG_TYPE msg_type);
    void setModeMsg(const std::string &msg);

    Ui::ProgramState getState();
    void setState(const Ui::ProgramState &new_state);
    void enforceState(const Ui::ProgramState &new_state);

    std::string getModeName(Ui::ProgramState mode = Ui::VOID);

private slots:
    void on_freedrive_mode_button_clicked();
    void on_pause_program_button_clicked();

    void on_run_controller_button_clicked();

    void on_move_to_start_btn_clicked();
    void on_register_startPose_btn_clicked();

    void on_train_model_btn_clicked();

    void on_save_trained_model_btn_clicked();

    void on_load_trained_model_btn_clicked();

    void on_load_training_data_btn_clicked();

    void on_controller_log_checkbox_toggled(bool checked);

    void on_save_controller_data_btn_clicked();

    void on_print_pos_btn_clicked();

    void closeEvent (QCloseEvent *event);

private:
    std::string app_name;

    Ui::MainWindow *ui;

    QWidget *parent;

    std::mutex msg_mtx;
    std::mutex mode_msg_mtx;
    std::mutex btn_click_mtx;
    std::mutex state_mtx;

    void setStyleSheet(QAbstractButton *btn, const std::string &style_sheet);
    void setStyleSheet(QLabel *label, const QString &style_sheet);

    QColor info_color, warn_color, err_color, success_color;
    QColor label_bg_clolor;
    QFont msg_label_font;
    QFont mode_label_font;

    void PRINT_INFO_MSG(const std::string &msg);
    void PRINT_WARN_MSG(const std::string &msg);
    void PRINT_ERR_MSG(const std::string &msg);
    void PRINT_SUCCESS_MSG(const std::string &msg);

    void displayWarningMsg(const std::string &msg);
    bool switchModeQuestionBox(const std::string &msg);

};

#endif // MAINWINDOW_H
