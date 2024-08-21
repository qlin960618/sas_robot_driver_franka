#pragma once
#include "QtWidgets/qlabel.h"
#include "QtWidgets/qslider.h"
#include "qpushbutton.h"
#include <memory>
#include <eigen3/Eigen/Dense>

#include <QMainWindow>
#include "qspinbox.h"
#include <sas_robot_driver_franka/robot_interface_franka.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void set_reference_sliders(const VectorXd& q_initial);
    void enable_reference_sliders(const bool& status);
    void set_push_button_color(QPushButton* push_button, const QColor& color);
    void enable_push_button(QPushButton* push_button, const bool& state);
    void blink_button(QPushButton* push_button, const bool& state, const QColor& color_on, const QColor& color_off);
    void update_read_joints(const VectorXd& q, const VectorXd& q_dot);
    void update_led_timer_status();
    void enable_check_boxes(const bool& state);
    void update_status_bar(const std::string& message);


private slots:



    void on_pushButton_connect_clicked();

    void on_reference_joint_1_valueChanged(int value);
    void on_reference_joint_2_valueChanged(int value);
    void on_reference_joint_3_valueChanged(int value);
    void on_reference_joint_4_valueChanged(int value);
    void on_reference_joint_5_valueChanged(int value);
    void on_reference_joint_6_valueChanged(int value);
    void on_reference_joint_7_valueChanged(int value);

    void on_pushButton_default_ref_clicked();

    //void on_checkBox_joint_commands_clicked(bool checked);
    //void on_checkBox_homingGripper_clicked(bool checked);
    //void on_checkBox_robotHoming_clicked(bool checked);
    //void on_checkBox_move_hand_clicked(bool checked);

    void on_horizontalSlider_gripper_valueChanged(int value);


    void on_checkBox_homingGripper_clicked(bool checked);

    void on_checkBox_move_hand_clicked(bool checked);

    void on_checkBox_robotHoming_clicked(bool checked);

    void on_checkBox_joint_commands_clicked(bool checked);

    void on_checkBox_joint_commands_stateChanged();

    void on_checkBox_move_hand_stateChanged();

    void on_checkBox_robotHoming_stateChanged();

    void on_checkBox_homingGripper_stateChanged();

    void on_pushButton_disconnect_clicked();

    void on_pushButton_initialize_clicked();

    void on_pushButton_deinitialize_clicked();

    void on_pushButton_robotHoming_clicked();



    void on_pushButton_joint_commands_clicked();



    void on_pushButton_handHoming_clicked();

    void on_pushButton_move_hand_clicked();

private:
    Ui::MainWindow *ui;
    //std::shared_ptr<Ui::MainWindow> ui;
    std::vector<QSlider*> reference_joint_x_;
    std::vector<QLabel*> label_ref_joint_x_;
    std::vector<QDoubleSpinBox*> spin_box_read_joint_x_;
    std::vector<QDoubleSpinBox*> spin_box_read_deg_joint_x_;
    std::vector<QDoubleSpinBox*> spin_box_read_deg_joint_vel_x_;

    QColor green_color_ = QColor::fromRgb(144,238,144);

    QColor green_led_color_ = QColor::fromRgb(0,255,0);
    QColor gray_color_ = QColor::fromRgb(180,180,180);

    std::shared_ptr<RobotInterfaceFranka> robot_driver_franka_sptr_ = nullptr;
    int timerId;

    VectorXd q_target_ =(VectorXd(7)<<0, -0.3, 0, -1.92, 0, M_PI_2, M_PI_4).finished();
    VectorXd q_rest_configuration_= (VectorXd(7)<<0, -0.3, 0, -1.92, 0, M_PI_2, M_PI_4).finished();

    double command_gripper_ = 0.05;

    double time_from_robot_;

    bool connect_flag_      = false;
    bool initialize_flag_   = false;
    bool deinitialize_flag_ = false;
    bool disconnect_flag_   = false;

protected:
    void timerEvent(QTimerEvent *event);
};
