#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QMessageBox>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    timerId = startTimer(50);

    ui->statusbar->addPermanentWidget(ui->label_author);
    ui->label_author->setText("Juan Jose Quiroz Omana");
    ui->statusbar->showMessage("Welcome to JuankaEmika", 5000);
    ui->lineEdit_robotIp->setText("172.16.0.2");

    reference_joint_x_
        = {ui->reference_joint_1, ui->reference_joint_2, ui->reference_joint_3,
           ui->reference_joint_4, ui->reference_joint_5, ui->reference_joint_6,
           ui->reference_joint_7};

    label_ref_joint_x_
        ={ui->label_ref_joint_1, ui->label_ref_joint_2, ui->label_ref_joint_3,
           ui->label_ref_joint_4, ui->label_ref_joint_5, ui->label_ref_joint_6,
           ui->label_ref_joint_7};

    spin_box_read_joint_x_
        = {ui->doubleSpinBox_q_1, ui->doubleSpinBox_q_2, ui->doubleSpinBox_q_3,
           ui->doubleSpinBox_q_4, ui->doubleSpinBox_q_5, ui->doubleSpinBox_q_6,
           ui->doubleSpinBox_q_7};

    spin_box_read_deg_joint_x_
        = {ui->doubleSpinBox_deg_q_1, ui->doubleSpinBox_deg_q_2, ui->doubleSpinBox_deg_q_3,
           ui->doubleSpinBox_deg_q_4, ui->doubleSpinBox_deg_q_5, ui->doubleSpinBox_deg_q_6,
           ui->doubleSpinBox_deg_q_7};

    spin_box_read_deg_joint_vel_x_
        ={ui->doubleSpinBox_q_dot_1, ui->doubleSpinBox_q_dot_2, ui->doubleSpinBox_q_dot_3,
          ui->doubleSpinBox_q_dot_4, ui->doubleSpinBox_q_dot_5, ui->doubleSpinBox_q_dot_6,
          ui->doubleSpinBox_q_dot_7,};

    enable_reference_sliders(false);
    ui->pushButton_robotHoming->setEnabled(false);
    ui->pushButton_handHoming->setEnabled(false);
    ui->pushButton_default_ref->setEnabled(false);
    ui->pushButton_joint_commands->setEnabled(false);
    ui->pushButton_move_hand->setEnabled(false);

    ui->horizontalSlider_gripper->setEnabled(false);
    ui->horizontalSlider_gripper->setRange(0, 800);
    ui->horizontalSlider_gripper->setSliderPosition(500);

    ui->checkBox_handGripper->setCheckState(Qt::Checked);
    ui->checkBox_handGripper->setEnabled(false);
    //ui->checkBox_robotHoming->setEnabled(false);
    //ui->checkBox_homingGripper->setEnabled(false);
    //ui->checkBox_joint_commands->setEnabled(false);
    //ui->checkBox_move_hand->setEnabled(false);
    enable_check_boxes(false);

    ui->pushButton_led->setEnabled(false);

    ui->pushButton_initialize->setEnabled(false);
    ui->pushButton_deinitialize->setEnabled(false);
    ui->pushButton_disconnect->setEnabled(false);

    set_push_button_color(ui->pushButton_connect, green_color_);
    set_push_button_color(ui->pushButton_initialize, gray_color_);
    set_push_button_color(ui->pushButton_deinitialize, gray_color_);
    set_push_button_color(ui->pushButton_disconnect, gray_color_);

    set_reference_sliders(q_rest_configuration_);
    set_push_button_color(ui->pushButton_joint_commands, gray_color_);
    set_push_button_color(ui->pushButton_default_ref, gray_color_);
    set_push_button_color(ui->pushButton_handHoming, gray_color_);
    set_push_button_color(ui->pushButton_move_hand, gray_color_);
    set_push_button_color(ui->pushButton_robotHoming, gray_color_);
    std::cout<<"---------------------------------------------------------------"<<std::endl;
    std::cout<<"---------------------------------------------------------------"<<std::endl;
    std::cout << R"(
               _                   _           ______           _ _
              | |                 | |         |  ____|         (_) |
              | |_   _  __ _ _ __ | | ____ _  | |__   _ __ ___  _| | ____ _
          _   | | | | |/ _` | '_ \| |/ / _` | |  __| | '_ ` _ \| | |/ / _` |
         | |__| | |_| | (_| | | | |   < (_| | | |____| | | | | | |   < (_| |
          \____/ \__,_|\__,_|_| |_|_|\_\__,_| |______|_| |_| |_|_|_|\_\__,_|

     )" << '\n';
    std::cout<<"-----------------------------------------------------------------"<<std::endl;
    std::cout<<"-----------------------------------------------------------------"<<std::endl;
}

//const VectorXd q_rest_configuration_ =(VectorXd(7)<<0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4).finished();

MainWindow::~MainWindow()
{
    delete ui;
    killTimer(timerId);
}

void MainWindow::enable_check_boxes(const bool& state)
{
    ui->checkBox_robotHoming->setEnabled(state);
    ui->checkBox_homingGripper->setEnabled(state);
    ui->checkBox_joint_commands->setEnabled(state);
    ui->checkBox_move_hand->setEnabled(state);
}

void MainWindow::update_status_bar(const std::string &message)
{
    ui->statusbar->showMessage(QString::fromStdString(message), 5000);
}

void MainWindow::set_reference_sliders(const VectorXd& q_initial)
{


    /*
    connect(ui->reference_joint_1, SIGNAL(valueChanged(int)),
            ui->label_ref_joint_1, SLOT(setValue(int)));

         */
    Eigen::VectorXd q_max_ =
        (Eigen::VectorXd(7)<< 130, 85, 140, -25, 140,  240, 150).finished();
    Eigen::VectorXd q_min_ =
        (Eigen::VectorXd(7)<<-130,-85, -140, -150, -140, 48, -150).finished();


    int i=0;
    for (auto& elem : reference_joint_x_)
    {
        elem->setRange(q_min_[i], q_max_[i]);
        elem->setSliderPosition((180/M_PI)*q_initial[i]);
        label_ref_joint_x_[i]->setText(QString::number((180/M_PI)*q_initial[i]));
        i++;
    }

}

void MainWindow::enable_reference_sliders(const bool& status)
{
    ui->pushButton_joint_commands->setDisabled(status);
    for (auto& elem : reference_joint_x_)
    {
        elem->setEnabled(status);
    }
}

void MainWindow::set_push_button_color(QPushButton *push_button, const QColor &color)
{
    push_button->setStyleSheet(QString("background-color: %1").arg(color.name()));
}





void MainWindow::on_pushButton_connect_clicked()
{
    QString robotIp = ui->lineEdit_robotIp->text();
    std::string robotIp_std = robotIp.toStdString();

    //QMessageBox::about(this,"title", "This will move the robot!");
    //QMessageBox::critical(this, "Title", "Hey hey");
    auto mode_txt = ui->comboBox->currentText().toStdString();
    RobotInterfaceFranka::MODE MODE;
    if (mode_txt== std::string("None"))
    {
        MODE = RobotInterfaceFranka::MODE::None;
    }else if (mode_txt== std::string("PositionControl"))
    {
        MODE = RobotInterfaceFranka::MODE::PositionControl;
    }else if (mode_txt== std::string("VelocityControl"))
    {
        MODE = RobotInterfaceFranka::MODE::VelocityControl;
    }
    else
    {
        std::runtime_error(std::string("Wrong mode. "));
    }


    //qDebug()<<"Robot pointer: "<< !!robot_driver_franka_sptr_;
    if (!robot_driver_franka_sptr_)
    {
        //qDebug()<<"Create Pointer ";
        robot_driver_franka_sptr_ = std::make_shared<RobotInterfaceFranka>(robotIp_std,
                                                                          MODE,
                                                                          RobotInterfaceFranka::HAND::ON);

    }

    robot_driver_franka_sptr_->connect();
    q_rest_configuration_ = robot_driver_franka_sptr_->get_home_robot_configuration();
    set_reference_sliders(robot_driver_franka_sptr_->get_joint_positions());
    connect_flag_      = true;
    initialize_flag_   = false;
    deinitialize_flag_ = false;
    disconnect_flag_   = false;

    enable_push_button(ui->pushButton_connect, false);
    enable_push_button(ui->pushButton_initialize, true);
    enable_push_button(ui->pushButton_disconnect, true);

    auto max_width = robot_driver_franka_sptr_->read_gripper(RobotInterfaceFranka::MAX_WIDTH);
    ui->horizontalSlider_gripper->setRange(100, int(max_width*10000)-100);
    auto width = robot_driver_franka_sptr_->read_gripper(RobotInterfaceFranka::WIDTH);
    ui->doubleSpinBox_hand->setValue(width);

    //qDebug()<<"Hola: "<< mode_txt ;
}

void MainWindow::on_pushButton_initialize_clicked()
{
    QMessageBox::warning(this, "Warning", "Initialize can move the robot. "
                                          "Please make sure to have the user stop button at hand! "
                                          "Use at your own risk");

    enable_push_button(ui->pushButton_initialize, false);
    enable_push_button(ui->pushButton_deinitialize, true);
    enable_push_button(ui->pushButton_disconnect, false);
    initialize_flag_ = true;
    robot_driver_franka_sptr_->initialize();
    enable_check_boxes(true);

}

void MainWindow::on_pushButton_deinitialize_clicked()
{
    enable_push_button(ui->pushButton_deinitialize, false);
    enable_push_button(ui->pushButton_disconnect, true);

    initialize_flag_ = false;
    deinitialize_flag_ = true;
    enable_check_boxes(false);
    robot_driver_franka_sptr_->deinitialize();

    ui->checkBox_homingGripper->setCheckState(Qt::Unchecked);
    ui->checkBox_move_hand->setCheckState(Qt::Unchecked);
    ui->checkBox_handGripper->setCheckState(Qt::Unchecked);
    ui->checkBox_joint_commands->setCheckState(Qt::Unchecked);
    ui->checkBox_robotHoming->setCheckState(Qt::Unchecked);
}


void MainWindow::on_pushButton_disconnect_clicked()
{
    enable_push_button(ui->pushButton_disconnect, false);
    enable_push_button(ui->pushButton_initialize, false);
    enable_push_button(ui->pushButton_deinitialize, false);
    enable_push_button(ui->pushButton_connect, true);
    update_status_bar(robot_driver_franka_sptr_->get_status_message());
    ui->comboBox->setEnabled(false);
    connect_flag_      = false;
    initialize_flag_   = false;
    deinitialize_flag_ = false;
    disconnect_flag_   = false;
    robot_driver_franka_sptr_->disconnect();

    ui->checkBox_homingGripper->setCheckState(Qt::Unchecked);
    ui->checkBox_move_hand->setCheckState(Qt::Unchecked);
    ui->checkBox_handGripper->setCheckState(Qt::Checked);
    ui->checkBox_joint_commands->setCheckState(Qt::Unchecked);
    ui->checkBox_robotHoming->setCheckState(Qt::Unchecked);


}



void MainWindow::timerEvent(QTimerEvent *event)
{
    //qDebug() << "Update...  "<<connect_flag_;
    if (connect_flag_)
    {
        auto q = robot_driver_franka_sptr_->get_joint_positions();
        auto q_dot = robot_driver_franka_sptr_->get_joint_velocities();
        time_from_robot_ = robot_driver_franka_sptr_->get_time();
        //std::cout<<"q: "<<q.transpose()<<" time: "<< time_from_robot_<<std::endl;

        update_led_timer_status();
        update_read_joints(q, q_dot);
        ui->doubleSpinBox_time->setValue(time_from_robot_);
        robot_driver_franka_sptr_->set_target_joint_positions(q_target_);
        ui->label_robot_mode->setText(QString::fromStdString(robot_driver_franka_sptr_->get_robot_mode()));

    }else
    {
        enable_push_button(ui->pushButton_led, false);
    }
    if (robot_driver_franka_sptr_)
    {
        update_status_bar(robot_driver_franka_sptr_->get_status_message());
    }

}


void MainWindow::update_led_timer_status()
{
    int number = int(time_from_robot_)%2;
    blink_button(ui->pushButton_led, number, green_led_color_, gray_color_);
}

void MainWindow::update_read_joints(const VectorXd& q, const VectorXd& q_dot)
{
    int i=0;
    for (auto& elem : spin_box_read_joint_x_)
    {
        elem->setValue(q[i]);
        spin_box_read_deg_joint_x_[i]->setValue(q[i]*180/M_PI);
        spin_box_read_deg_joint_vel_x_[i]->setValue(q_dot[i]);
        i++;
    }

}




void MainWindow::enable_push_button(QPushButton* push_button, const bool& state)
{
    push_button->setEnabled(state);
    QColor color;
    if (state == true)
    {
        color = green_color_;
    }else
    {
        color = gray_color_;
    }
    set_push_button_color(push_button, color);
}


void MainWindow::blink_button(QPushButton* push_button,
                              const bool& state,
                              const QColor& color_on,
                              const QColor& color_off)
{
    //push_button->setEnabled(false);
    QColor color;
    if (state == true)
    {
        color = color_on;
    }else
    {
        color = color_off;
    }
    set_push_button_color(push_button, color);
}


void MainWindow::on_reference_joint_1_valueChanged(int value)
{
    ui->label_ref_joint_1->setText(QString::number(value));
    q_target_[0] = value*(M_PI/180.0);
}

void MainWindow::on_reference_joint_2_valueChanged(int value)
{
    ui->label_ref_joint_2->setText(QString::number(value));
    q_target_[1] = value*(M_PI/180.0);
}
void MainWindow::on_reference_joint_3_valueChanged(int value)
{
    ui->label_ref_joint_3->setText(QString::number(value));
    q_target_[2] = value*(M_PI/180.0);
}
void MainWindow::on_reference_joint_4_valueChanged(int value)
{
    ui->label_ref_joint_4->setText(QString::number(value));
    q_target_[3] = value*(M_PI/180.0);
}
void MainWindow::on_reference_joint_5_valueChanged(int value)
{
    ui->label_ref_joint_5->setText(QString::number(value));
    q_target_[4] = value*(M_PI/180.0);
}
void MainWindow::on_reference_joint_6_valueChanged(int value)
{
    ui->label_ref_joint_6->setText(QString::number(value));
    q_target_[5] = value*(M_PI/180.0);
}
void MainWindow::on_reference_joint_7_valueChanged(int value)
{
    ui->label_ref_joint_7->setText(QString::number(value));
    q_target_[6] = value*(M_PI/180.0);
}

void MainWindow::on_pushButton_default_ref_clicked()
{
    set_reference_sliders(q_rest_configuration_);
    ui->horizontalSlider_gripper->setSliderPosition(7);
}




void MainWindow::on_horizontalSlider_gripper_valueChanged(int value)
{
    ui->label_ref_joint_hand->setText(QString::number(value));
    command_gripper_ = double(value*0.0001);
    //qDebug()<<"command: "<<command_gripper_;
}

void MainWindow::on_checkBox_homingGripper_clicked(bool checked)
{
    if (checked){
        // Uncheck other checkboxes
        //ui->checkBox_homingGripper->setCheckState(Qt::Unchecked);
        ui->checkBox_move_hand->setCheckState(Qt::Unchecked);
        ui->checkBox_handGripper->setCheckState(Qt::Unchecked);
        ui->checkBox_joint_commands->setCheckState(Qt::Unchecked);
        ui->checkBox_robotHoming->setCheckState(Qt::Unchecked);
    }
}


void MainWindow::on_checkBox_move_hand_clicked(bool checked)
{
    if (checked){
        // Uncheck other checkboxeshandHoming
        ui->checkBox_homingGripper->setCheckState(Qt::Unchecked);
        //ui->checkBox_move_hand->setCheckState(Qt::Unchecked);
        ui->checkBox_handGripper->setCheckState(Qt::Unchecked);
        ui->checkBox_joint_commands->setCheckState(Qt::Unchecked);
        ui->checkBox_robotHoming->setCheckState(Qt::Unchecked);

    }
}


void MainWindow::on_checkBox_robotHoming_clicked(bool checked)
{
    if (checked){
        // Uncheck other checkboxes
        ui->checkBox_homingGripper->setCheckState(Qt::Unchecked);
        ui->checkBox_move_hand->setCheckState(Qt::Unchecked);
        ui->checkBox_handGripper->setCheckState(Qt::Unchecked);
        ui->checkBox_joint_commands->setCheckState(Qt::Unchecked);
        //ui->checkBox_robotHoming->setCheckState(Qt::Unchecked);
    }
}


void MainWindow::on_checkBox_joint_commands_clicked(bool checked)
{
    if (checked){
        // Uncheck other checkboxes
        ui->checkBox_homingGripper->setCheckState(Qt::Unchecked);
        ui->checkBox_move_hand->setCheckState(Qt::Unchecked);
        ui->checkBox_handGripper->setCheckState(Qt::Unchecked);
        //ui->checkBox_joint_commands->setCheckState(Qt::Unchecked);
        ui->checkBox_robotHoming->setCheckState(Qt::Unchecked);

    }
}


void MainWindow::on_checkBox_joint_commands_stateChanged()
{
    Qt::CheckState current_state = ui->checkBox_joint_commands->checkState();
    if (current_state == Qt::Checked)
    {
        enable_reference_sliders(true);
        ui->pushButton_joint_commands->setEnabled(true);
        ui->pushButton_default_ref->setEnabled(true);
        set_push_button_color(ui->pushButton_joint_commands, green_color_);
        set_push_button_color(ui->pushButton_default_ref, green_color_);
        //set_push_button_color(ui->pushButton_handHoming, gray_color_);
        //set_push_button_color(ui->pushButton_move_hand, gray_color_);
        //set_push_button_color(ui->pushButton_robotHoming, gray_color_);

    }
    else{
        enable_reference_sliders(false);
        ui->pushButton_joint_commands->setEnabled(false);
        ui->pushButton_default_ref->setEnabled(false);
        set_push_button_color(ui->pushButton_joint_commands, gray_color_);
        set_push_button_color(ui->pushButton_default_ref, gray_color_);
    }
}

void MainWindow::on_checkBox_robotHoming_stateChanged()
{
    Qt::CheckState current_state = ui->checkBox_robotHoming->checkState();
    if (current_state == Qt::Checked)
    {

        ui->pushButton_robotHoming->setEnabled(true);
        set_push_button_color(ui->pushButton_robotHoming, green_color_);

    }
    else{
        ui->pushButton_robotHoming->setEnabled(false);
        set_push_button_color(ui->pushButton_robotHoming, gray_color_);
    }
}


void MainWindow::on_checkBox_move_hand_stateChanged()
{
    Qt::CheckState current_state = ui->checkBox_move_hand->checkState();
    if (current_state == Qt::Checked)
    {

        ui->pushButton_move_hand->setEnabled(true);
        set_push_button_color(ui->pushButton_move_hand, green_color_);
        ui->horizontalSlider_gripper->setEnabled(true);

    }
    else{
        ui->pushButton_move_hand->setEnabled(false);
        set_push_button_color(ui->pushButton_move_hand, gray_color_);
        ui->horizontalSlider_gripper->setEnabled(false);
    }
}



void MainWindow::on_checkBox_homingGripper_stateChanged()
{
    Qt::CheckState current_state = ui->checkBox_homingGripper->checkState();
    if (current_state == Qt::Checked)
    {

        ui->pushButton_handHoming->setEnabled(true);
        set_push_button_color(ui->pushButton_handHoming, green_color_);

    }
    else{
        ui->pushButton_handHoming->setEnabled(false);
        set_push_button_color(ui->pushButton_handHoming, gray_color_);
    }
}




void MainWindow::on_pushButton_robotHoming_clicked()
{
    QMessageBox::warning(this, "Warning", "This will move the robot. "
                                          "Please make sure to have the user stop button at hand! "
                                          "Use at your own risk");
    robot_driver_franka_sptr_->move_robot_to_target_joint_positions(
        robot_driver_franka_sptr_->get_home_robot_configuration());
}




void MainWindow::on_pushButton_joint_commands_clicked()
{
    std::cout<<"q_target_: "<<q_target_.transpose()<<std::endl;
    std::cout<<"q_target_: "<<(180/M_PI)*q_target_.transpose()<<std::endl;
    QMessageBox::warning(this, "Warning", "This will move the robot. "
                                          "Please make sure to have the user stop button at hand! "
                                          "Use at your own risk");
    //robot_driver_franka_sptr_->move_robot_to_target_joint_positions(q_target_);
    robot_driver_franka_sptr_->move_robot_to_target_joint_positions(q_target_);
}




void MainWindow::on_pushButton_handHoming_clicked()
{
    robot_driver_franka_sptr_->gripper_homing();
    auto width     = robot_driver_franka_sptr_->read_gripper(RobotInterfaceFranka::WIDTH);
    ui->doubleSpinBox_hand->setValue(width);
}


void MainWindow::on_pushButton_move_hand_clicked()
{
    auto max_width = robot_driver_franka_sptr_->read_gripper(RobotInterfaceFranka::MAX_WIDTH);
    auto width     = robot_driver_franka_sptr_->read_gripper(RobotInterfaceFranka::WIDTH);
    ui->doubleSpinBox_hand->setValue(width);

    if (command_gripper_ < 0.01 or command_gripper_ > max_width)
    {
        qDebug()<<"The desired value is outside the range. ";
    }else{

        robot_driver_franka_sptr_->set_gripper(command_gripper_);
    }
}

