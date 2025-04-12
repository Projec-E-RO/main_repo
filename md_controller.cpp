#include "md_controller/com.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

Communication Com;
MotorVar Motor1, Motor2;  // 두 개의 모터를 위한 구조체

geometry_msgs::msg::TransformStamped odom_tf;
sensor_msgs::msg::JointState joint_states;

BYTE SendCmdRpm = OFF;
int rpm_ = 0;

void CmdRpmCallBack(const std_msgs::msg::Int32::SharedPtr msg) {
    rpm_ = msg->data;
    SendCmdRpm = ON;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("md_controller_node");

    // TF 브로드캐스터 생성
    rclcpp::Time stamp_now;
    tf2_ros::TransformBroadcaster tf_broadcaster_(node);

    // RPM 콜백 함수 구독
    auto rpm_sub = node->create_subscription<std_msgs::msg::Int32>("/cmd_rpm", 1000, CmdRpmCallBack);

    // 모터 드라이버 설정 (두 개의 모터)
    node->declare_parameter("MDUI", 0);       
    node->declare_parameter("MDT", 183);        
    node->declare_parameter("Port", "/dev/ttyUSB0");
    node->declare_parameter("Baudrate", 57600);    
    node->declare_parameter("ID1", 1);             // 모터 1 ID
    node->declare_parameter("ID2", 2);             // 모터 2 ID
    node->declare_parameter("GearRatio", 30);     
    node->declare_parameter("poles", 30);         

    // 모터 파라미터 가져오기
    node->get_parameter("MDUI", Com.nIDMDUI);
    node->get_parameter("MDT", Com.nIDMDT);
    node->get_parameter("Port", Com.nPort);
    node->get_parameter("Baudrate", Com.nBaudrate);
    node->get_parameter("ID1", Motor1.ID);        // 모터 1 ID
    node->get_parameter("ID2", Motor2.ID);        // 모터 2 ID
    node->get_parameter("GearRatio", Motor1.GearRatio);
    node->get_parameter("poles", Motor1.poles);
    
    // Motor2의 poles와 기어비 추가 설정
    node->get_parameter("poles", Motor2.poles);
    node->get_parameter("GearRatio", Motor2.GearRatio);

    // 초기화 중에 ID가 제대로 설정되었는지 로그를 통해 확인
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 1 ID: %d", Motor1.ID);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 2 ID: %d", Motor2.ID);

    // PPR 값 계산
    Motor1.PPR = Motor1.poles * 3 * Motor1.GearRatio;           
    if (Motor1.PPR == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor 1 PPR is zero! Check poles and GearRatio.");
        Motor1.PPR = 1;  // 안전한 기본값 설정
    }
    Motor1.Tick2RAD = (360.0 / Motor1.PPR) * PI / 180;

    Motor2.PPR = Motor2.poles * 3 * Motor2.GearRatio;           
    if (Motor2.PPR == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Motor 2 PPR is zero! Check poles and GearRatio.");
        Motor2.PPR = 1;  // 안전한 기본값 설정
    }
    Motor2.Tick2RAD = (360.0 / Motor2.PPR) * PI / 180;

    IByte iData;
    int nArray[2];
    static BYTE fgInitsetting, byCntInitStep, byCntComStep, byCnt2500us, byCntStartDelay, byCntCase[5];
    
    byCntInitStep = 1;
    Motor1.InitMotor = ON;
    Motor2.InitMotor = ON;
    fgInitsetting = OFF;
    Motor1.InitError = 0;
    Motor2.InitError = 0;
    Motor1.last_rad = 0;
    Motor2.last_rad = 0;
    Motor1.last_tick = 0;
    Motor2.last_tick = 0;

    InitSerial();  // 통신 초기화

    while (rclcpp::ok()) {
        // 두 모터의 데이터를 모두 받음
        ReceiveDataFromController(Motor1.InitMotor);
        ReceiveDataFromController(Motor2.InitMotor);

        if (++byCnt2500us == 50) {
            byCnt2500us = 0;

            if (fgInitsetting == ON) {
                switch (++byCntComStep) {
                case 1: {
                    // 두 모터의 TF 생성 및 위치 업데이트
                    geometry_msgs::msg::TransformStamped transformStamped1;
                    transformStamped1.header.stamp = node->now();
                    transformStamped1.header.frame_id = "world";
                    transformStamped1.child_frame_id = "motor1_joint";

                    transformStamped1.transform.translation.x = 0.0;
                    transformStamped1.transform.translation.y = 0.0;
                    transformStamped1.transform.translation.z = 0.15;

                    Motor1.current_tick = Com.position;
                    Motor1.last_diff_tick = Motor1.current_tick - Motor1.last_tick;
                    Motor1.last_tick = Motor1.current_tick;
                    Motor1.last_rad += Motor1.Tick2RAD * (double)Motor1.last_diff_tick;
                    
                    // 쿼터니언 값이 NaN인지 확인
                    if (std::isnan(Motor1.last_rad)) {
                        Motor1.last_rad = 0.0;  // NaN일 경우 초기화
                    }
        
                    tf2::Quaternion q1;
                    q1.setRPY(0, 0, -Motor1.last_rad);
                    q1.normalize();  // 쿼터니언 정규화
                    transformStamped1.transform.rotation.x = q1.x();
                    transformStamped1.transform.rotation.y = q1.y();
                    transformStamped1.transform.rotation.z = q1.z();
                    transformStamped1.transform.rotation.w = q1.w();

                    // 모터 2의 TF
                    geometry_msgs::msg::TransformStamped transformStamped2;
                    transformStamped2.header.stamp = node->now();
                    transformStamped2.header.frame_id = "world";
                    transformStamped2.child_frame_id = "motor2_joint";

                    transformStamped2.transform.translation.x = 0.0;
                    transformStamped2.transform.translation.y = 0.0;
                    transformStamped2.transform.translation.z = 0.15;

                    Motor2.current_tick = Com.position;
                    Motor2.last_diff_tick = Motor2.current_tick - Motor2.last_tick;
                    Motor2.last_tick = Motor2.current_tick;
                    Motor2.last_rad += Motor2.Tick2RAD * (double)Motor2.last_diff_tick;
                    
                    // 쿼터니언 값이 NaN인지 확인
                    if (std::isnan(Motor2.last_rad)) {
                        Motor2.last_rad = 0.0;  // NaN일 경우 초기화
                    }

                    tf2::Quaternion q2;
                    q2.setRPY(0, 0, -Motor2.last_rad);
                    q2.normalize();  // 쿼터니언 정규화
                    transformStamped2.transform.rotation.x = q2.x();
                    transformStamped2.transform.rotation.y = q2.y();
                    transformStamped2.transform.rotation.z = q2.z();
                    transformStamped2.transform.rotation.w = q2.w();

                    tf_broadcaster_.sendTransform(transformStamped1);
                    tf_broadcaster_.sendTransform(transformStamped2);
                    break;
                }
                case 2: 
                    if (++byCntCase[byCntComStep] == TIME_100MS) {
                        byCntCase[byCntComStep] = 0;

                        if (SendCmdRpm) {
                            iData = Short2Byte(rpm_ * Motor1.GearRatio); // 모터 1의 RPM
                            nArray[0] = iData.byLow;
                            nArray[1] = iData.byHigh;
                            PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor1.ID, nArray);

                            iData = Short2Byte(rpm_ * Motor2.GearRatio); // 모터 2의 RPM
                            nArray[0] = iData.byLow;
                            nArray[1] = iData.byHigh;
                            PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor2.ID, nArray);

                            SendCmdRpm = OFF;
                        } else {
                            nArray[0] = PID_MAIN_DATA;
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor1.ID, nArray);  // 모터 1의 데이터 요청
                            PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor2.ID, nArray);  // 모터 2의 데이터 요청
                        }
                    }
                    byCntComStep = 0;
                    break;
                }
            } else {
                if (byCntStartDelay <= 200) byCntStartDelay++;
                else {
                    switch (byCntInitStep) {
                    case 1:
                        nArray[0] = PID_MAIN_DATA;
                        PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor1.ID, nArray);
                        PutMdData(PID_REQ_PID_DATA, Com.nIDMDT, Motor2.ID, nArray);

                        if (Motor1.InitMotor == ON && Motor2.InitMotor == ON)
                            Motor1.InitError++;
                        else
                            byCntInitStep++;

                        if (Motor1.InitError > 10 || Motor2.InitError > 10) {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "MOTOR INIT ERROR!!");
                            return 0;
                        }
                        break;
                    case 2:
                        byCntInitStep++;
                        break;

                    case 3:
                        nArray[0] = 0;
                        nArray[1] = 0;
                        PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor1.ID, nArray);
                        PutMdData(PID_VEL_CMD, Com.nIDMDT, Motor2.ID, nArray);
                        byCntInitStep++;
                        break;

                    case 4:
                        nArray[0] = 0;
                        PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor1.ID, nArray);
                        PutMdData(PID_POSI_RESET, Com.nIDMDT, Motor2.ID, nArray);
                        byCntInitStep++;
                        break;

                    case 5:
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MOTOR INIT END");
                        fgInitsetting = ON;
                        break;
                    }
                }
            }
        }

        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}

