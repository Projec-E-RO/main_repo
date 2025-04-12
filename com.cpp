#include "md_controller/com.hpp"

serial::Serial ser;
//MotorVar Motor1, Motor2;  // Motor1과 Motor2를 구분하여 사용

// Get the low and high byte from short
IByte Short2Byte(short sIn)
{
    IByte Ret;
    Ret.byLow = sIn & 0xff;
    Ret.byHigh = sIn >> 8 & 0xff;
    return Ret;
}

// Make short data from two bytes
int Byte2Short(BYTE byLow, BYTE byHigh)
{
    return (byLow | (int)byHigh << 8);
}

// Make long data from four bytes
int Byte2LInt(BYTE byData1, BYTE byData2, BYTE byData3, BYTE byData4)
{
    return ((int)byData1 | (int)byData2 << 8 | (int)byData3 << 16 | (int)byData4 << 24);
}

// Initialize serial communication in ROS
int InitSerial(void)
{
    try
    {
        ser.setPort(Com.nPort);
        ser.setBaudrate(Com.nBaudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1667); // 1667 when baud is 57600, 0.6ms
        ser.setTimeout(to);                                        // 2857 when baud is 115200, 0.35ms
        ser.open();
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Unable to open port ");
        return -1;
    }
    if (ser.isOpen())
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Serial Port initialized");
    else
        return -1;
    return 0;
}

// For sending the data (One ID)
int PutMdData(BYTE byPID, BYTE byMID, int id_num, int nArray[])
{
    IByte iData;
    BYTE byPidDataSize, byDataSize, i, j;
    static BYTE byTempDataSum;

    for (j = 0; j < MAX_PACKET_SIZE; j++) Com.bySndBuf[j] = 0;

    Com.bySndBuf[0] = byMID;
    Com.bySndBuf[1] = 0;
    Com.bySndBuf[2] = id_num;
    Com.bySndBuf[3] = byPID;

    switch (byPID)
    {
    case PID_REQ_PID_DATA:
        byDataSize = 1;
        byPidDataSize = 7;
        byTempDataSum = 0;

        Com.bySndBuf[4] = byDataSize;
        Com.bySndBuf[5] = (BYTE)nArray[0];

        for (i = 0; i < (byPidDataSize - 1); i++) byTempDataSum += Com.bySndBuf[i];
        Com.bySndBuf[byPidDataSize - 1] = ~(byTempDataSum) + 1; // check sum

        ser.write(Com.bySndBuf, byPidDataSize);

        break;

    case PID_POSI_RESET:
        byDataSize = 1;
        byPidDataSize = 7;
        byTempDataSum = 0;

        Com.bySndBuf[4] = byDataSize;
        Com.bySndBuf[5] = nArray[0];

        for (i = 0; i < (byPidDataSize - 1); i++) byTempDataSum += Com.bySndBuf[i];
        Com.bySndBuf[byPidDataSize - 1] = ~(byTempDataSum) + 1; // check sum

        ser.write(Com.bySndBuf, byPidDataSize);

        break;

    case PID_COMMAND:
        byDataSize = 1;
        byPidDataSize = 7;
        byTempDataSum = 0;

        Com.bySndBuf[4] = byDataSize;
        Com.bySndBuf[5] = nArray[0];

        for (i = 0; i < (byPidDataSize - 1); i++) byTempDataSum += Com.bySndBuf[i];
        Com.bySndBuf[byPidDataSize - 1] = ~(byTempDataSum) + 1; // check sum

        ser.write(Com.bySndBuf, byPidDataSize);

        break;

    case PID_VEL_CMD:
        byDataSize = 2;  // RPM 값은 2바이트로 처리
        byPidDataSize = 8;  // 전체 패킷 크기
        byTempDataSum = 0;

        Com.bySndBuf[4] = byDataSize;
        Com.bySndBuf[5] = nArray[0];  // RPM (low byte)
        Com.bySndBuf[6] = nArray[1];  // RPM (high byte)

        // 디버깅: RPM 값 출력
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending RPM for Motor %d: RPM = %d", id_num, (nArray[0] << 8) + nArray[1]);

        for (i = 0; i < (byPidDataSize - 1); i++) byTempDataSum += Com.bySndBuf[i];
        Com.bySndBuf[byPidDataSize - 1] = ~(byTempDataSum) + 1; // 체크섬 계산

        // 디버깅: 패킷 전송 전 출력
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending data to motor %d: %x %x %x %x", id_num, Com.bySndBuf[0], Com.bySndBuf[1], Com.bySndBuf[2], Com.bySndBuf[3]);

        ser.write(Com.bySndBuf, byPidDataSize);
        break;
    }

    return SUCCESS;
}
int MdReceiveProc(void) // Save the identified serial data to defined variable according to PID NUMBER data
{
    BYTE byRcvRMID, byRcvTMID, byRcvID, byRcvPID, byRcvDataSize;

    byRcvRMID = Com.byRcvBuf[0];
    byRcvTMID = Com.byRcvBuf[1];
    byRcvID = Com.byRcvBuf[2];
    byRcvPID = Com.byRcvBuf[3];
    byRcvDataSize = Com.byRcvBuf[4];

    // 디버깅: 수신된 데이터 출력
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received data for Motor %d: PID = %d, RPM = %d, Position = %ld",
                byRcvID, byRcvPID, Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]), Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]));

    switch (byRcvPID)
    {
    case PID_MAIN_DATA:
        // Set RPM and position for Motor1 or Motor2 based on ID
        if (byRcvID == Motor1.ID)
        {
            Motor1.rpm = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Motor1.position = Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 1 RPM: %d, Position: %ld", Motor1.rpm, Motor1.position);
        }
        else if (byRcvID == Motor2.ID)
        {
            Motor2.rpm = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
            Motor2.position = Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 2 RPM: %d, Position: %ld", Motor2.rpm, Motor2.position);
        }
        break;
    }

    return SUCCESS;
}

int AnalyzeReceivedData(BYTE byArray[], BYTE byBufNum) // Analyze the communication data
{
    static BYTE byChkSec;
    BYTE i, j;
    int count = 0;

    for (j = 0; j < byBufNum; j++)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received byte: %d", byArray[j]);  // 수신된 데이터 로그 추가

        switch (Com.byStep)
        {
        case 0: // Put the transmitting machine ID after checking the data
            if ((byArray[j] == 0) || (byArray[j] == 183))
            {
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byChkComError = 0;
                count++;
                if (count == 2)
                    Com.byStep++;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR (1): Unexpected value received: %d", byArray[j]);  // 오류 로그 추가
                count = 0;
                Com.byStep = 0;
                Com.fgPacketOK = 0;
                Com.byPacketNum = 0;
                Com.byChkComError++;
                return FAIL;
            }
            break;

        case 1: // Check ID
            if (byArray[j] == Motor1.ID || byArray[j] == Motor2.ID)
            {
                Com.byChkSum += byArray[j];
                Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
                Com.byStep++;
                Com.byChkComError = 0;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR (2): Unexpected ID received: %d", byArray[j]);  // 오류 로그 추가
                Com.byStep = 0;
                Com.byPacketNum = 0;
                Com.byChkComError++;
                return FAIL;
            }
            break;

        case 2: // Put the PID number into the array
            Com.byChkSum += byArray[j];
            Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
            Com.byStep++;
            break;

        case 3: // Put the DATANUM into the array
            Com.byMaxDataNum = byArray[j];
            Com.byDataNum = 0;
            Com.byChkSum += byArray[j];
            Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
            Com.byStep++;
            break;

        case 4: // Put the DATA into the array
            Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
            Com.byChkSum += byArray[j];

            if (++Com.byDataNum >= MAX_DATA_SIZE)
            {
                Com.byStep = 0;
                Com.byTotalRcvDataNum = 0;
                break;
            }

            if (Com.byDataNum >= Com.byMaxDataNum)
                Com.byStep++;
            break;

        case 5: // Put the check sum after Checking checksum
            Com.byChkSum += byArray[j];
            Com.byRcvBuf[Com.byPacketNum++] = byArray[j];
            if (Com.byChkSum == 0)
            {
                Com.fgPacketOK = 1;
                Com.fgComDataChk = 1;
                Com.byDataNum = 0;
                Com.byMaxDataNum = 0;
            }

            Com.byStep = 0;
            Com.byTotalRcvDataNum = 0;
            break;

        default:
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR (3): Unknown state in packet processing");
            Com.byStep = 0;
            Com.fgComComple = ON;
            return FAIL;
        }

        if (Com.fgPacketOK)
        {
            Com.fgPacketOK = 0;
            Com.byPacketSize = 0;
            Com.byPacketNum = 0;

            // Process the data from the motor based on the ID
            if (byArray[2] == Motor1.ID)
            {
                Motor1.rpm = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
                Motor1.position = Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 1 RPM: %d, Position: %ld", Motor1.rpm, Motor1.position);
            }
            else if (byArray[2] == Motor2.ID)
            {
                Motor2.rpm = Byte2Short(Com.byRcvBuf[5], Com.byRcvBuf[6]);
                Motor2.position = Byte2LInt(Com.byRcvBuf[15], Com.byRcvBuf[16], Com.byRcvBuf[17], Com.byRcvBuf[18]);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Motor 2 RPM: %d, Position: %ld", Motor2.rpm, Motor2.position);
            }

            MdReceiveProc(); // save the identified serial data to defined variable
        }

        if (Com.byChkComError == 10) // while 50ms
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERROR (4): Too many communication errors");

            Com.byChkComError = 0;
            Com.byStep = 0;
            Com.byChkSum = 0;
            Com.byMaxDataNum = 0;
            Com.byDataNum = 0;
            for (i = 0; i < MAX_PACKET_SIZE; i++)
                Com.byRcvBuf[i] = 0;
            j = byBufNum;
            return FAIL;
        }
    }
    return SUCCESS;
}



int ReceiveDataFromController(BYTE init) // Analyze the communication data
{
    BYTE byRcvBuf[250];
    BYTE byBufNumber;

    byBufNumber = ser.available();

    if (byBufNumber != 0)
    {
        byBufNumber = MAX_DATA_SIZE;

        ser.read(byRcvBuf, byBufNumber);

        if (init == ON)
        {
            if (byRcvBuf[2] == Motor1.ID)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ID %d Motor Init success!", Motor1.ID);
                Motor1.InitMotor = OFF;
            }
            else if (byRcvBuf[2] == Motor2.ID)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ID %d Motor Init success!", Motor2.ID);
                Motor2.InitMotor = OFF;
            }
        }
        else
        {
            AnalyzeReceivedData(byRcvBuf, byBufNumber);
        }
    }
    return 1;
}
