#include "dp_crf_6392_uart_communication.h"
#include "ui_dp_crf_6392_uart_communication.h"
#include "dp_crf_6392_config.h"
#include <QDesktopWidget>
#include <QtSerialPort/QSerialPort>
#include <QDebug>
#include "QMessageBox"
#include "dp_crf_6392_macros.h"
#include "dp_crf_6392_gpio_loop_back_test.h"
#include "ui_dp_crf_6392_gpio_loop_back_test.h"
#include "dp_crf_6392_form_flash.h"
#include "dp_crf_6392_v_uhf_transceiver.h"
#include "about.h"
unsigned char DP_SDR_V_CHKSUM_FG = DP_CRF_6392_INIT_VAL;
unsigned char DP_SDR_L_CHKSUM_FG = DP_CRF_6392_INIT_VAL;
unsigned char DP_SDR_TAB_FLAG = DP_CRF_6392_INIT_VAL; // Flag for Tab widget of GPIO Enable
dp_crf_6392_Uart_communication::dp_crf_6392_Uart_communication(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::dp_crf_6392_Uart_communication)
{
    ui->setupUi(this);
    //setGeometry(0,30,width(),height());
    setGeometry(0,30,1430,560);
    objGlobalStructure.m_V_Obj_Timer = new QTimer(this);
    objGlobalStructure.m_L_Obj_Timer = new QTimer(this);
    objGlobalStructure.objSerialUARTComV = new QSerialPort(this);
    objGlobalStructure.objSerialUARTComL = new QSerialPort(this);
    ui->DP_BTN_L_BAND_COMM_CLOSE->setHidden(true);
    ui->DP_BTN_V_UHF_COMM_CLOSE->setHidden(true);
    ui->label->setGeometry(0,0,1430,560);
    setWindowTitle(tr("UART Configuration"));
#ifndef __Debug__
    qDebug() << "Number of available ports: " << QSerialPortInfo::availablePorts().length();
#endif //__Debug__
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        ui->DP_CMB_V_UHF_Port_Number->addItem(info.portName());
        ui->DP_CMB_L_BAND_Port_Number->addItem(info.portName());
    }
    connect(objGlobalStructure.m_V_Obj_Timer,SIGNAL(timeout()),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_V_UHF,SLOT(DP_V_UART_ERR_Response()));
    connect(objGlobalStructure.m_L_Obj_Timer,SIGNAL(timeout()),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_L_BAND,SLOT(DP_L_UART_ERR_Response()));
    connect(objGlobalStructure.objSerialUARTComV,objGlobalStructure.objSerialUARTComV->readyRead,this,dp_crf_6392_Uart_communication::on_DP_BTN_V_UHF_ReadData);
    connect(objGlobalStructure.objSerialUARTComL,objGlobalStructure.objSerialUARTComL->readyRead,this,dp_crf_6392_Uart_communication::on_DP_BTN_L_BAND_ReadData);
    connect(this,SIGNAL(DP_CRF_V_UHF_GPIOData_Send(unsigned char,unsigned char,unsigned char)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_GPIO,SLOT(DP_CRF_GPIO_V_UHF_LED_Update(unsigned char,unsigned char,unsigned char)));
    connect(this,SIGNAL(DP_CRF_L_Band_GPIOData_Send(unsigned char,unsigned char,unsigned char)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_GPIO,SLOT(DP_CRF_GPIO_L_Band_LED_Update(unsigned char,unsigned char,unsigned char)));
    connect(this,SIGNAL(DP_CRF_V_UHF_IO_Data_Send(unsigned char)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_V_UHF,SLOT(DP_BTN_V_UHF_IO_LED_Update(unsigned char)));
    connect(this,SIGNAL(DP_CRF_L_BAND_IO_Data_Send(unsigned char,unsigned char)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_L_BAND,SLOT(DP_BTN_L_BAND_IO_LED_Update(unsigned char,unsigned char)));
    connect(this,SIGNAL(DP_CRF_Action_Log_Data_Send(QString,QString)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent()),SLOT(DP_CRF_6392_Update_Action_Log(QString,QString)));
    connect(this,SIGNAL(DP_CRF_SPI_READ_DATA_SEND(unsigned char)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_V_UHF,SLOT(DP_SPI_Data_Read(unsigned char)));
    connect(this,SIGNAL(DP_CRF_CHK_SUM_DATA_SEND(unsigned int)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_About,SLOT(DP_CRF_V_CHK_SUM_RCV_DATA(unsigned int)));
    connect(this,SIGNAL(DP_CRF_L_CHK_SUM_DATA_SEND(unsigned int)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent())->m_Obj_About,SLOT(DP_CRF_L_CHK_SUM_RCV_DATA(unsigned int)));
    connect(this,SIGNAL(DP_CRF_O_TAB_DATA_SEND(unsigned char,unsigned char,unsigned char)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent()),SLOT(DP_CRF_6392_O_Tab_Data_Rcve(unsigned char,unsigned char,unsigned char)));
    connect(this,SIGNAL(DP_CRF_C_TAB_DATA_SEND(unsigned char,unsigned char,unsigned char)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent()),SLOT(DP_CRF_6392_C_Tab_Data_Rcve(unsigned char,unsigned char,unsigned char)));
    connect(this,SIGNAL(DP_CRF_GPIO_TAB_DATA_SEND(unsigned char,unsigned char,unsigned char)),reinterpret_cast<dp_crf_6392_form_flash *>(this->parent()),SLOT(DP_CRF_6392_GPIO_Tab_Data_Rcve(unsigned char,unsigned char,unsigned char)));
}
dp_crf_6392_Uart_communication::~dp_crf_6392_Uart_communication()
{
    delete ui;
}
void dp_crf_6392_Uart_communication::on_DP_BTN_V_UHF_COMM_OPEN_clicked()
{
    objGlobalStructure.m_usCOMStatusV = DP_CRF_6392_COM_OPEN;
    DP_SDR_V_UART_FLAG = DP_SDR_SET;
    ++DP_SDR_TAB_FLAG;
    //Tab Flag will enable the GPIO tab widget
    if(DP_SDR_TAB_FLAG < DP_CRF_6392_TWO)
        emit DP_CRF_O_TAB_DATA_SEND(DP_SDR_TAB_INDEX_ONE,DP_SDR_TAB_INDEX_TWO,DP_SDR_TAB_INDEX_THREE);
    if(DP_SDR_TAB_FLAG == DP_CRF_6392_TWO)
        emit DP_CRF_GPIO_TAB_DATA_SEND(DP_SDR_TAB_INDEX_ONE,DP_SDR_TAB_INDEX_TWO,DP_SDR_TAB_INDEX_THREE);
    ui->DP_BTN_V_UHF_COMM_CLOSE->setHidden(false);
    ui->DP_BTN_V_UHF_COMM_OPEN->setHidden(true);
    if(serialDeviceIsConnectedV == false)
    {
        sV_UHF_Data.ucData = 0x01;
        objGlobalStructure.objSerialUARTComV->setPortName(ui->DP_CMB_V_UHF_Port_Number->currentText());
        serialDeviceIsConnectedV = true;
#ifndef __Debug__
        qDebug() << "connecting to: "<< ui->DP_CMB_V_UHF_Port_Number->currentText();
#endif //__Debug__
        if(objGlobalStructure.objSerialUARTComV->open(QIODevice::ReadWrite))
        {
            //Now the serial port is open try to set configuration
            if(!objGlobalStructure.objSerialUARTComV->setBaudRate(QSerialPort::Baud230400))        //Depends on your boud-rate on the Device
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComV->errorString();
#endif //__Debug__
            if(!objGlobalStructure.objSerialUARTComV->setDataBits(QSerialPort::Data8))
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComV->errorString();
#endif //__Debug__
            if(!objGlobalStructure.objSerialUARTComV->setParity(QSerialPort::NoParity))
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComV->errorString();
#endif //__Debug__
            if(!objGlobalStructure.objSerialUARTComV->setStopBits(QSerialPort::OneStop))
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComV->errorString();
#endif //__Debug__
            if(!objGlobalStructure.objSerialUARTComV->setFlowControl(QSerialPort::NoFlowControl))
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComV->errorString();
#endif //__Debug__
            //If any error was returned the serial il corrctly configured

            qDebug() << "V/UHF : Connection to: "<< ui->DP_CMB_V_UHF_Port_Number->currentText() << " " << " connected";

            ActionLog.clear();
            ActionLog.append(" V/UHF : Connection to port: ");
            ActionLog.append(ui->DP_CMB_V_UHF_Port_Number->currentText());
            ActionLog.append(" Connected");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            ActionLog.clear();
            sV_UHF_Data.sRetval = DP_CRF_V_UART_COMM_Wrapper(DP_SDR_CMD_V_UHF_CHK_SUM, &sV_UHF_Data.ucData,sizeof(sV_UHF_Data.ucData), &sV_UHF_Data.ucReadData, &sV_UHF_Data.ucReadDataSize);
            if(sV_UHF_Data.sRetval == DP_CRF_6392_FAILURE)
            {
                ActionLog.append(" V/UHF Check Sum Send Failure");
                emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
            }
            else
            {
                DP_SDR_V_CHKSUM_FG++;
                ActionLog.append(" V/UHF Check Sum Send Success");
                emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            }
#if 0
            sRetval = DP_CRF_UART_COMM_Wrapper(ucCmdID, &sV_UHF_Data.ucData,sizeof(sV_UHF_Data.ucData), &sV_UHF_Data.ucReadData, &sV_UHF_Data.ucReadDataSize);
            if(sRetval == DP_CRF_6392_SUCCESS)
            {
                if(ucReadData == DP_CRF_6392_SUCCESS)
                {

                }
            }
#endif

        }
        else
        {
#ifndef __Debug__
            qDebug() << "V/UHF : Connection to: "<< ui->DP_CMB_V_UHF_Port_Number->currentText() << " " << " not connected";
            qDebug() <<"Error: "<<objGlobalStructure.objSerialUARTComV->errorString();
#endif //__Debug__
            serialDeviceIsConnectedV = false;
            ActionLog.clear();
            ActionLog.append(" V/UHF : Connection to port Fail : ");
            ActionLog.append(ui->DP_CMB_V_UHF_Port_Number->currentText());
            ActionLog.append(objGlobalStructure.objSerialUARTComV->errorString());
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
            //ui->DP_BTN_V_UHF_COMM_OPEN->setDisabled(true);
        }
        // to send the Command to enable the communication
    }
    else
    {
#ifndef __Debug__
        qDebug() << "V/UHF : Can't connect, another device is connected";
#endif //__Debug__
        ActionLog.clear();
        ActionLog.append(" V/UHF : Can't connect, another device is connected");
        emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
    }
}
void dp_crf_6392_Uart_communication::on_DP_BTN_V_UHF_COMM_CLOSE_clicked()
{
    ui->DP_BTN_V_UHF_COMM_CLOSE->setHidden(true);
    ui->DP_BTN_V_UHF_COMM_OPEN->setHidden(false);
    //ui->DP_BTN_L_BAND_COMM_CLOSE->setEnabled(true);
    //ui->DP_BTN_L_BAND_COMM_OPEN->setEnabled(true);
    DP_SDR_V_UART_FLAG = DP_SDR_RESET;
    --DP_SDR_TAB_FLAG;
    emit DP_CRF_C_TAB_DATA_SEND(DP_SDR_TAB_INDEX_ONE,DP_SDR_TAB_INDEX_TWO,DP_SDR_TAB_INDEX_THREE);
    objGlobalStructure.objSerialUARTComV->close();
#ifndef __Debug__
    qDebug() << "V/UHF : Connection to: "<< ui->DP_CMB_V_UHF_Port_Number->currentText() << " " << " Disconnected";
#endif //__Debug__
    serialDeviceIsConnectedV = false;
    ActionLog.clear();
    ActionLog.append(" V/UHF : Connection to port: ");
    ActionLog.append(ui->DP_CMB_V_UHF_Port_Number->currentText());
    ActionLog.append(" Disconnected ");
    emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
    objGlobalStructure.m_usCOMStatusV = DP_CRF_6392_COM_CLOSE;
}
void dp_crf_6392_Uart_communication::on_DP_BTN_L_BAND_COMM_OPEN_clicked()
{
    ui->DP_BTN_L_BAND_COMM_CLOSE->setHidden(false);
    ui->DP_BTN_L_BAND_COMM_OPEN->setHidden(true);
    objGlobalStructure.m_usCOMStatusL = DP_CRF_6392_COM_OPEN;
    //  ui->DP_BTN_V_UHF_COMM_CLOSE->setEnabled(false);
    //ui->DP_BTN_V_UHF_COMM_OPEN->setEnabled(false);
    ++DP_SDR_TAB_FLAG;
    if(DP_SDR_TAB_FLAG < DP_CRF_6392_TWO)
        emit DP_CRF_O_TAB_DATA_SEND(DP_SDR_TAB_INDEX_TWO,DP_SDR_TAB_INDEX_ONE,DP_SDR_TAB_INDEX_THREE);
    if(DP_SDR_TAB_FLAG == DP_CRF_6392_TWO)
        emit DP_CRF_GPIO_TAB_DATA_SEND(DP_SDR_TAB_INDEX_ONE,DP_SDR_TAB_INDEX_TWO,DP_SDR_TAB_INDEX_THREE);
    if(serialDeviceIsConnectedL == false)
    {
        objGlobalStructure.objSerialUARTComL->setPortName(ui->DP_CMB_L_BAND_Port_Number->currentText());
#ifndef __Debug__
        qDebug() << "connecting to: "<< ui->DP_CMB_L_BAND_Port_Number->currentText();
#endif //__Debug__
        if(objGlobalStructure.objSerialUARTComL->open(QIODevice::ReadWrite))
        {
           serialDeviceIsConnectedL = true;
            //Now the serial port is open try to set configuration
            if(!objGlobalStructure.objSerialUARTComL->setBaudRate(QSerialPort::Baud230400))        //Depends on your boud-rate on the Device
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComL->errorString();
#endif //__Debug__
            if(!objGlobalStructure.objSerialUARTComL->setDataBits(QSerialPort::Data8))
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComL->errorString();
#endif //__Debug__
            if(!objGlobalStructure.objSerialUARTComL->setParity(QSerialPort::NoParity))
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComL->errorString();
#endif //__Debug__
            if(!objGlobalStructure.objSerialUARTComL->setStopBits(QSerialPort::OneStop))
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComL->errorString();
#endif //__Debug__
            if(!objGlobalStructure.objSerialUARTComL->setFlowControl(QSerialPort::NoFlowControl))
#ifndef __Debug__
                qDebug()<<objGlobalStructure.objSerialUARTComL->errorString();
#endif //__Debug__
            //If any error was returned the serial il corrctly configured

            qDebug() << "L-BAND : Connection to: "<< ui->DP_CMB_L_BAND_Port_Number->currentText() << " " << " connected";
            ActionLog.clear();
            ActionLog.append("L-BAND : Connection to port: ");
            ActionLog.append(ui->DP_CMB_L_BAND_Port_Number->currentText());
            ActionLog.append(" Connected");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            ActionLog.clear();
            sL_Band_Data.sRetval = DP_CRF_L_UART_COMM_Wrapper(DP_SDR_CMD_L_BAND_CHK_SUM, &sL_Band_Data.ucData,sizeof(sL_Band_Data.ucData), &sL_Band_Data.ucReadData, &sL_Band_Data.ucReadDataSize);
            if(sL_Band_Data.sRetval == DP_CRF_6392_FAILURE)
            {
                ActionLog.append(" L-Band Check Sum Send Failure");
                emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
            }
            else
            {
                DP_SDR_L_CHKSUM_FG++;
                ActionLog.append(" L-Band Check Sum send Success");
                emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            }
        }
        else
        {
#ifndef __Debug__
            qDebug() << "L-BAND : Connection to: "<< ui->DP_CMB_L_BAND_Port_Number->currentText() << " " << " not connected";
            qDebug() <<"Error: "<<objGlobalStructure.objSerialUARTComL->errorString();
#endif //__Debug__
            serialDeviceIsConnectedL = false;
            ActionLog.clear();
            ActionLog.append("L-BAND: Connection to port Fail : ");
            ActionLog.append(ui->DP_CMB_L_BAND_Port_Number->currentText());
            ActionLog.append(objGlobalStructure.objSerialUARTComL->errorString());
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
        }
        // to send the Command to enable the communication
    }
    else
    {
#ifndef __Debug__
        qDebug() << "L-BAND : Can't connect, another device is connected";
#endif //__Debug__
        ActionLog.clear();
        ActionLog.append("L-BAND : Can't connect, another device is connected");
        emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
    }
}
void dp_crf_6392_Uart_communication::on_DP_BTN_L_BAND_COMM_CLOSE_clicked()
{
    ui->DP_BTN_L_BAND_COMM_CLOSE->setHidden(true);
    ui->DP_BTN_L_BAND_COMM_OPEN->setHidden(false);
    // ui->DP_BTN_V_UHF_COMM_CLOSE->setEnabled(true);
    //ui->DP_BTN_V_UHF_COMM_OPEN->setEnabled(true);
    objGlobalStructure.objSerialUARTComL->close();
    --DP_SDR_TAB_FLAG;
    emit DP_CRF_C_TAB_DATA_SEND(DP_SDR_TAB_INDEX_ONE,DP_SDR_TAB_INDEX_TWO,DP_SDR_TAB_INDEX_THREE);
#ifndef __Debug__
    qDebug() << "L-BAND : Connection to: "<< ui->DP_CMB_L_BAND_Port_Number->currentText() << " " << " Disconnected";
#endif //__Debug__
    serialDeviceIsConnectedL = false;
    ActionLog.clear();
    ActionLog.append("L-BAND : Connection to port: ");
    ActionLog.append(ui->DP_CMB_L_BAND_Port_Number->currentText());
    ActionLog.append(" Disconnected ");
    emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
    objGlobalStructure.m_usCOMStatusL = DP_CRF_6392_COM_CLOSE;
}
short dp_crf_6392_Uart_communication::DP_CRF_V_UART_COMM_Wrapper(unsigned char in_ucCmdID, unsigned char *in_ucData, unsigned char in_ucDataSize, unsigned char *out_ucData, unsigned char *outDataSize)
{
    unsigned char ucLoop = DP_CRF_6392_INIT_VAL;
    short sRetVal = DP_CRF_6392_INIT_VAL;
    /* UART COMMUNICATION TX Packet */
    S_TX_RS232_PACKET sTxPacket;
    memset(&sTxPacket,0,sizeof(sTxPacket));
    if(serialDeviceIsConnectedV != true)
    {
        QMessageBox::warning(this,"UART COMM", "Open the V/UHF COMM Port");
    }
    /*Copy the received data to Send Packet*/
    sTxPacket.ucHeader[0] = DP_SDR_COM_CMD_HEADER_LSB; // 0X41
    sTxPacket.ucHeader[1] = DP_SDR_COM_CMD_HEADER_MSB; // 0X54
    sTxPacket.ucCmd = in_ucCmdID;
    sTxPacket.ucFooter = DP_SDR_COM_CMD_FOOTER; //0X23
    memcpy(&sTxPacket.arrucData,in_ucData, in_ucDataSize);
#ifndef __Debug__
    qDebug("V_CmdID : %d",sTxPacket.ucCmd);
    qDebug("V_Datasize : %d",in_ucDataSize);
    for(ucLoop = DP_CRF_6392_INIT_VAL; ucLoop < in_ucDataSize; ucLoop++)
    {
        qDebug("V_Data[%d]:%d",ucLoop,sTxPacket.arrucData[ucLoop]);
    }
    qDebug("V_Footer:%X",sTxPacket.ucFooter);

#endif
    sRetVal = objGlobalStructure.objSerialUARTComV->write((const char*)&sTxPacket.ucHeader,sizeof(sTxPacket.ucHeader));
    for(int i=0;i<300;i++);
    sRetVal = objGlobalStructure.objSerialUARTComV->write((const char*)&sTxPacket.ucCmd,sizeof(sTxPacket.ucCmd));
    for(int i=0;i<300;i++);
    sRetVal = objGlobalStructure.objSerialUARTComV->write((const char*)&sTxPacket.arrucData,sizeof(sTxPacket.arrucData));
    for(int i=0;i<300;i++);
    sRetVal = objGlobalStructure.objSerialUARTComV->write((const char*)&sTxPacket.ucFooter,sizeof(sTxPacket.ucFooter));
    objGlobalStructure.m_V_Obj_Timer->start(3000);
#ifndef __Debug__
    qDebug("Send Success");
#endif //__Debug__
    for(int i=0;i<2000;i++);
    // on_DP_BTN_V_UHF_ReadData();
    // ActionLog.clear();
    // ActionLog = "V-UHF DATA SEND SUCCESS";
    //emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
    return sRetVal;
}
short dp_crf_6392_Uart_communication::DP_CRF_L_UART_COMM_Wrapper(unsigned char in_ucCmdID, unsigned char *in_ucData, unsigned char in_ucDataSize, unsigned char *out_ucData, unsigned char *outDataSize)
{
    //  emit DP_CRF_L_Band_GPIOData_Send(1,1,0X03);
    unsigned char ucLoop = DP_CRF_6392_INIT_VAL;
    short sRetVal = DP_CRF_6392_INIT_VAL;
    /* UART COMMUNICATION TX Packet */
    S_TX_RS232_PACKET sTxPacket;
    memset(&sTxPacket,0,sizeof(sTxPacket));
    if(serialDeviceIsConnectedL != true)
    {
        QMessageBox::warning(this,"UART COMM", "Open the L-BAND COMM Port");
    }
    /*Copy the received data to Send Packet*/
    sTxPacket.ucHeader[0] = DP_SDR_COM_CMD_HEADER_LSB; // 0X41
    sTxPacket.ucHeader[1] = DP_SDR_COM_CMD_HEADER_MSB; // 0X54
    sTxPacket.ucCmd = in_ucCmdID;
    sTxPacket.ucFooter = DP_SDR_COM_CMD_FOOTER; //0X23
    memcpy(&sTxPacket.arrucData,in_ucData, in_ucDataSize);
#ifndef __Debug__
    // qDebug("L_Header: %x,:%x",sTxPacket.ucHeader[0], sTxPacket.ucHeader[1]);
    qDebug("L_CmdID : %d",sTxPacket.ucCmd);
    qDebug("L_Datasize : %d",in_ucDataSize);

    for(ucLoop = DP_CRF_6392_INIT_VAL; ucLoop < in_ucDataSize; ucLoop++)
    {
        qDebug("L_Data[%d]:%d",ucLoop,sTxPacket.arrucData[ucLoop]);
    }

    qDebug("L_Footer:%X",sTxPacket.ucFooter);
#endif //__Debug__


    sRetVal = objGlobalStructure.objSerialUARTComL->write((const char*)&sTxPacket.ucHeader,sizeof(sTxPacket.ucHeader));
    for(int i=0;i<300;i++);
    sRetVal = objGlobalStructure.objSerialUARTComL->write((const char*)&sTxPacket.ucCmd,sizeof(sTxPacket.ucCmd));
    for(int i=0;i<300;i++);
    sRetVal = objGlobalStructure.objSerialUARTComL->write((const char*)&sTxPacket.arrucData,sizeof(sTxPacket.arrucData));
    for(int i=0;i<300;i++);
    sRetVal = objGlobalStructure.objSerialUARTComL->write((const char*)&sTxPacket.ucFooter,sizeof(sTxPacket.ucFooter));
    objGlobalStructure.m_L_Obj_Timer->start(3000);
#ifndef __Debug__
    qDebug("Send Success");
#endif //__Debug__
    for(int i=0;i<300;i++);
    // on_DP_BTN_L_BAND_ReadData();
    //ActionLog.clear();
    //ActionLog = "L-Band DATA SEND SUCCESS";
    // emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
    return sRetVal;
}
void dp_crf_6392_Uart_communication::on_DP_BTN_V_UHF_ReadData()
{
    objGlobalStructure.m_V_Obj_Timer->stop();
   /* ActionLog.clear();
    ActionLog.append("V/UHF - Band Ack Success");
    emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);*/
    QByteArray V_UHF_Response;
    unsigned char ucarrData[8];
    unsigned char ucLoop;
    unsigned char ucLoopBacksel;
    unsigned char ucHighLow;
    unsigned char ucGPIOData;
    memset(&V_UHFRxData,0,sizeof(V_UHFRxData));
#define _DEBUG_PRINT_
    V_UHF_Response = objGlobalStructure.objSerialUARTComV->readAll();
    for(ucLoop = 0;ucLoop < V_UHF_Response.length(); ucLoop++)
    {
        ucarrData[ucLoop] = V_UHF_Response.at(ucLoop);
#ifndef __Debug__
        qDebug("Data[%d]:%d",ucLoop,ucarrData[ucLoop]);
#endif //__Debug__
    }
    /**copy the received data to UART RX PACKET **/
    memcpy(&V_UHFRxData,ucarrData,sizeof(V_UHFRxData));
    /* check for Header-LSB ,MSB  and Footer  */
    if ((V_UHFRxData.ucHeader[DP_CRF_6392_ZERO] == DP_SDR_COM_CMD_HEADER_LSB) && (V_UHFRxData.ucHeader[DP_CRF_6392_ONE] == DP_SDR_COM_CMD_HEADER_MSB) && (V_UHFRxData.ucFooter == DP_SDR_COM_CMD_FOOTER))
    {
        switch (V_UHFRxData.ucCmd)
        {
        case DP_SDR_CMD_V_UHF_UART_COMM:
        {
#ifndef __Debug__
            qDebug("UART Communication CMD\n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" V/UHF  UART Communication Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_TRANSCEIVER_AND_BITE_MODE_SEL:
        {
#ifndef __Debug__
            qDebug("TRNS-BITE CMD\n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" V/UHF  TRNS-BITE  Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_TX_RX_MODE_SEL:
        {
#ifndef _Debug_
            qDebug("TX RX Mode CMD\n\r");
#endif //__Debug__
            ActionLog.clear();
            ActionLog.append(" V/UHF  TX RX Mode Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_COSITE_NONCOSITE_MODE_SEL:
        {
#ifndef __Debug__
            qDebug(" Cosite NonCosite CMD\n");
#endif //__Debuf__
            ActionLog.clear();
            ActionLog.append(" V/UHF Cosite NonCosite Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_WIDE_AND_NARROW_BAND:
        {
#ifndef __Debug__
            qDebug("Wide Narrow CMD\n");
#endif //__debug__
            ActionLog.clear();
            ActionLog.append(" V/UHF Wide Narrow Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_FREQUENC_SEL:
        {
#ifndef __Debug__
            qDebug("Frequency CMD\n");
#endif //__debug__
            ActionLog.clear();
            ActionLog.append(" V/UHF Frequency sel Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            /*SYSTME TESTING */
        }
            break;
        case DP_SDR_CMD_V_UHF_GUARD_RX_GAIN_CTRL:
        {
#ifndef __Debug__
            qDebug("Guard Receiver Gain CMD\n");
#endif
            ActionLog.clear();
            ActionLog.append(" V/UHF Guard Receiver Gain Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_GUARD_RX_BAND_SEL:
        {
#ifndef __Debug__
            qDebug("Guard RX Band CMD\n");
#endif
            ActionLog.clear();
            ActionLog.append(" V/UHF Guard RX Band Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_MAIN_RX_AMP_CTRL:
        {
#ifndef __Debug__
            qDebug("Main Rx Gain CMD\n");
#endif
            ActionLog.clear();
            ActionLog.append(" V/UHF Main Rx Gain Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_MAIN_RX_ATT_CTRL:
        {
#ifndef __Debug__
            qDebug("Attenuation CMD\n");
#endif
            ActionLog.clear();
            ActionLog.append(" V/UHF Attenuation Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_GPIO_LOOPBACK:
        {
#ifndef __Debug__
            qDebug(" V-UHF GPIO CMD \n");
#endif
            ucHighLow = V_UHFRxData.arrucData[0];
            ucLoopBacksel =  V_UHFRxData.arrucData[1];
            ucGPIOData = V_UHFRxData.arrucData[2];
            emit DP_CRF_V_UHF_GPIOData_Send(ucHighLow,ucLoopBacksel,ucGPIOData);
            ActionLog.clear();
            ActionLog.append(" V/UHF GPIO Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);

        }
            break;
        case DP_SDR_CMD_V_UHF_IO_VALIDATION_TEST:
        {
#ifndef __Debug__
            qDebug("IO CMD\n");
#endif
            ucHighLow = V_UHFRxData.arrucData[0];
            emit DP_CRF_V_UHF_IO_Data_Send(ucHighLow);
            ActionLog.clear();
            ActionLog.append(" V/UHF IO Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_V_UHF_SPI_VALIDATION_TEST:
        {
#ifndef __Debug__
            qDebug("SPI CMD\n");
#endif                  
            ActionLog.clear();
            ActionLog.append(" V/UHF SPI Write Success ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            /*if(V_UHFRxData.arrucData[DP_CRF_6392_ZERO] == 2)
            {
                ActionLog.clear();
                memcpy(&objGlobalStructure.m_uc_V_UartData,V_UHFRxData.arrucData,sizeof(&objGlobalStructure.m_uc_V_UartData));
                emit DP_CRF_SPI_READ_DATA_SEND(objGlobalStructure.m_uc_V_UartData[DP_CRF_6392_ONE]);
                sprintf(cBuff,"SPI Read Data:%d",objGlobalStructure.m_uc_V_UartData[DP_CRF_6392_ONE]);
                ActionLog.append(cBuff);
                emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            }
            else
            {
                ActionLog.clear();
                ActionLog.append(" V/UHF SPI Not Read ");
                emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
            }*/
        }
            break;
        case DP_SDR_CMD_V_UHF_CHK_SUM:
        {
#ifndef __Debug__
            qDebug("V/UHF Check Sum CMD\n");
#endif
            memcpy(&objGlobalStructure.m_i_V_ChkSum,V_UHFRxData.arrucData,sizeof(&objGlobalStructure.m_i_V_ChkSum));
            emit DP_CRF_CHK_SUM_DATA_SEND(objGlobalStructure.m_i_V_ChkSum);
            ActionLog.clear();
            ActionLog.append("V/UHF Check Sum Receive Success");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        default:
        {
            /*INVALID COMMAND */
#ifndef __Debug__
            qDebug("Invalid CMD \n");
#endif
            ActionLog.clear();
            ActionLog.append(" V/UHF Invalid CMD Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
        }
            break;
        }
    }
    else
    {
#ifndef __Debug__
        qDebug(" H-F Error \n");
#endif //__Debug__
        ActionLog.clear();
        ActionLog.append(" V/UHF H-F Error Response");
        emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
    }
#ifndef __Debug__
    qDebug() << "V/UHF DATA RECEIVED ";
#endif //__Debug__
}
void dp_crf_6392_Uart_communication::on_DP_BTN_L_BAND_ReadData()
{
    objGlobalStructure.m_L_Obj_Timer->stop();
  /*  ActionLog.clear();
    ActionLog.append("L - Band Ack Success");
    emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);*/
    QByteArray L_Band_Response;
    unsigned char ucarrData[8];
    unsigned char ucLoop;
    unsigned char ucLoopBacksel;
    unsigned char ucHighLow, ucHighLowAck;
    unsigned char ucGPIOData;
    memset(&L_Band_RxData,0,sizeof(L_Band_RxData));
    L_Band_Response = objGlobalStructure.objSerialUARTComL->readAll();
    for(ucLoop = 0;ucLoop < L_Band_Response.length(); ucLoop++)
    {
        ucarrData[ucLoop] = L_Band_Response.at(ucLoop);
#ifndef __Debug__
        qDebug("Data[%d]:%d",ucLoop,ucarrData[ucLoop]);
#endif //__Debug__
    }
    /**copy the received data to UART RX PACKET **/
    memcpy(&L_Band_RxData,ucarrData,sizeof(L_Band_RxData));
    /* check for Header-LSB ,MSB  and Footer  */
    if ((L_Band_RxData.ucHeader[DP_CRF_6392_ZERO] == DP_SDR_COM_CMD_HEADER_LSB) && (L_Band_RxData.ucHeader[DP_CRF_6392_ONE] == DP_SDR_COM_CMD_HEADER_MSB) && (L_Band_RxData.ucFooter == DP_SDR_COM_CMD_FOOTER))
    {
        switch(L_Band_RxData.ucCmd)
        {
        case DP_SDR_CMD_L_BAND_UART_COMMUNICATION:
        {
#ifndef __Debug__
            printf("UART Communication CMD\n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" L-band  UART Communication Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_L_BAND_TRANSCEIVER_AND_BITE_MODE:
        {
#ifndef __Debug__
            printf("TRNS-BITE CMD\n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" L-band  TRNS-BITE Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_L_BAND_TX_RX_MODE:
        {
#ifndef __Debug__
            printf("TX RX Mode CMD\n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" L-band  TX RX Mode Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_L_BAND_TRANSCEIVER_ATTENUATION:
        {
#ifndef __Debug__
            printf("Main and AUX RX Attenuation CMD\n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" L-band  Main and AUX RX Attenuation Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_L_BAND_MAIN_AND_AUX_AMP_CTRL:
        {
#ifndef __Debug__
            printf("Main and AUX AMP ON/OFF CMD\n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" L-band  Main and AUX AMP ON/OFF Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_L_BAND_GPIO_LOOPBACK_TEST:
        {

#ifndef __Debug__
            printf(" L-Band GPIO Loop Back CMD \n\r");
#endif
            ucHighLow = L_Band_RxData.arrucData[0];
            ucLoopBacksel =  L_Band_RxData.arrucData[1];
            ucGPIOData = L_Band_RxData.arrucData[2];
            emit DP_CRF_L_Band_GPIOData_Send(ucHighLow,ucLoopBacksel,ucGPIOData);
            ActionLog.clear();
            ActionLog.append(" L-band  GPIO Loop Back Response ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_L_BAND_SPI_VALIDATION_TEST:
        {
#ifndef __Debug__
            printf("SPI CMD\n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" L-band  SPI Write Success ");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            /*if(L_Band_RxData.arrucData[DP_CRF_6392_ZERO] == 2)
            {
                memcpy(&objGlobalStructure.m_uc_L_UartData,L_Band_RxData.arrucData,sizeof(&objGlobalStructure.m_uc_L_UartData));
                emit DP_CRF_SPI_READ_DATA_SEND(objGlobalStructure.m_uc_L_UartData[DP_CRF_6392_ONE]);
                sprintf(cBuff,"SPI Read Data:%d",objGlobalStructure.m_uc_L_UartData[DP_CRF_6392_ONE]);
                ActionLog.append(cBuff);
                emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
            }*/
            //memcpy(&objGlobalStructure.m_uc_L_UartData,V_UHFRxData.arrucData,sizeof(&objGlobalStructure.m_uc_L_UartData));
        }
            break;
        case DP_SDR_CMD_L_BAND_IO_VALIDATION_TEST:
        {
#ifndef __Debug__
            printf("IO CMD\n\r");
#endif
            ucHighLow = L_Band_RxData.arrucData[0];
            ucHighLowAck = L_Band_RxData.arrucData[1];

            emit DP_CRF_L_BAND_IO_Data_Send(ucHighLow,ucHighLowAck);

            ActionLog.clear();
            ActionLog.append(" L-band  IO CMD Response ");

            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;
        case DP_SDR_CMD_L_BAND_CHK_SUM:
        {
#ifndef __Debug__
            qDebug("L - Band Check Sum CMD\n");
#endif
            memcpy(&objGlobalStructure.m_i_L_ChkSum,L_Band_RxData.arrucData,sizeof(&objGlobalStructure.m_i_L_ChkSum));
            emit DP_CRF_L_CHK_SUM_DATA_SEND(objGlobalStructure.m_i_L_ChkSum);
            ActionLog.clear();
            ActionLog.append("L - Band Check Sum Receive Success");
            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_SUCCESS);
        }
            break;

        default:
        {
            /*INVALID COMMAND */
#ifndef __Debug__
            printf("V/UHF Invalid CMD \n\r");
#endif
            ActionLog.clear();
            ActionLog.append(" L-band  Invalid Response ");

            emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
        }
            break;
        }

    }
    else
    {
#ifndef __Debug__
        qDebug(" H-F Error \n");
#endif //__Debug__
        ActionLog.clear();
        ActionLog.append(" L-band H-F Error Response");
        emit DP_CRF_Action_Log_Data_Send(ActionLog,DP_CRF_6392_PRINT_FAILURE);
    }

#ifndef __Debug__
    qDebug() << "L-band DATA RECEIVED ";
#endif //__Debug__
    /*******To set uart response flag ***/
    gUART_RespFlag = DP_SDR_SET;
}
