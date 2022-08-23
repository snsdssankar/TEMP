#ifndef DP_CRF_6392_UART_COMMUNICATION_H
#define DP_CRF_6392_UART_COMMUNICATION_H

#include <QWidget>
#include "dp_crf_6392_datastructure.h"
#include <QTimer>

extern unsigned char DP_SDR_V_UART_FLAG;
extern SDP_CRF_6392_GLOBAL_STRUCTURE objGlobalStructure;
extern unsigned char DP_SDR_UART_V_L_Port;
extern unsigned int gUART_RespFlag;
namespace Ui {
class dp_crf_6392_Uart_communication;
}

class dp_crf_6392_Uart_communication : public QWidget
{
    Q_OBJECT

public:
    explicit dp_crf_6392_Uart_communication(QWidget *parent = 0);
    ~dp_crf_6392_Uart_communication();

    QString deviceDescription;
    QString serialBuffer;
    qint32 baudrate;
    QString portName;

    bool serialDeviceIsConnectedV = false;
    bool serialDeviceIsConnectedL = false;

    S_RX_RS232DATA_PACKET L_Band_RxData;
    S_RX_RS232DATA_PACKET V_UHFRxData;
    SDP_SDR_COMM_Data_Packet sV_UHF_Data;
    SDP_SDR_COMM_Data_Packet sL_Band_Data;

    QString ActionLog;
    char cBuff[50] = {0};
    unsigned char ucTxRxBuffer[120] = {0};



    short DP_CRF_V_UART_COMM_Wrapper(unsigned char in_ucCmdID, unsigned char *in_ucData, unsigned char in_ucDataSize, unsigned char *out_ucData, unsigned char *outDataSize);
    short DP_CRF_L_UART_COMM_Wrapper(unsigned char in_ucCmdID, unsigned char *in_ucData, unsigned char in_ucDataSize, unsigned char *out_ucData, unsigned char *outDataSize);

    void on_DP_BTN_L_BAND_ReadData();
    void on_DP_BTN_V_UHF_ReadData();

signals:
    void DP_CRF_V_UHF_GPIOData_Send(unsigned char in_ucHighLow,unsigned char in_ucLoopBacksel,unsigned char in_ucGPIOData);

    void DP_CRF_L_Band_GPIOData_Send(unsigned char in_ucHighLow,unsigned char in_ucLoopBacksel,unsigned char in_ucGPIOData);

    void DP_CRF_V_UHF_IO_Data_Send(unsigned char in_ucHighLow);
    void DP_CRF_L_BAND_IO_Data_Send(unsigned char, unsigned char);

    void DP_CRF_Action_Log_Data_Send(QString Out_strData,QString color);
    void DP_CRF_SPI_READ_DATA_SEND(unsigned char);
    void DP_CRF_CHK_SUM_DATA_SEND(unsigned int);
    void DP_CRF_L_CHK_SUM_DATA_SEND(unsigned int);
    void DP_CRF_O_TAB_DATA_SEND(unsigned char,unsigned char, unsigned char);
    void DP_CRF_C_TAB_DATA_SEND(unsigned char,unsigned char, unsigned char);
    void DP_CRF_GPIO_TAB_DATA_SEND(unsigned char,unsigned char, unsigned char);
private slots:


    void on_DP_BTN_V_UHF_COMM_OPEN_clicked();

    void on_DP_BTN_V_UHF_COMM_CLOSE_clicked();

    void on_DP_BTN_L_BAND_COMM_OPEN_clicked();

    void on_DP_BTN_L_BAND_COMM_CLOSE_clicked();



private:
    Ui::dp_crf_6392_Uart_communication *ui;
};

#endif // DP_CRF_6392_UART_COMMUNICATION_H
