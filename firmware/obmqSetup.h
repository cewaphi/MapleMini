// vim: expandtab tabstop=2 shiftwidth=2 softtabstop=2
// OBMQ setup

#ifndef _obmqSetup_h_
# define _obmqSetup_h_

#define MSG_INIT				0x00
#define MSG_INIT_OBMQ				0x01
#define MSG_INIT_CAN				0x02
#define MSG_INIT_SERIAL				0x03

#define MSG_CMD_GPIO				0xc0
#define MSG_CMD_UART				0xc1
#define MSG_CMD_VERSION				0xc2
#define MSG_CMD_PWMCAPTURE			0xc3
#define MSG_CMD_PWM				0xc4
#define MSG_CMD_SENSOR				0xc5
#define MSG_CMD_POLLSENSORS			0xc6
#define MSG_CMD_UNIQUEID			0xc7
#define MSG_CMD_ADC				0xc8
#define MSG_CMD_RESET				0xc9

void obmqSendMessage(char message);
void obmqSetup(void);

#endif // _obmqSetup_h_

