/*
* RFID.c
*
* Created: 7/28/2017 11:09:16 AM
* Author : ToanNM <toannm.hust@gmail.com>
*/ 
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
/** DEFINE PIN/PORT **/
#define PIN_LED_STT                     PIND4
#define PORT_LED_STT                    PORTD
#define DDR_LED_STT                     DDRD
#define PIN_LED_STT1                    PINC3
#define PORT_LED_STT1                   PORTC
#define DDR_LED_STT1                    DDRC
#define PIN_SPK                         PINA1
#define PORT_SPK                        PORTA
#define DDR_SPK                         DDRA
#define PIN_CR95_SS                     PINB3
#define PIN_CR95_MOSI                   PINB5
#define PIN_CR95_MISO                   PINB6
#define PIN_CR95_SCK                    PINB7
#define PORT_CR95_SPI                   PORTB
#define DDR_CR95_SPI                    DDRB
#define PIN_IRQ                         PINB2
#define PORT_IRQ                        PORTB
#define DDR_IRQ                         DDRB
/** DEFINE MACRO **/
#define GPIO_WRITE_HIGHT(PORT,PIN)      PORT |= (1 << PIN);
#define GPIO_WRITE_LOW(PORT,PIN)        PORT &= ~(1 << PIN);
#define MAX(x,y)                        ((x > y)? x : y)
#define MIN(x,y)                        ((x < y)? x : y)
#define ABS(x)                          ((x)>0 ? (x) : -(x))
#define CHECKVAL(val, min,max)          ((val < min || val > max) ? FALSE : TRUE)
#ifndef errchk
#define errchk(fCall) if (status = (fCall), status != CR95HF_SUCCESS_CODE) \
{goto Error;} else
#endif
void write_low_cr95();
void write_high_cr95();
int check_bit_zero(int reg, int bit);
void init_alert();
void beepbeep(int n);
void CR95HF_Wakeup();
/** Define SPI drive **/
void SPI_init();
uint8_t SPI_tranrecv(uint8_t data);
/** Define UART drive **/
void UART_init();
void UART_send(uint8_t data);
int UART_send_multibyte(uint8_t *data,int len);
/** Define CH95HF FLAGs **/
#define ERR_SOF_HIGH                    0x63   
#define ERR_SOF_LOW                     0x65   
#define ERR_EGT                         0x66   
#define ERR_2B2L                        0x67
#define ERR_2M2M                        0x68
#define ERR_FRAME                       0x71
#define FRAME_OK                        0x80
#define USER_STOP                       0x85
#define ERR_COM                         0x86
#define FRAME_WAIT_TOUT                 0x87
#define SOF_INVL                        0x88
#define OVF_BUFF                        0x89
#define ERR_FRAMING                     0x8A
#define ERR_EGT_1                       0x8B
#define LEN_VALID                       0x8C
#define ERR_CRC                         0x8D
#define RECV_LOST                       0x8E
#define NO_FIELD                        0x8F
#define UNINT_BYTE                      0x90
/** DEFINE CR95HF FUNCTION FLAGs **/
#define CR95HF_ERRORCODE_DEFAULT        0xFE
#define CMD_RESET                       0x01
#define IDN                             0x01
#define PROTOCOLSELECT                  0x02
#define SENDRECEIVE                     0x04
#define IDLE                            0x07
#define RDREG                           0x08
#define WRREG                           0x09
#define BAUDRATE                        0x0A
#define ECHO                            0x55
#define CMD_POLL                        0x03
#define CR95HF_SUCCESS_CODE             0x00
#define CR95HF_FLAG_DATA_READY_MASK     0xF8
#define CMD_CMD                         0x00
#define CMD_READ                        0x02
/** DEFINE GLOBAL BUFFER OFFSET **/
#define CR95HF_COMMAND_OFFSET           0x00
#define CR95HF_LENGTH_OFFSET            0x01
#define CR95HF_DATA_OFFSET              0x02

/** DEFINE PROTOCOL ISO 14443-A **/
#define PROTOCOL_TAG_ISO14443A          0x02
#define CONTROL_14443A_NBBYTE           0x03
#define SELECT_BUFFER_SIZE              6
#define IDLE_BUFFER_SIZE                16
#define SENDRECV_BUFFER_SIZE            257
#define RDREG_BUFFER_SIZE               5
#define READ_REGISTER                   0x08
#define WRREG_BUFFER_SIZE               257
#define WRITE_REGISTER                  0x09
#define MAX_BUFFER_SIZE                 128
#define DUMMY_BYTE                      0xFF
void CR95HF_poll();
void CR95HF_sendcmd(int *data);
void CR95HF_recvresponse(int *data);
/** DEFINE ISO15693 **/
#define MAX_LENTH_DATA                  64
uint8_t rx_buffer[MAX_LENTH_DATA];
uint8_t UID[8];
uint8_t data[70];
uint8_t res[66];
uint8_t dataread[84];
volatile int ck = 0;
int write       = 0;
int error       = 0;
int timeout     = 0;
int numcount    = 0;
long cc;

void clear_buffer(uint8_t *data,int size);
int ISO15693_protocolselect();
int ISO15693_inventory(uint8_t *UID);
int ISO15693_IDN();
int ISO15693_Echo();
int ISO15693_Write(uint8_t* datawrite,int size);
int ISO15693_Read(uint8_t block_num, uint8_t *response);
int ISO15693_Read_Multi(uint8_t *response, int datalen, uint8_t block_start);
int check_same_data(uint8_t *data1, uint8_t *data2, int size);
void makedata(uint8_t *data, uint8_t *temp);
void TIMER_init();
int main(void)
{
  _delay_ms(1000);
  CR95HF_Wakeup();
  SPI_init();
  DDRB |= (1 << PB3);
  init_alert();
  UART_init();
  TIMER_init();
  sei();
  /* Replace with your application code */
  while (1) 
  {
    if (ck == 1)
    {
      int checkuart = 0;
      ISO15693_IDN();
      ISO15693_protocolselect();
      clear_buffer(dataread,84);
      int checkread = ISO15693_Read_Multi(dataread,60,0x00);
      if (checkread == 1)
      {
        checkuart = UART_send_multibyte(dataread,60);
      }
      if (checkuart > 0)
      {
        PORT_LED_STT &= ~(1 << PIN_LED_STT);
        PORT_LED_STT1 |= (1 << PIN_LED_STT1);
        beepbeep(2);
        ck = 0;
      }
      if (write == 1)
      {
        PORT_LED_STT1 &= ~(1 << PIN_LED_STT1);
        ISO15693_protocolselect();
        int check = ISO15693_inventory(UID);
        TIMSK |= ( 1 << TOIE0);
        while((check == 0) && (timeout < 200))
        {
          check = ISO15693_inventory(UID);
        }
        TIMSK &= ~(1 << TOIE0);
        if ((check == 0) || (timeout >= 30))
        {
          error = 1;
        }
        timeout = 0;
        if (error != 1)
        {
          int check_data_write_not_error = 0;
          TIMSK |= ( 1 << TOIE0);
          while(check_data_write_not_error == 0 && (timeout < 30))  
          /**when writing success, reading again, 
            *If data read = data write, and timeout < 10s 
            *=> writing is not error **/
          {
            clear_buffer(data,70);
            makedata(data,rx_buffer);
            ISO15693_Write(data,60);
            ISO15693_protocolselect();
            clear_buffer(dataread,84);
            ISO15693_Read_Multi(dataread,60,0x00);
            check_data_write_not_error = check_same_data(data,dataread,60);
          }
          TIMSK &= ~(1 << TOIE0);
          if ((check_data_write_not_error == 0) || (timeout >= 30))
          {         
            beepbeep(4);
            write = 0;  
          }
          else
          {
            beepbeep(3);
            write = 0;
          }
          timeout = 0;
        }
        else
        {
          write = 0;
        }
      }
      _delay_ms(300);
    }
    if (ck == 0)
    {
      PORT_LED_STT |= (1 << PIN_LED_STT);
      ISO15693_protocolselect();
      ck = ISO15693_inventory(UID);
      if (write == 1)
      {
        PORT_LED_STT1 &= ~(1 << PIN_LED_STT1);
        ISO15693_protocolselect();
        int check = ISO15693_inventory(UID);
        TIMSK |= ( 1 << TOIE0); 
        while((check == 0) && (timeout < 30))
        {
          check = ISO15693_inventory(UID);
        }
        TIMSK &= ~(1 << TOIE0);
        if ((check == 0) || (timeout >= 30))
        {
          error = 1;
        }
        timeout = 0;
        if (error != 1)
        {
          int check_data_write_not_error = 0;
          TIMSK |= ( 1 << TOIE0);
          while(check_data_write_not_error == 0 && (timeout < 30))  
          /**when writing success, reading again, 
            *If data read = data write, and timeout < 10s 
            *=> writing is not error **/
          {
            clear_buffer(data,70);
            makedata(data,rx_buffer);
            ISO15693_Write(data,60);
            ISO15693_protocolselect();
            clear_buffer(dataread,84);
            ISO15693_Read_Multi(dataread,60,0x00);
            check_data_write_not_error = check_same_data(data,dataread,60);
          }
          TIMSK &= ~(1 << TOIE0);
          if ((check_data_write_not_error == 0) || (timeout >= 30))
          {
            beepbeep(4);
            write = 0;
          }
          else
          {
            beepbeep(3);
            write = 0;
          }
          timeout = 0;
        }
        else
        {
          write = 0;
        }
      }
    }
  }
}
int ISO15693_protocolselect()
{
  uint8_t temp[64];
  clear_buffer(temp,64);
  write_low_cr95();
  SPI_tranrecv(0x00);                         // SPI control byte to send command to CR95HF
  SPI_tranrecv(0x02);                         // Set protocol command
  SPI_tranrecv(0x02);                         // length of data to follow
  SPI_tranrecv(0x01);                         // code for ISO/IEC 15693
  SPI_tranrecv(0x09);                         // Wait for SOF, 100% modulation, append CRC
  write_high_cr95();
  write_low_cr95();
  while(temp[0] != 0x08)
  {
    SPI_tranrecv(0x03);                       // SPI control byte to send polling command to CR95HF
    temp[0] = SPI_tranrecv(0x03);             // Write 3 until
    temp[0] &= 0xF8;                          // Mask: 0000 1xxx
  }
  write_high_cr95();
  write_low_cr95();
  SPI_tranrecv(0x02);                         // SPI control byte to send read command to CR95HF
  temp[0] = SPI_tranrecv(0xFF);               // Read response code
  temp[1] = SPI_tranrecv(0xFF);               // Read length of data response
  write_high_cr95();
  if ((temp[0] == 0x00) && (temp[1] == 0x00))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
int ISO15693_inventory(uint8_t *UID)
{
  uint8_t temp[64];
  clear_buffer(temp,64);
  write_low_cr95();
  SPI_tranrecv(0x00);                          // SPI control byte to send command to CR95HF
  SPI_tranrecv(0x04);                          // Send Receive CR95HF command
  SPI_tranrecv(0x03);                          // length of data that follows is 0
  SPI_tranrecv(0x26);                          // request Flags byte
  SPI_tranrecv(0x01);                          // Inventory Command for ISO/IEC 15693
  SPI_tranrecv(0x00);                          // mask length for inventory command
  write_high_cr95();
  write_low_cr95();
  while(temp[0] != 0x08)
  {
    SPI_tranrecv(0x03);                        // SPI control byte to send polling command to CR95HF
    temp[0] = SPI_tranrecv(0x03);              // Write 3 until
    temp[0] &= 0xF8;                           // Mask: 0000 1xxx
  }
  write_high_cr95();
  write_low_cr95();
  SPI_tranrecv(0x02);                          // SPI control byte to send read command to CR95HF
  temp[0] = SPI_tranrecv(0xFF);                // Read response code
  temp[1] = SPI_tranrecv(0xFF);                // Read length of data response
  if (temp[1] != 0x00)
  {
    for(int i = 0; i < temp[1]; i++)
    {
      temp[i + 2] = SPI_tranrecv(0xFF);        // Read all data response
    } 
  }
  write_high_cr95();
  if (temp[0] == 0x80)
  {
    for (int i = 11; i>= 4; i--)
    {
      UID[11 - i] = temp[i];
    } 
    return 1;
  }
  else
  {
    return 0;
  } 
}
int ISO15693_IDN()
{
  uint8_t temp[64];
  clear_buffer(temp,64);
  write_low_cr95();
  SPI_tranrecv(0x00);                          // SPI control byte to send command to CR95HF
  SPI_tranrecv(0x01);                          // IDN command
  SPI_tranrecv(0x00);                          // length of data that follows is 0
  write_high_cr95();
  write_low_cr95();
  while(temp[0] != 0x08)
  {
    SPI_tranrecv(0x03);                        // SPI control byte to send polling command to CR95HF
    temp[0] = SPI_tranrecv(0x03);              // Write 3 until
    temp[0] &= 0xF8;                           // Mask: 0000 1xxx
  }
  write_high_cr95();
  write_low_cr95();
  SPI_tranrecv(0x02);                          // SPI control byte to send read command to CR95HF
  temp[0] = SPI_tranrecv(0xFF);                // Read response code
  temp[1] = SPI_tranrecv(0xFF);                // Read length of data response
  if (temp[1] != 0x00)
  {
    for (int i = 0; i < temp[1]; i++)
    {
      temp[i + 2] = SPI_tranrecv(0xFF);        // Read all data response
    }   
  }
  write_high_cr95();
  if((temp[0] == 0x00) && (temp[1] = 0x0F))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
int ISO15693_Echo()
{
  uint8_t temp[8];
  write_low_cr95();
  SPI_tranrecv(0x00);                          // SPI control byte to send command to CR95HF  
  SPI_tranrecv(0x55);                          // Echo command
  write_high_cr95();
  write_low_cr95();
  while(temp[0] != 0x08)
  {
    SPI_tranrecv(0x03);                        // SPI control byte to send polling command to CR95HF
    temp[0] = SPI_tranrecv(0x03);              // Write 3 until
    temp[0] &= 0xF8;                           // Mask: 0000 1xxx
  }
  write_high_cr95();
  write_low_cr95();
  SPI_tranrecv(0x02);                          // SPI control byte to send read command to CR95HF
  temp[0] = SPI_tranrecv(0xFF);                // Read response code
  write_high_cr95();
  if (temp[0] == 0x55)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
int ISO15693_Write(uint8_t* datawrite,int size)
{
  int i = 0, k = 0;
  int result = 0;
  uint8_t temp[64];
  clear_buffer(temp,64);
  int num_block_to_write = ((size % 4) == 0) ? (size / 4) : (size / 4  + 1);
  int buffer[num_block_to_write][4];
  for (int m = 0; m < num_block_to_write; m++)
  {
    for(int n = 0; n < 4; n++)
    {
      if (i < size)
      { 
        buffer[m][n] = datawrite[i];
        i++; 
      }
    }
  }
  while((k < num_block_to_write))
  {
    while (result == 0)
    {
      write_low_cr95();
      SPI_tranrecv(0x00);                      // SPI control byte to send command to CR95HF
      SPI_tranrecv(0x04);                      // Send Receive CR95HF command
      SPI_tranrecv(0x07);                      // length of data that follows
      SPI_tranrecv(0x02);                      // request Flags byte
      SPI_tranrecv(0x21);                      // Write Single Block command for ISO/IEC 15693
      SPI_tranrecv(k);                         // Memory block address
      SPI_tranrecv(buffer[k][0]);              // first byte block of memory block
      SPI_tranrecv(buffer[k][1]);              // second byte block of memory block
      SPI_tranrecv(buffer[k][2]);              // third byte block of memory block
      SPI_tranrecv(buffer[k][3]);              // fourth byte block of memory block
      write_high_cr95();
      write_low_cr95();
      while(temp[0] != 0x08)
      {
        SPI_tranrecv(0x03);                    // SPI control byte to send polling command to CR95HF
        temp[0] = SPI_tranrecv(0x03);          // Write 3 until
        temp[0] &= 0xF8;                       // Mask: 0000 1xxx
      }
      write_high_cr95();
      write_low_cr95();
      SPI_tranrecv(0x02);                      // SPI control byte to send read command to CR95HF
      temp[0] = SPI_tranrecv(0xFF);            // Read response code
      temp[1] = SPI_tranrecv(0xFF);            // Read length of data response
      if (temp[1] != 0x00)
      {
        for (int m = 0; m < temp[1]; m++)
        {
          temp[i + 2] = SPI_tranrecv(0xFF);
        }
      }
      write_high_cr95();
      if (temp[0] == 0x80)
      {
        result = 1;
      }
      else
      {
        result = 0;
      }
    }
    k++;
    result = 0;
  }
  return 1;
}
int ISO15693_Read(uint8_t block_num, uint8_t *response)
{
  uint8_t temp[64];
  write_low_cr95();
  SPI_tranrecv(0x00);                           // SPI control byte to send command to CR95HF
  SPI_tranrecv(0x04);                           // Send Receive CR95HF command
  SPI_tranrecv(0x03);                           // length of data that follows
  SPI_tranrecv(0x02);                           // request Flags byte
  SPI_tranrecv(0x20);                           // Read Single Block command for ISO/IEC 15693
  SPI_tranrecv(block_num);                      // memory block address
  write_high_cr95();
  write_low_cr95();
  while(temp[0] != 0x08)
  {
    SPI_tranrecv(0x03);                         // SPI control byte to send polling command to CR95HF
    temp[0] = SPI_tranrecv(0x03);               // Write 3 until
    temp[0] &= 0xF8;                            // Mask: 0000 1xxx
  }
  write_high_cr95();
  write_low_cr95();
  SPI_tranrecv(0x02);                           // SPI control byte to send read command to CR95HF
  temp[0] = SPI_tranrecv(0xFF);                 // Read response code
  temp[1] = SPI_tranrecv(0xFF);                 // Read length of data response
  if(temp[1] != 0x00)
  {
    for(int i = 0; i < temp[1]; i ++)
    {
      temp[i + 2] = SPI_tranrecv(0xFF);
    }
  }
  write_high_cr95();
  if(temp[0] == 0x80)
  {
    for(int i = 0; i < temp[1]; i++)
    {
      response[i] = temp[i + 2];
    }
    return 1;
  }
  else
  {
    return 0;
  }
}
int ISO15693_Read_Multi(uint8_t *response, int datalen, uint8_t block_start)
{
  uint8_t temp[16];
  write_low_cr95();
  SPI_tranrecv(0x00);                           // SPI control byte to send command to CR95HF
  SPI_tranrecv(0x04);                           // Send Receive CR95HF command
  SPI_tranrecv(0x04);                           // length of data that follows
  SPI_tranrecv(0x02);                           // request Flags byte
  SPI_tranrecv(0x23);                           // Read Multiple Block command for ISO/IEC 15693
  SPI_tranrecv(block_start);                    // memory block address to start
  SPI_tranrecv(0x3C);                           // number of byte to read
  write_high_cr95();
  write_low_cr95();
  while(temp[0] != 0x08)
  {
    SPI_tranrecv(0x03);                         // SPI control byte to send polling command to CR95HF
    temp[0] = SPI_tranrecv(0x03);               // Write 3 until
    temp[0] &= 0xF8;                            // Mask: 0000 1xxx
  }
  write_high_cr95();
  write_low_cr95();
  SPI_tranrecv(0x02);                           // SPI control byte to send read command to CR95HF
  temp[0] = SPI_tranrecv(0xFF);                 // Read response code
  temp[1] = SPI_tranrecv(0xFF);                 // Read length of data response
  SPI_tranrecv(0xFF);
  if (temp[1] != 0x00)
  {
    for (int i = 0; i < temp[1]; i ++)
    {
      response[i] = SPI_tranrecv(0xFF);
    }
  }
  write_high_cr95();
  if (temp[0] == 0x80)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
void clear_buffer(uint8_t *data, int size)
{
  for (int i = 0; i < size; i++)
  {
    data[i] = 0x00;
  }
}
void SPI_init()
{
  PORTB |= (1 << PB4);
  DDRB |= ( 1 << DDB4);
  SPCR |= (1 << MSTR) | (1 << SPR0);
  SPCR |= (1 << SPE);
  DDR_CR95_SPI |= (1 << PIN_CR95_MOSI) | (1 << PIN_CR95_SCK);
}
uint8_t SPI_tranrecv(uint8_t data)
{
  SPDR = data;
  while((SPSR & (1 << SPIF)) == 0);
  return SPDR;
}
void UART_init()
{
  UBRRH = 0;
  UBRRL = 12;
  UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
  UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
}
void UART_send(uint8_t data)
{
  while(check_bit_zero(UCSRA,UDRE)){};
  UDR = data;
}
int UART_send_multibyte(uint8_t *data, int len)
{
  int i = 0;
  while(i < len)
  {
    UART_send(data[i]);
    i++;
  }
  return len;
}
void write_low_cr95()
{
  GPIO_WRITE_LOW(PORT_CR95_SPI,PIN_CR95_SS);
}
void write_high_cr95()
{
  GPIO_WRITE_HIGHT(PORT_CR95_SPI,PIN_CR95_SS);
}
int check_bit_zero(int reg, int bit)
{
  return (!(reg & (1 << bit)));
}
void init_alert()
{
  DDR_LED_STT  |= (1 << PIN_LED_STT);
  DDR_LED_STT1 |= (1 << PIN_LED_STT1);
  DDR_SPK   |= (1 << PIN_SPK);
  PORT_LED_STT |= (1 << PIN_LED_STT);
  PORT_LED_STT1 |= (1 << PIN_LED_STT1);
  PORT_SPK  |= (1 << PIN_SPK);
}
void CR95HF_Wakeup()
{
  DDR_IRQ |= (1 << PIN_IRQ);
  PORT_IRQ |= (1 << PIN_IRQ);
  _delay_ms(100);
  PORT_IRQ |= (0 << PIN_IRQ);
  _delay_us(1000);
  PORT_IRQ |= (1 << PIN_IRQ);
}
void CR95HF_poll()
{
  int polling_status = 0;
  write_low_cr95();
  while(polling_status != CR95HF_FLAG_DATA_READY_MASK )
  {
    SPI_tranrecv(CMD_POLL);
    polling_status = SPI_tranrecv(CMD_POLL);
    polling_status &= CR95HF_FLAG_DATA_READY_MASK;
  }
  write_high_cr95();
  PORT_LED_STT &= ~(1 << PIN_LED_STT);
}
void CR95HF_sendcmd(int *data)
{
  write_low_cr95();
  SPI_tranrecv(CMD_CMD);
  if (*data == ECHO)
  {
    SPI_tranrecv(ECHO);
  }
  else
  {
    int len = data[1];
    SPI_tranrecv(len);
    for(int i = 0; i < len; i++)
    {
      SPI_tranrecv(data[i+2]);
    }
  }
  write_high_cr95();
}
void CR95HF_recvresponse(int *data)
{
  write_low_cr95();
  SPI_tranrecv(CMD_READ);
  data[0] = SPI_tranrecv(DUMMY_BYTE);
  if (data[0] == ECHO)
  {
    data[1] = 0x00;
  }
  else if (data[0] == 0xFF)
  {
    data[1] = 0x00;
  }
  else
  {
    data[1] = SPI_tranrecv(DUMMY_BYTE);
    if(data[1] != 0)
    {
      for (int i = 0; i < data[1]; i++)
      {
        data[i+2] = SPI_tranrecv(DUMMY_BYTE);
      }
    }
  }
  write_high_cr95();
}
void beepbeep(int n)
{
  for (int i = 0; i < n; i++ )
  {
    PORT_SPK &= ~(1 << PIN_SPK);
    _delay_ms(300);
    PORT_SPK |= (1 << PIN_SPK);
    _delay_ms(100); 
  }
}
void makedata(uint8_t *data,uint8_t *temp)
{
  for (int i = 0; i < 60; i ++)
  {
    data[i] = temp[i];
  }
}
void TIMER_init()
{
  TCCR0 = (1 << CS01) | ( 1 << CS00);   //Scaler = 64
  TCNT0 = 131;
}
int check_same_data(uint8_t *data1, uint8_t *data2, int size)
{
  for (int i = 0; i < size; i++)
  {
    if (data1[i] != data2[i])
    {
      return 0;
    }
  }
  return 1;
}
ISR(USART_RXC_vect)
{
  rx_buffer[numcount] = UDR;
  if ((rx_buffer[numcount] == '\n') && (numcount > 59))
  {
    write = 1;
    numcount = 0;
  }
  else
  {
    numcount++;
  }
}
ISR(TIMER0_OVF_vect)
{
  cc++;
  if (cc == 300)
  {
    beepbeep(1);
    PORT_LED_STT1 ^= (1 << PIN_LED_STT1);
    cc = 0;
    timeout ++;
  }
  TCNT0 = 131;
}
