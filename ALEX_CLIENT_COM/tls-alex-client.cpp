#include <ncurses.h>
// Routines to create a TLS client
#include <iostream>

#include "make_tls_client.h"

// Network packet types
#include "netconstants.h"

// Packet types, error codes, etc.
#include "constants.h"

#define FORWARD_BAKWARD_DIST    2
#define LEFT_RIGHT_ANGLE        5
#define PWM_TURN                100
#define PWM_STRAIGHT            100

static volatile int networkActive = 0;

void handleError(const char *buffer)
{
  switch(buffer[1])
  {
    case RESP_OK:
      printw("Command / Status OK\n");
      break;

    case RESP_BAD_PACKET:
      printw("BAD MAGIC NUMBER FROM ARDUINO\n");
      break;

    case RESP_BAD_CHECKSUM:
      printw("BAD CHECKSUM FROM ARDUINO\n");
      break;

    case RESP_BAD_COMMAND:
      printw("PI SENT BAD COMMAND TO ARDUINO\n");
      break;

    case RESP_BAD_RESPONSE:
      printw("PI GOT BAD RESPONSE FROM ARDUINO\n");
      break;

    default:
      printw("PI IS CONFUSED!\n");
      break;
  }
  refresh();
}

void handleStatus(const char *buffer)
{
  int32_t data[16];
  memcpy(data, &buffer[1], sizeof(data));

  printw("\n ------- ALEX STATUS REPORT ------- \n");
  printw("Left Forward Ticks:\t\t%d\n", data[0]);
  printw("Right Forward Ticks:\t\t%d\n", data[1]);
  printw("Left Reverse Ticks:\t\t%d\n", data[2]);
  printw("Right Reverse Ticks:\t\t%d\n", data[3]);
  printw("Left Forward Ticks Turns:\t%d\n", data[4]);
  printw("Right Forward Ticks Turns:\t%d\n", data[5]);
  printw("Left Reverse Ticks Turns:\t%d\n", data[6]);
  printw("Right Reverse Ticks Turns:\t%d\n", data[7]);
  printw("Forward Distance:\t\t%d\n", data[8]);
  printw("Reverse Distance:\t\t%d\n", data[9]);
  double distance = data[10] * 0.034 / 2;
  printw("Ultrasonic Distance:\t\t%lf\n", distance );
  printw("\n---------------------------------------\n");
  refresh();
}

void handleColour ( const char *buffer ) {
  int32_t data[16];
  memcpy(data, &buffer[1], sizeof(data));

  int32_t r_value = data[0];
  int32_t g_value = data[1];
  int32_t b_value = data[2];

  printw ( "\n ---- ALEX COLOUR REPORT ---- \n" );
  printw ( "RED: \t\t%d\n", r_value );
  printw ( "GREEN: \t\t%d\n", g_value );
  printw ( "BLUE: \t\t%d\n", b_value );

  if ( ( r_value < 260 && r_value > 155 ) 
      && ( g_value < 360 && g_value > 250 ) 
      && ( b_value < 270 && b_value > 170 ) ) {
    if ( r_value + g_value + b_value < 660 ) {
      printw ( "Colour detected is ORANGE\n" );
    } else { 
      printw ( "Colour detected is RED\n" );
    }
  } else if ( ( r_value < 370 && r_value > 300 ) 
      && ( g_value < 290 && g_value > 220 ) 
      && ( b_value < 240 && b_value > 180 ) ) {
    printw ( "Colour detected is GREEN\n" );
  } else if ( ( r_value < 320 && r_value > 230 ) 
      && ( g_value < 240 && g_value > 170 ) 
      && ( b_value < 250 && b_value > 170 ) ) {
    printw ( "Colour detected is GREEN\n" );
  } else {
    printw ( "NONE\n" );
  }
  refresh();
}

void handleMessage(const char *buffer)
{
  printw("MESSAGE FROM ALEX: %s\n", &buffer[1]);
  refresh();
}

void handleCommand(const char *buffer)
{

}

void handleNetwork(const char *buffer, int len)
{
  // The first byte is the packet type
  int type = buffer[0];

  switch (type)
  {
    case NET_ERROR_PACKET:
      handleError(buffer);
      break;

    case NET_STATUS_PACKET:
      handleStatus(buffer);
      break;

    case NET_MESSAGE_PACKET:
      handleMessage(buffer);
      break;

    case NET_COMMAND_PACKET:
      handleCommand(buffer);
      break;

    case NET_COLOUR_PACKET:
      handleColour ( buffer );
      break;
  }
}

void sendData(void *conn, const char *buffer, int len)
{
  printw("\nSENDING %d BYTES DATA\n\n", len);
  refresh();

  if(networkActive)
  {
    int bytes_written_to_ssl = sslWrite(conn, buffer, len);
    networkActive = (bytes_written_to_ssl > 0);
  }

}

void *readerThread(void *conn)
{
  char buffer[128];

  while(networkActive)
  {
    int len = sslRead(conn, buffer, sizeof(buffer));
    printw("read %d bytes from server.\n", len);
    refresh();

    networkActive = (len > 0);

    if (networkActive) {
      handleNetwork(buffer, len);
    }
  }

  printw("Exiting network listener thread\n");
  refresh();

  stopClient();
  EXIT_THREAD(conn);
}

void flushInput()
{
  char c;

  while((c = getchar()) != '\n' && c != EOF);
}

void getParams(int32_t *params)
{
  printw("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
  printw("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
  refresh();
  scanw("%d %d", &params[0], &params[1]);
}

void getParamsClearCounter(int32_t *params)
{
  printw("Enter value to clear counter.\n");
  printw("0 - CLEAR ALL COUNTERS\n");
  printw("1 - Clear leftForwardTicks\n");
  printw("2 - Clear rightForwardTicks\n");
  printw("3 - Clear leftReverseTicks\n");
  printw("4 - Clear rightReverseTicks\n");
  printw("5 - Clear leftForwardTicksTurns\n");
  printw("6 - Clear rightForwardTicksTurns\n");
  printw("7 - Clear leftReverseTicksTurns\n");
  printw("8 - Clear rightReverseTicksTurns\n");
  printw("9 - Clear forwardDist\n");
  printw("10 - Clear reverseDist\n");
  refresh();
  scanw("%d", &params[0]);
  params[1] = 0; 
}

void *writerThread(void *conn)
{
  int quit = 0;

  while (!quit)
  {
    char command;
    int32_t ch;

    initscr();
    cbreak();
    keypad ( stdscr, TRUE );
    scrollok( stdscr, TRUE );
    noecho();

    printw("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, c=clear stats, g=get stats, o=get colour, q=exit)\n");
    refresh();

    char buffer[10];
    int32_t params[2];

    buffer[0] = NET_COMMAND_PACKET;

    ch = getch();

    if ( ch == KEY_UP || ch == KEY_DOWN || ch == KEY_LEFT || ch == KEY_RIGHT ) 
    {
      switch ( ch ) {
        case KEY_UP:
          buffer[1] = 'f';
          params[0] = FORWARD_BAKWARD_DIST;
          params[1] = PWM_STRAIGHT;
          break;

        case KEY_DOWN:
          buffer[1] = 'b';
          params[0] = FORWARD_BAKWARD_DIST;
          params[1] = PWM_STRAIGHT;
          break;

        case KEY_LEFT:
          buffer[1] = 'l';
          params[0] = LEFT_RIGHT_ANGLE;
          params[1] = PWM_TURN;
          break;

        case KEY_RIGHT:
          buffer[1] = 'r';
          params[0] = LEFT_RIGHT_ANGLE;
          params[1] = PWM_TURN;
          break;
      }
      memcpy ( &buffer[2], params, sizeof(params) );
      sendData (conn, buffer, sizeof(buffer) );
      endwin();
    } else 
    {
      endwin();
      nocbreak();
      keypad ( stdscr, FALSE );
      echo();

      command  = static_cast < char >  (ch); 
      switch( command )
      {
        case 'f':
        case 'F':
        case 'b':
        case 'B':
        case 'l':
        case 'L':
        case 'r':
        case 'R':
          getParams(params);
          buffer[1] = command;
          memcpy ( &buffer[2], params, sizeof(params) );
          sendData (conn, buffer, sizeof(buffer) );
          break;

        case 'c':
        case 'C':
          getParamsClearCounter(params);
          buffer[1] = command;
          memcpy ( &buffer[2], params, sizeof(params) );
          sendData (conn, buffer, sizeof(buffer) );
          break;

        case 's':
        case 'S':
        case 'g':
        case 'G':
        case 'o':
        case 'O':
          buffer[1] = command;
          params[0]=0;
          params[1]=0;
          memcpy(&buffer[2], params, sizeof(params));
          sendData(conn, buffer, sizeof(buffer));
          break;

        case 'q':
        case 'Q':
          quit = 1;
          break;

        default:
          printf("BAD COMMAND\n");
          break;

      }
    }
  }

  endwin();
  nocbreak();
  keypad ( stdscr, FALSE );
  echo();

  printf("Exiting keyboard thread\n");

  stopClient();
  EXIT_THREAD(conn);
}

#define SERVER_NAME "172.20.10.5" // to change ip address if connected to a diff hotspot
#define CA_CERT_FNAME "signing.pem"
#define PORT_NUM 5000
#define CLIENT_CERT_FNAME "laptop.crt"
#define CLIENT_KEY_FNAME "laptop.key"
#define SERVER_NAME_ON_CERT "CancerCar"

void connectToServer()
{
  createClient(SERVER_NAME, PORT_NUM, 1, CA_CERT_FNAME, SERVER_NAME_ON_CERT, 1, CLIENT_CERT_FNAME, CLIENT_KEY_FNAME, readerThread, writerThread);
}

int main( )
{
  networkActive = 1;
  connectToServer();


  while(client_is_running());

  printw("\nMAIN exiting\n\n");
  refresh();
}

