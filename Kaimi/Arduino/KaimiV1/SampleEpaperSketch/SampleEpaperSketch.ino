/*
 * Hello world sample for controlling an ePaperShield from an Arduino
 *
 * This version is for an Arduino Leonardo.
 * Changes may be required for other types of Arduino.
 * On the Leonardo,
 *    "Serial" refers to the serial link to the PC (the virtual COMM port).
 *    "Serial3" refers to pins 0 (RX1) and 1 (TX1) on the headers.
 *
 * Copyright (C) 2013 Leland Morrison.
 *
This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#define  ENABLE_ECHO_MESSAGES   1

char      outbuf[40];
int       msg_count;

char      abuf[40];

const char hexascii[] = "0123456789ABCDEF";

int  print_char_to_monitor( char ch )
{
  if ((ch >= ' ') && (ch < 126))
    Serial.write( ch );
  else if ((ch == '\r') || (ch == '\n'))
    Serial.write( ch );
  else if (ch == 0xff)
    Serial.write( "." );
  else
  {
    char  digits[3];
    
    digits[0] = hexascii[ (ch >> 4) & 0x0f ];
    digits[1] = hexascii[  ch       & 0x0f ];
    digits[2] = 0;
    
    Serial.write( "\\x" );
    Serial.write( digits );
  }
}

char  echo_buf[4];

int  send_char_wait_echo( char ch )
{
  int  cnt;
  
  Serial3.write( ch );
  
  cnt = 1;
  while( (cnt != 0) && (echo_buf[0]!=ch) )
  {
    cnt = Serial3.readBytes( echo_buf, 1 );
    
#if 0
    if (cnt == 1)
      print_char_to_monitor( echo_buf[0] );
    else 
#endif
    if (cnt == 0)
    {
      Serial.write( "|" );
      Serial.write(  ch );
    }
    else if (cnt != 1)
    {
      Serial.write( "^" );
      Serial.write(  ch );
    }
  }
  
  if (ch == '\r')
  {
    /* attempt to grab the nl that should be following along */
    cnt = Serial3.readBytes( echo_buf+1, 1 );
    
#if 0
    if (cnt == 1)
      print_char_to_monitor( echo_buf[1] );
    else
#endif
    if (cnt == 0)
      Serial.write( "missing nl " );
    else if (cnt != 1)
      Serial.write( "^nl " );
  }
  
  return echo_buf[0];
}

int  eps_send_with_echo_wait( const char * s )
{
  int  pos = 0;
  
  while (s[pos])
  {
    send_char_wait_echo( s[pos] );
    
    ++pos;
  }
}

int  eps_clear_inbuf( void )
{
  send_char_wait_echo( '\r' );
  send_char_wait_echo( '\r' );
}

#if 1
char  ch_buf[4];

unsigned  eps_getline( char response_buffer[], unsigned max_response_size, long gl_timeout )
{
  unsigned long t_end;
  unsigned      cnt, gotc;
  int           ch;
  
  /* Wait for a one-line response from the ePS */
  //Serial.write( "gl> " );
  
  t_end = millis( );
  //Serial.println( t_end );
  t_end += gl_timeout;
  //Serial.println( t_end );
  
  cnt = 0;
  ch_buf[0] = 0;
  do {
    /* invoke setTimeout before each readBytes( ) */
    Serial3.setTimeout( 200 );
    gotc = Serial3.readBytes( ch_buf, 1 );
    if (gotc > 0)
    {
       response_buffer[cnt++] = ch_buf[0];
       //print_char_to_monitor( ch_buf[0] );
    }
  } while ((ch_buf[0] != '\n') &&
           (t_end >= millis( )) &&
           (cnt < max_response_size-1));
  response_buffer[cnt] = 0;
  
#if 0
  if (ch_buf[0] == '\n')
    Serial.write( "got nl  " );
  else if (cnt >= max_response_size-1)
  {
    Serial.write( "cnt=" );
    Serial.println( cnt );
  }
  else if (t_end >= millis())
  {
    Serial.write( "tend=" );
    Serial.println( t_end );
    Serial.write( " vs " );
    Serial.println( millis() );
  }
#endif

#if 0
  while ( (cnt > 0) && (response_buffer[cnt-1] == '\r') || (response_buffer[cnt-1] == '\n') )
  {
     --cnt;
     response_buffer[cnt] = 0;
  }
#endif

#if 0
  Serial.println( "<" );
  
  Serial.write( "cnt=" );
  Serial.println( cnt );
#endif
  
  return cnt;
}
#else
unsigned  eps_getline( char response_buffer[], unsigned max_response_size )
{
  unsigned  cnt;
   
  /* Wait for a one-line response from the ePS */
  Serial3.setTimeout( 4000 );
   
  cnt = Serial3.readBytesUntil( '\n', response_buffer, max_response_size );
   
  return cnt;
}
#endif

int  eps_op( const char *s_op )
{
  unsigned  cnt;
   
  /* Send a command to the ePS, wait for the command echo */
  eps_send_with_echo_wait( s_op );
  Serial.write( s_op );
   
  if (send_char_wait_echo( '\r' ) != '\r')
  {
     eps_clear_inbuf( );
  }
  
  Serial.write( "\n" );
  
  return  cnt > 0;
}
 

int  eps_op_with_response( const char *s_op, long timeout )
{
  unsigned  cnt;
   
  /* Send a command to the ePS, wait for the command echo */
  cnt = eps_op( s_op );
  
  Serial.write( ">" );
  
  /* AND read a line of output */
  cnt = eps_getline( abuf, sizeof(abuf)-2, timeout );
  abuf[sizeof(abuf)-1] = 0;
  abuf[cnt] = 0;
   
  if (cnt <= 0)
  {
     Serial.write( "timeout for response\r\n" );
  }
  else
  {
     //Serial.write( "Response was: " );
     Serial.write( abuf );
     Serial.write( "\n" );
  }
}

void  setup() {
  Serial.begin( 115200 );  // USB CDC
  Serial3.begin( 115200 );  // Rx0, tx0 pins
   
  // blinky LED setup
  pinMode( 13, OUTPUT );
   
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo
  }
   
  /* Wait for a one-line response from the ePS */
  Serial3.setTimeout( 2500 );
  
  eps_op( "reset" );
  delay( 50 );
  
  msg_count = 0;
  
  eps_clear_inbuf( );
  
  // start up the display
  eps_op_with_response( "on", 2500 );
}


void  loop( ) {
  Serial.write( "************************************\r\n" );
  Serial.write( "loop top \r\n" );
  //Serial.println( millis( ) );
  msg_count++;
  
  // Blank the internal bitmap
  eps_op( "clear" );
  
  // Send a string to be written on the internal bitmap
  // Note that the leading quote (') is not part of the text.
  sprintf( outbuf, "'Hello World %2d!", msg_count );
  eps_op( outbuf );
  
  // Set the top-left coordinates to write the text at
  eps_op( "10 10 moveto" );
  
  // Draw the text
  eps_op( "draw_text" );
  
  //Serial.println( millis( ) );
  // Send the internal bitmap to the display
  // transfer the image from the internal bitmap to the display
  eps_op_with_response( "display_render", 2500 );
  
  // turn off the display
  eps_op_with_response( "off", 5000 );
  //Serial.println( millis( ) );
  
  
  /* blink the LED */
  digitalWrite( 13, LOW  );
  
  delay(8000);
  
  //Serial3.setTimeout( 2000 );
  Serial3.readBytes( abuf, sizeof(abuf)-2 );
  Serial.write( "ml rx> " );
  Serial.write( abuf );
  
  digitalWrite( 13, HIGH );
  delay(1000);
  
  eps_getline( abuf, sizeof(abuf)-2, 2000 );
  
  //Serial.println( millis( ) );
  
  // Erase the display
  // this uses the internal bitmap to send a negative image
  // which reduces ghosting problems.
  
  // start up the display
  eps_op_with_response( "on", 2500 );
  //Serial.println( millis( ) );
  
  // transfer the image from the internal bitmap to the display
  eps_op_with_response( "display_erase", 2500 );
  //Serial.println( millis( ) );
  
  // ^^ back to loop top ^^
 }
 
