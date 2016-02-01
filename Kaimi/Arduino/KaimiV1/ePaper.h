class ePaper {
  
private:
char abuf[40];
char ch_buf[4];
char echo_buf[4];

char  send_char_wait_echo(char ch) {
  int  cnt = 1;
  
  Serial3.write(ch);

  while ((cnt != 0) && (echo_buf[0] != ch)) {
    cnt = Serial3.readBytes(echo_buf, 1);
  }
  
  if (ch == '\r') {
    /* Attempt to grab the nl that should be following along */
    cnt = Serial3.readBytes(echo_buf + 1, 1 );
  }
  
  return echo_buf[0];
}

void  eps_send_with_echo_wait(const char* s) {
  int  pos = 0;
  
  while (s[pos]) {
    send_char_wait_echo(s[pos]);
    ++pos;
  }
}

int  eps_clear_inbuf(void) {
  send_char_wait_echo( '\r' );
  send_char_wait_echo( '\r' );
}
public:
void eps_getline(char response_buffer[], unsigned max_response_size, long gl_timeout) {
  unsigned long t_end;
  unsigned      cnt;
  unsigned      gotc;
  int           ch;
  
  /* Wait for a one-line response from the ePS */
  t_end = millis( );
  t_end += gl_timeout;
  
  cnt = 0;
  ch_buf[0] = 0;
  do {
    /* invoke setTimeout before each readBytes( ) */
    Serial3.setTimeout(200);
    gotc = Serial3.readBytes( ch_buf, 1 );
    if (gotc > 0) {
       response_buffer[cnt++] = ch_buf[0];
    }
  } while ((ch_buf[0] != '\n') &&
           (t_end >= millis()) &&
           (cnt < max_response_size-1));
  response_buffer[cnt] = 0;  
 }

public:

ePaper() {
  Serial3.begin(115200);
  while (!Serial3) {}
  Serial3.setTimeout(2500);
  eps_op("reset");
  delay(50);
  eps_clear_inbuf();
}

void eps_op(const char* s_op) {
  unsigned  cnt;

  /* Send a command to the ePS, wait for the command echo */
  eps_send_with_echo_wait(s_op);
  if (send_char_wait_echo( '\r' ) != '\r') {
     eps_clear_inbuf();
  }
}
 
void  eps_op_with_response(const char *s_op, long timeout) {
  /* Send a command to the ePS, wait for the command echo */
  eps_op( s_op );
  
  /* AND read a line of output */
  eps_getline(abuf, sizeof(abuf) - 2, timeout);
}

};

