#include "epaper.h"

ePaper* epaper;
int msgCount = 0;
char tempString[128];

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  epaper = new ePaper();
  //epaper->eps_op_with_response("on", 2500);
  epaper->eps_op("'dejaVuSansMono8");
  epaper->eps_op("select_font");
}

String jetsonStatus = "???";
String pi1Status = "ERR";

String lastCommand = "cmdVel 0.0 1.3";

void loop() {
  epaper->eps_op_with_response("on", 1500); // Must turn on each loop?
  epaper->eps_op("clear");

  epaper->eps_op("0 0 moveto");
  sprintf(tempString, "'Jet: %3s | Pi1: %3s", jetsonStatus.c_str(), pi1Status.c_str());
  epaper->eps_op(tempString);
  epaper->eps_op("draw_text");


  epaper->eps_op("next_line");
  sprintf(tempString, "'CMD: %s", lastCommand.c_str());
  epaper->eps_op(tempString);
  epaper->eps_op("draw_text");
 
  epaper->eps_op("next_line");
  sprintf(tempString, "'Left:%3d | Right:%3d", msgCount++, 1003 + msgCount);
  epaper->eps_op(tempString);
  epaper->eps_op("draw_text");

  epaper->eps_op_with_response("14 display_transfer", 500);
}
