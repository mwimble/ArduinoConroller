#ifndef __MAP_H
#define __MAP_H

#include "QuadratureEncoder.h"

extern Log logger;
extern QuadratureEncoder quadratureEncoder;

class Map {
  public:
  enum TTurn {
    L,
    C,
    R,
    DO_NOT_USE_TURN};
    
  enum TLogicalHeading {
    N,
    E,
    S,
    W,
    DO_NOT_USE_HEADING};
    
  Map(Map* parent, TTurn turn, TLogicalHeading logicalHeading, float realHeading, bool hasLeftTurn, bool hasRightTurn, bool hasContinuation) :
    center_(NULL),
    has_continuation_(hasContinuation),
    has_left_turn_(hasLeftTurn),
    has_right_turn_(hasRightTurn),
    heading_degrees_(realHeading),
    left_(NULL),
    length_(0),
    logical_heading_(logicalHeading), 
    logical_x_(0),
    logical_y_(0),
    loop_detected_(false),
    odometer_(QuadratureEncoder::Counter()),
    parent_(parent),
    right_(NULL),
    turn_(turn) {
    if (parent) {
      length_ = odometer_ - parent->odometer_;
      switch (logicalHeading) {
        case N:
        logical_x_ = parent->logical_x_;
        logical_y_ = parent->logical_y_ + length_;
        break;

        case E:
        logical_x_ = parent->logical_x_ + length_;
        logical_y_ = parent->logical_y_;
        break;
        
        case S:
        logical_x_ = parent->logical_x_;
        logical_y_ = parent->logical_y_ - length_;
        break;
        
        case W:
        logical_x_ = parent->logical_x_ - length_;
        logical_y_ = parent->logical_y_;
        break;
      }
      
      switch (turn) {
        case L:
        parent->left_ = this;
        break;
        
        case C:
        parent->center_ = this;
        break;
        
        case R:
        parent->right_ = this;
        break;
      }
    } else {
      logical_x_ = 0;
      logical_y_ = 0;
    }
  }
  
  void Dump() {
    logger.print("== == MAP @");
    logger.println((long) this);
    logger.print("  x: ");
    logger.print(logical_x_);
    logger.print(", y: ");
    logger.print(logical_y_);
    logger.print(", turn: ");
    logger.print(kTurnName[turn_]);
    logger.print(", heading: ");
    logger.print(heading_degrees_);
    logger.print(", logical heading: ");
    logger.println(kHeadingName[logical_heading_]);
    
    logger.print("  length: ");
    logger.print(length_);
    logger.print(", odometer: ");
    logger.print(odometer_);
    logger.print(", loop detected: ");
    logger.print(loop_detected_ ? "T" : "f");
    logger.print(", has_left_turn: ");
    logger.print(has_left_turn_ ? "T" : "f");
    logger.print(", has_right_turn: ");
    logger.print(has_right_turn_ ? "T" : "f");
    logger.print(", has_continuation: ");
    logger.println(has_continuation_ ? "T" : "f");

    logger.print("left: ");
    logger.print((long) left_);
    logger.print(", center: ");
    logger.print((long) center_);
    logger.print(", right: ");
    logger.println((long) right_);
  }
  
  static void DumpTree(Map* node) {
    if (node) {
      node->Dump();
      if (node->left_) DumpTree(node->left_);
      if (node->center_) DumpTree(node->center_);
      if (node->right_) DumpTree(node->right_);
    } else {
      logger.print("NULL NODE");
    }
  }
  
  TLogicalHeading LogicalHeading() { return logical_heading_; }
  
  static const char* kHeadingName[DO_NOT_USE_HEADING];
  static const TLogicalHeading kLeftOf[DO_NOT_USE_HEADING];
  static const TLogicalHeading kRightOf[DO_NOT_USE_HEADING];
  
  static const char* kTurnName[DO_NOT_USE_TURN];
  
  private:

  Map*              center_;            // Where a continuation leads.
  bool              has_continuation_;
  bool              has_left_turn_;
  bool              has_right_turn_;
  float             heading_degrees_;   // Sensor heading.
  Map*              left_;              // Where a left turn leads.
  unsigned long     length_;            // Length along logical heading. 0 => not traveled yet.
  TLogicalHeading   logical_heading_;   // Logical heading, relative to starting in an E direction.
  long              logical_x_;         // Estimated x position.
  long              logical_y_;         // Estimated y position;
  bool              loop_detected_;
  long              odometer_;          // At node creation.
  Map*              parent_;            // Where this node came from.
  Map*              right_;             // Where a right turn leads.
  TTurn             turn_;              // Turn made from parent.
};

const char* Map::kHeadingName[DO_NOT_USE_HEADING] = {"N", "E", "S", "W"};
const Map::TLogicalHeading Map::kLeftOf[DO_NOT_USE_HEADING] = {Map::W, Map::N, Map::E, Map::S};
const Map::TLogicalHeading Map::kRightOf[DO_NOT_USE_HEADING] = {Map::E, Map::S, Map::W, Map::N};

const char* Map::kTurnName[DO_NOT_USE_TURN] = {"L", "C", "R"};

#endif
