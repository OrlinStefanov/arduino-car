// Host test for car_protocol.h (not part of the sketch: the Arduino IDE ignores this folder).
//   g++ -std=c++11 -I.. car_protocol_test.cpp -o car_protocol_test && ./car_protocol_test
#include <stdio.h>
#include "car_protocol.h"

static int failures = 0;

#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

static bool parse(const char* line, CarCmd* c) { return carParse(line, c); }

static void drive_cases() {
  CarCmd c;
  CHECK(parse("F 100", &c) && c.type == CMD_DRIVE && c.arg == DIR_F && c.value == 100 && c.ms == 0);
  CHECK(parse("b 70 1500", &c) && c.type == CMD_DRIVE && c.arg == DIR_B && c.value == 70 && c.ms == 1500);
  CHECK(parse("L 0", &c) && c.type == CMD_DRIVE && c.arg == DIR_L && c.value == 0);
  CHECK(parse("  r   255  5000  ", &c) && c.arg == DIR_R && c.value == 255 && c.ms == 5000);
  CHECK(parse("F 999", &c) && c.value == 255);              // clamped
  CHECK(parse("F 100 9999", &c) && c.ms == 5000);           // clamped
  CHECK(!parse("F", &c) && c.type == CMD_BAD);              // speed missing
  CHECK(!parse("F abc", &c) && c.type == CMD_BAD);
  CHECK(!parse("F -5", &c) && c.type == CMD_BAD);
  CHECK(!parse("F 10 20 30", &c) && c.type == CMD_BAD);     // too many numbers
  CHECK(!parse("F 12345", &c) && c.type == CMD_BAD);        // more than 4 digits
  CHECK(!parse("F 10x", &c) && c.type == CMD_BAD);
  CHECK(!parse("FORWARD 10", &c) && c.type == CMD_BAD);
}

static void other_cases() {
  CarCmd c;
  CHECK(parse("S", &c) && c.type == CMD_STOP);
  CHECK(parse("stop", &c) && c.type == CMD_STOP);
  CHECK(!parse("S now", &c) && c.type == CMD_BAD);
  CHECK(parse("?", &c) && c.type == CMD_STATUS);
  CHECK(!parse("?x", &c) && c.type == CMD_BAD);
  CHECK(parse("CLAW 90", &c) && c.type == CMD_SERVO && c.arg == PART_CLAW && c.value == 90);
  CHECK(parse("arm 135", &c) && c.arg == PART_ARM && c.value == 135);
  CHECK(parse("BASE 400", &c) && c.arg == PART_BASE && c.value == 180);   // clamped to a servo's range
  CHECK(!parse("CLAW", &c) && c.type == CMD_BAD);
  CHECK(!parse("CLAW 90 5", &c) && c.type == CMD_BAD);
  CHECK(!parse("BASEX 5", &c) && c.type == CMD_BAD);
  CHECK(parse("MODE MANUAL", &c) && c.type == CMD_MODE && c.arg == MODE_MANUAL);
  CHECK(parse("mode avoid", &c) && c.arg == MODE_AVOID);
  CHECK(parse("MODE   Follow  ", &c) && c.arg == MODE_FOLLOW);
  CHECK(parse("MODE LINE", &c) && c.arg == MODE_LINE);
  CHECK(!parse("MODE", &c) && c.type == CMD_BAD);
  CHECK(!parse("MODE FLY", &c) && c.type == CMD_BAD);
  CHECK(!parse("MODE LINE 1", &c) && c.type == CMD_BAD);
  CHECK(!parse("", &c) && c.type == CMD_NONE);
  CHECK(!parse("   ", &c) && c.type == CMD_NONE);
  CHECK(!parse("hello", &c) && c.type == CMD_BAD);
}

static int feed(CarLine* l, const char* text, CarCmd* out, int* lines) {
  int parsed = 0;
  *lines = 0;
  for (const char* p = text; *p; p++) {
    if (carLineFeed(l, *p)) {
      (*lines)++;
      if (carParse(l->buf, out)) parsed++;
      carLineInit(l);
    }
  }
  return parsed;
}

static void line_cases() {
  CarLine l;
  CarCmd c;
  int lines;
  carLineInit(&l);
  CHECK(feed(&l, "F 100\r\nS\n", &c, &lines) == 2 && lines == 2 && c.type == CMD_STOP);   // two lines, CRLF ok
  CHECK(feed(&l, "F 1", &c, &lines) == 0 && lines == 0);                                  // half a line waits
  CHECK(feed(&l, "00\n", &c, &lines) == 1 && c.type == CMD_DRIVE && c.value == 100);      // ...and completes
  // a line longer than the buffer is thrown away whole, and the next one is fine
  char longline[80];
  memset(longline, 'A', sizeof(longline));
  longline[70] = '\n';
  longline[71] = 0;
  CHECK(feed(&l, longline, &c, &lines) == 0 && lines == 1);
  CHECK(feed(&l, "S\n", &c, &lines) == 1 && c.type == CMD_STOP);
  CHECK(feed(&l, "\n\n", &c, &lines) == 0 && lines == 2);                                  // blank lines are nothing
}

int main() {
  drive_cases();
  other_cases();
  line_cases();
  if (failures) { printf("%d check(s) failed\n", failures); return 1; }
  printf("all checks passed\n");
  return 0;
}
