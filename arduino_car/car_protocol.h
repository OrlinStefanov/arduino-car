// The car's Bluetooth command language, kept free of Arduino types so it can be
// compiled and tested on a PC (see firmware/README.md). One command per line:
//
//   F|B|L|R <speed 0-255> [ms]     drive forward / back / spin left / spin right
//   S                              stop (also leaves a self-driving mode)
//   CLAW|ARM|BASE <degrees>        move a servo
//   MODE MANUAL|AVOID|FOLLOW|LINE  who is driving
//   ?                              answer with the car's state
//
// Letters are case-insensitive, lines end with \n (a \r before it is ignored).
// A number must be plain digits. Speed and ms are clamped to their limits, so
// a slightly-off value still does something sensible; anything that is not a
// command is CMD_BAD and the car ignores it.
#ifndef CAR_PROTOCOL_H
#define CAR_PROTOCOL_H

#include <stdint.h>
#include <string.h>

#define CAR_LINE_MAX 24            // longest line kept; a longer one is thrown away whole
#define CAR_MAX_SPEED 255
#define CAR_MAX_MS 5000            // longest timed move

enum { CMD_NONE = 0, CMD_DRIVE, CMD_STOP, CMD_SERVO, CMD_MODE, CMD_STATUS, CMD_BAD };
enum { DIR_F = 0, DIR_B, DIR_L, DIR_R };
enum { PART_CLAW = 0, PART_ARM, PART_BASE };
enum { MODE_MANUAL = 0, MODE_AVOID, MODE_FOLLOW, MODE_LINE };

struct CarCmd {
  uint8_t type;                    // CMD_*
  uint8_t arg;                     // DIR_*, PART_* or MODE_*
  int16_t value;                   // speed, or degrees
  uint16_t ms;                     // timed move length, 0 = until the dead-man stop
};

struct CarLine {
  char buf[CAR_LINE_MAX + 1];
  uint8_t len;
  bool overflow;
};

static inline void carLineInit(CarLine* l) {
  l->len = 0;
  l->overflow = false;
  l->buf[0] = 0;
}

// Feed one received byte. True when a whole line is ready in l->buf (or the
// line was too long: buf is then empty and parses as CMD_NONE); call
// carLineInit after using it.
static inline bool carLineFeed(CarLine* l, char c) {
  if (c == '\r') return false;
  if (c == '\n') {
    if (l->overflow) { l->len = 0; l->overflow = false; }
    l->buf[l->len] = 0;
    return true;
  }
  if (l->overflow) return false;
  if (l->len >= CAR_LINE_MAX) { l->overflow = true; return false; }
  l->buf[l->len++] = c;
  return false;
}

static inline char carUpper(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; }

// A whole word at *p (up to a space or the end), compared with `word` ignoring case.
static inline bool carWord(const char** p, const char* word) {
  const char* s = *p;
  size_t n = strlen(word);
  for (size_t i = 0; i < n; i++) if (carUpper(s[i]) != word[i]) return false;
  if (s[n] != 0 && s[n] != ' ') return false;
  *p = s + n;
  return true;
}

static inline void carSkipSpaces(const char** p) { while (**p == ' ') (*p)++; }

// Plain digits, at most 4, then a space or the end. False (and *p untouched) otherwise.
static inline bool carNumber(const char** p, int32_t* out) {
  const char* s = *p;
  int32_t v = 0;
  uint8_t digits = 0;
  while (*s >= '0' && *s <= '9') {
    if (++digits > 4) return false;
    v = v * 10 + (*s - '0');
    s++;
  }
  if (digits == 0 || (*s != 0 && *s != ' ')) return false;
  *out = v;
  *p = s;
  return true;
}

static inline bool carAtEnd(const char* p) { carSkipSpaces(&p); return *p == 0; }

// Parse one line. True when it is a command (out->type is not CMD_BAD); an
// empty line gives CMD_NONE and false.
static inline bool carParse(const char* line, CarCmd* out) {
  out->type = CMD_NONE;
  out->arg = 0;
  out->value = 0;
  out->ms = 0;
  const char* p = line;
  carSkipSpaces(&p);
  if (*p == 0) return false;

  out->type = CMD_BAD;
  int32_t n = 0;

  if (*p == '?' && carAtEnd(p + 1)) { out->type = CMD_STATUS; return true; }

  if (carWord(&p, "S") || carWord(&p, "STOP")) {
    if (!carAtEnd(p)) return false;
    out->type = CMD_STOP;
    return true;
  }

  const char* dirs[] = {"F", "B", "L", "R"};
  for (uint8_t d = 0; d < 4; d++) {
    const char* q = p;
    if (!carWord(&q, dirs[d])) continue;
    carSkipSpaces(&q);
    if (!carNumber(&q, &n)) return false;
    out->value = (int16_t)(n > CAR_MAX_SPEED ? CAR_MAX_SPEED : n);
    carSkipSpaces(&q);
    if (*q) {
      if (!carNumber(&q, &n)) return false;
      out->ms = (uint16_t)(n > CAR_MAX_MS ? CAR_MAX_MS : n);
      if (!carAtEnd(q)) return false;
    }
    out->type = CMD_DRIVE;
    out->arg = d;
    return true;
  }

  const char* parts[] = {"CLAW", "ARM", "BASE"};
  for (uint8_t i = 0; i < 3; i++) {
    const char* q = p;
    if (!carWord(&q, parts[i])) continue;
    carSkipSpaces(&q);
    if (!carNumber(&q, &n) || !carAtEnd(q)) return false;
    out->type = CMD_SERVO;
    out->arg = i;
    out->value = (int16_t)(n > 180 ? 180 : n);
    return true;
  }

  const char* q = p;
  if (carWord(&q, "MODE")) {
    carSkipSpaces(&q);
    const char* modes[] = {"MANUAL", "AVOID", "FOLLOW", "LINE"};
    for (uint8_t m = 0; m < 4; m++) {
      const char* r = q;
      if (carWord(&r, modes[m]) && carAtEnd(r)) {
        out->type = CMD_MODE;
        out->arg = m;
        return true;
      }
    }
  }
  return false;
}

#endif  // CAR_PROTOCOL_H
