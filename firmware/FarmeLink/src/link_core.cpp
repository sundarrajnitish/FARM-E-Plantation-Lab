// link_core.cpp - see link_core.h
#include "link_core.h"

namespace farme {

void Link::begin(uint32_t now) {
  *this = Link();
  last_hb_ = now;
}

void Link::pop() {
  for (uint8_t i = 1; i < n_; ++i) q_[i - 1] = q_[i];
  if (n_) --n_;
}

uint16_t Link::command(uint8_t code, int32_t a, int32_t b, uint32_t now) {
  last_code = code;
  if (code == CMD_STOP) n_ = 0;  // nothing queued before a STOP matters any more
  if (code == CMD_DRIVE) {
    // a newer drive command supersedes one that has not been sent yet
    for (uint8_t i = 0; i < n_; ++i) {
      if (q_[i].code == CMD_DRIVE && q_[i].tries == 0) {
        q_[i].a = a; q_[i].b = b; q_[i].queued = now;
        return q_[i].seq;
      }
    }
  }
  if (n_ >= QUEUE) return 0;
  seq_ = seq_next(seq_);
  Pending& p = q_[n_++];
  p.seq = seq_; p.code = code; p.a = a; p.b = b;
  p.queued = now; p.last_sent = 0; p.tries = 0;
  return p.seq;
}

void Link::on_frame(const Frame& fr, uint32_t now) {
  if (fr.type != 'T' || fr.n < 10) return;
  ++frames_in;
  last_rx_ = now;
  tel_.valid = true;
  tel_.ack = (uint16_t)fr.f[0];
  tel_.state = (uint8_t)fr.f[1];
  tel_.mode = (uint8_t)fr.f[2];
  tel_.flags = (uint16_t)fr.f[3];
  tel_.ground_mm = (int16_t)fr.f[4];
  tel_.hills = (uint16_t)fr.f[5];
  tel_.row = (uint8_t)fr.f[6];
  tel_.tank_mL = (int16_t)fr.f[7];
  tel_.batt_dV = (uint16_t)fr.f[8];
  tel_.fault = (uint8_t)fr.f[9];
  if (n_ && q_[0].tries && tel_.ack == q_[0].seq) {
    ++acked;
    last_ack_latency_ms = now - q_[0].queued;
    pop();
  }
}

bool Link::next_tx(Frame* out, uint32_t now) {
  if (n_) {
    Pending& p = q_[0];
    if (p.tries == 0 || now - p.last_sent >= RETRY_MS) {
      if (p.tries >= MAX_TRIES) {
        ++gave_up;
        pop();
        return next_tx(out, now);
      }
      if (p.tries) ++retries;
      ++p.tries;
      p.last_sent = now;
      ++sent;
      out->type = 'C';
      out->n = 4;
      out->f[0] = p.seq; out->f[1] = p.code; out->f[2] = p.a; out->f[3] = p.b;
      last_hb_ = now;
      return true;
    }
    return false;
  }
  if (now - last_hb_ >= HEARTBEAT_MS) {
    last_hb_ = now;
    out->type = 'H';
    out->n = 2;
    out->f[0] = env_ok_ ? temp_dC_ : -999;
    out->f[1] = env_ok_ ? hum_ : -1;
    return true;
  }
  return false;
}

// ------------------------------------------------------------------ JSON
size_t json_put(char* out, size_t pos, size_t cap, const char* s) {
  if (!pos && cap) out[0] = 0;
  while (*s) {
    if (pos + 1 >= cap) return pos;
    out[pos++] = *s++;
  }
  out[pos] = 0;
  return pos;
}

size_t json_int(char* out, size_t pos, size_t cap, int32_t v) {
  char tmp[12];
  uint8_t k = 0;
  uint32_t u = v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
  do { tmp[k++] = (char)('0' + u % 10); u /= 10; } while (u);
  if (v < 0) tmp[k++] = '-';
  while (k) {
    if (pos + 1 >= cap) break;
    out[pos++] = tmp[--k];
  }
  out[pos] = 0;
  return pos;
}

static const char* const STATE_KEYS[] = {
    "IDLE", "MANUAL", "CALIBRATE", "DRIVE", "SETTLE", "SEED", "WATER",
    "BACKOFF", "TURN1", "SHIFT", "TURN2", "DONE", "PAUSED", "FAULT"};
static const char* const MODE_KEYS[] = {"IDLE", "MANUAL", "AUTO", "FAULT"};

size_t Link::status_json(char* out, size_t cap, uint32_t now) const {
  size_t p = 0;
  bool ok = link_ok(now);
  p = json_put(out, p, cap, "{\"link\":");
  p = json_put(out, p, cap, ok ? "true" : "false");
  p = json_put(out, p, cap, ",\"age_ms\":");
  p = json_int(out, p, cap, tel_.valid ? (int32_t)(now - last_rx_) : -1);
  p = json_put(out, p, cap, ",\"mode\":\"");
  p = json_put(out, p, cap, tel_.mode < 4 ? MODE_KEYS[tel_.mode] : "?");
  p = json_put(out, p, cap, "\",\"state\":\"");
  p = json_put(out, p, cap, tel_.state < 14 ? STATE_KEYS[tel_.state] : "?");
  p = json_put(out, p, cap, "\",\"flags\":");
  p = json_int(out, p, cap, tel_.flags);
  p = json_put(out, p, cap, ",\"fault\":");
  p = json_int(out, p, cap, tel_.fault);
  p = json_put(out, p, cap, ",\"ground_mm\":");
  p = json_int(out, p, cap, tel_.ground_mm);
  p = json_put(out, p, cap, ",\"hills\":");
  p = json_int(out, p, cap, tel_.hills);
  p = json_put(out, p, cap, ",\"row\":");
  p = json_int(out, p, cap, tel_.row);
  p = json_put(out, p, cap, ",\"tank_mL\":");
  p = json_int(out, p, cap, tel_.tank_mL);
  p = json_put(out, p, cap, ",\"batt_dV\":");
  p = json_int(out, p, cap, tel_.batt_dV);
  p = json_put(out, p, cap, ",\"temp_C\":");
  p = json_int(out, p, cap, env_ok_ ? temp_dC_ / 10 : -999);
  p = json_put(out, p, cap, ",\"hum\":");
  p = json_int(out, p, cap, env_ok_ ? hum_ : -1);
  p = json_put(out, p, cap, ",\"queue\":");
  p = json_int(out, p, cap, n_);
  p = json_put(out, p, cap, ",\"retries\":");
  p = json_int(out, p, cap, (int32_t)retries);
  p = json_put(out, p, cap, ",\"lost\":");
  p = json_int(out, p, cap, (int32_t)gave_up);
  p = json_put(out, p, cap, "}");
  return p;
}

}  // namespace farme
