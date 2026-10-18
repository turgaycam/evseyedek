#include "cloud_core.h"

#include <ArduinoJson.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace evse_cloud {

uint32_t backoff_delay_ms(uint32_t consecutiveFailures) {
  uint32_t d = kPollIntervalMs;
  for (uint32_t i = 0; i < consecutiveFailures && d < kBackoffMaxMs; ++i) d *= 2;
  return d > kBackoffMaxMs ? kBackoffMaxMs : d;
}

bool is_safe_token(const char* s) {
  if (s == nullptr) return false;
  size_t n = 0;
  for (; s[n] != '\0'; ++n) {
    char c = s[n];
    bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (!ok || n >= 64) return false;
  }
  return n > 0;
}

bool valid_station_code(const char* s) {
  if (s == nullptr) return false;
  size_t n = strlen(s);
  if (n < 3 || n > 16) return false;
  for (size_t i = 0; i < n; ++i) {
    char c = s[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
  }
  return true;
}

bool valid_secret(const char* s) {
  if (s == nullptr || strlen(s) != kSecretLen) return false;
  return is_safe_token(s);
}

bool normalize_server_url(const char* in, char out[kUrlMax]) {
  if (in == nullptr || out == nullptr) return false;
  out[0] = '\0';
  if (strncmp(in, "https://", 8) != 0) return false;
  size_t n = strlen(in);
  while (n > 8 && in[n - 1] == '/') --n;  // sondaki '/'
  if (n <= 8 || n >= kUrlMax) return false;
  // host[:port][/yol]
  size_t i = 8;
  size_t hostLen = 0;
  for (; i < n && in[i] != ':' && in[i] != '/'; ++i, ++hostLen) {
    char c = in[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' || c == '-')) return false;
  }
  if (hostLen == 0 || in[8] == '.' || in[8] == '-') return false;
  if (i < n && in[i] == ':') {
    ++i;
    size_t digits = 0;
    unsigned long port = 0;
    for (; i < n && in[i] >= '0' && in[i] <= '9'; ++i, ++digits) port = port * 10 + (unsigned long)(in[i] - '0');
    if (digits == 0 || digits > 5 || port == 0 || port > 65535) return false;
  }
  for (; i < n; ++i) {
    char c = in[i];
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' || c == '-' ||
          c == '_' || c == '/' || c == '~')) return false;
  }
  memcpy(out, in, n);
  out[n] = '\0';
  return true;
}

char normalize_state(char c) {
  return (c >= 'A' && c <= 'F') ? c : 'F';
}

static float finiteOr0(float v) {
  return (isnan(v) || isinf(v)) ? 0.0f : v;
}

int build_poll_json(char* out, size_t outLen, const char* code, const char* secret,
                    const Telemetry& t, const char* fw, const Ack& ack) {
  if (out == nullptr || outLen == 0) return -1;
  out[0] = '\0';
  if (!is_safe_token(code) || !is_safe_token(secret)) return -1;

  // fw yalnizca surum karakterleri icerebilir; aksi halde "?" gonderilir.
  char fwSafe[24] = "?";
  if (fw != nullptr) {
    size_t i = 0;
    bool ok = fw[0] != '\0';
    for (; fw[i] != '\0' && i < sizeof(fwSafe) - 1; ++i) {
      char c = fw[i];
      if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '.' || c == '-' || c == '_' || c == '+')) {
        ok = false;
        break;
      }
      fwSafe[i] = c;
    }
    if (ok && fw[i] == '\0') fwSafe[i] = '\0';
    else { fwSafe[0] = '?'; fwSafe[1] = '\0'; }
  }

  int n = snprintf(out, outLen,
                   "{\"code\":\"%s\",\"secret\":\"%s\",\"state\":\"%c\","
                   "\"ia\":%.2f,\"ib\":%.2f,\"ic\":%.2f,\"pW\":%.1f,\"eKWh\":%.3f,"
                   "\"tSec\":%lu,\"fw\":\"%s\"",
                   code, secret, normalize_state(t.state),
                   finiteOr0(t.ia), finiteOr0(t.ib), finiteOr0(t.ic), finiteOr0(t.pW), finiteOr0(t.eKWh),
                   (unsigned long)t.tSec, fwSafe);
  if (n < 0 || (size_t)n >= outLen) { out[0] = '\0'; return -1; }
  // Planli sarj alanlari. Sunucu araligi 0..604800; bilinmeyen deger null gonderilir.
  char resume[16] = "null";
  char nextCh[16] = "null";
  if (t.paused && t.resumeInSec >= 0 && t.resumeInSec <= 604800) snprintf(resume, sizeof(resume), "%ld", (long)t.resumeInSec);
  if (t.schedNextChangeInSec >= 0 && t.schedNextChangeInSec <= 604800)
    snprintf(nextCh, sizeof(nextCh), "%ld", (long)t.schedNextChangeInSec);
  int k = snprintf(out + n, outLen - n, ",\"paused\":%s,\"pauseReason\":%s,\"resumeInSec\":%s",
                   t.paused ? "true" : "false", t.paused ? "\"schedule\"" : "null", resume);
  if (k < 0 || (size_t)(n + k) >= outLen) { out[0] = '\0'; return -1; }
  n += k;
  k = snprintf(out + n, outLen - n, ",\"sched\":{\"on\":%s,\"blocked\":%s,\"timeOk\":%s,\"nextChangeInSec\":%s}",
               t.schedOn ? "true" : "false", t.schedBlocked ? "true" : "false",
               t.schedTimeOk ? "true" : "false", nextCh);
  if (k < 0 || (size_t)(n + k) >= outLen) { out[0] = '\0'; return -1; }
  n += k;
  int m;
  if (ack.has) {
    m = snprintf(out + n, outLen - n, ",\"ackId\":%lld,\"ackOk\":%s}", (long long)ack.id, ack.ok ? "true" : "false");
  } else {
    m = snprintf(out + n, outLen - n, "}");
  }
  if (m < 0 || (size_t)(n + m) >= outLen) { out[0] = '\0'; return -1; }
  return n + m;
}

bool parse_poll_response(const char* body, size_t len, PollResponse& out) {
  out = PollResponse();
  if (body == nullptr || len == 0 || len > 1024) return false;
  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, body, len) != DeserializationError::Ok) return false;
  if (!doc.is<JsonObject>()) return false;
  JsonVariantConst okV = doc["ok"];
  if (!okV.is<bool>()) return false;
  out.ok = okV.as<bool>();
  out.t = doc["t"] | (int64_t)0;
  JsonVariantConst cmd = doc["cmd"];
  if (cmd.isNull()) return true;
  if (!cmd.is<JsonObjectConst>()) return false;
  JsonVariantConst idV = cmd["id"];
  if (!idV.is<int64_t>()) return false;
  int64_t id = idV.as<int64_t>();
  if (id <= 0) return false;
  const char* type = cmd["type"] | "";
  out.hasCmd = true;
  out.cmdId = id;
  if (strcmp(type, "start") == 0) out.type = CmdType::Start;
  else if (strcmp(type, "stop") == 0) out.type = CmdType::Stop;
  else if (strcmp(type, "set_limit") == 0) out.type = CmdType::SetLimit;
  else out.type = CmdType::Unknown;
  return true;
}

const char* cmd_type_name(CmdType t) {
  switch (t) {
    case CmdType::Start: return "start";
    case CmdType::Stop: return "stop";
    case CmdType::SetLimit: return "set_limit";
    case CmdType::Unknown: return "unknown";
    default: return "none";
  }
}

// ---- CommandTracker ----
void CommandTracker::restore(bool hasLast, int64_t lastId, bool lastOk) {
  hasLast_ = hasLast && lastId > 0;
  lastId_ = hasLast_ ? lastId : 0;
  lastOk_ = hasLast_ ? lastOk : false;
}

CommandTracker::Decision CommandTracker::on_command(int64_t id) {
  if (inFlight_) return (id == inFlightId_) ? Decision::InFlight : Decision::Ignore;
  if (hasLast_ && id == lastId_) {
    ack_.has = true;
    ack_.id = id;
    ack_.ok = lastOk_;
    return Decision::ReAck;
  }
  inFlight_ = true;
  inFlightId_ = id;
  return Decision::Apply;
}

void CommandTracker::on_applied(int64_t id, bool ok) {
  if (!inFlight_ || id != inFlightId_) return;
  inFlight_ = false;
  inFlightId_ = 0;
  hasLast_ = true;
  lastId_ = id;
  lastOk_ = ok;
  ack_.has = true;
  ack_.id = id;
  ack_.ok = ok;
}

void CommandTracker::on_ack_delivered(int64_t id) {
  if (ack_.has && ack_.id == id) ack_ = Ack();
}

// ---- SafeGate ----
static bool vehiclePresentOk(char s) {
  return s == 'B' || s == 'C' || s == 'D';
}

bool SafeGate::on_start(char state, bool requireCloudStart) {
  if (!vehiclePresentOk(state)) return false;
  if (requireCloudStart) authorized_ = true;
  return true;
}

bool SafeGate::on_stop() {
  authorized_ = false;
  return true;
}

bool SafeGate::update(bool requireCloudStart, char state, bool localStop) {
  if (!requireCloudStart) {
    // Kapali modda izin eski kalmasin: sonradan acilinca yeni "start" gereksin.
    authorized_ = false;
    return true;
  }
  // Yalnizca arac ayrilmasi (A) ve yerel STOP izni kaldirir; baglanti kopmasi kaldirmaz.
  if (state == 'A' || localStop) authorized_ = false;
  return authorized_;
}

}  // namespace evse_cloud
