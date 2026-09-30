// ESP8266WebServer.h (simulation mock) - the request loop of the ESP8266
// core's web server. Requests come from the simulated phone (sim::Rig's HTTP
// queue); handleClient() serves at most one per call, as the real one does.
#pragma once
#include "Arduino.h"
#include "ESP8266WiFi.h"

enum HTTPMethod { HTTP_ANY, HTTP_GET, HTTP_HEAD, HTTP_POST, HTTP_PUT, HTTP_PATCH, HTTP_DELETE, HTTP_OPTIONS };

class ESP8266WebServer {
 public:
  typedef void (*THandlerFunction)();
  explicit ESP8266WebServer(int port = 80) { (void)port; }
  void begin();
  void close() {}
  void handleClient();
  void on(const char* uri, THandlerFunction fn) { on(uri, HTTP_ANY, fn); }
  void on(const char* uri, HTTPMethod m, THandlerFunction fn);
  void onNotFound(THandlerFunction fn) { not_found_ = fn; }
  String arg(const char* name);
  String arg(const String& name) { return arg(name.c_str()); }
  bool hasArg(const char* name);
  int args() const { return nargs_; }
  String uri() const { return String(path_); }
  HTTPMethod method() const { return HTTP_GET; }
  void sendHeader(const char* name, const char* value, bool first = false) { (void)name; (void)value; (void)first; }
  void send(int code, const char* ctype, const String& content) { send_raw(code, ctype, content.c_str(), content.length()); }
  void send(int code, const char* ctype, const char* content) { send_raw(code, ctype, content, strlen(content)); }
  void send(int code) { send_raw(code, "text/plain", "", 0); }
  void send_P(int code, PGM_P ctype, PGM_P content) { send_raw(code, ctype, content, strlen(content)); }
  void send_P(int code, PGM_P ctype, PGM_P content, size_t len) { send_raw(code, ctype, content, len); }

 private:
  void send_raw(int code, const char* ctype, const char* body, size_t len);
  struct Route { char uri[48]; HTTPMethod m; THandlerFunction fn; };
  Route routes_[16];
  int nroutes_ = 0;
  THandlerFunction not_found_ = nullptr;
  bool begun_ = false;
  int cur_ = -1;           // request being served (id)
  bool answered_ = false;
  char path_[96] = {};
  char names_[8][24] = {}, values_[8][64] = {};
  int nargs_ = 0;
};
