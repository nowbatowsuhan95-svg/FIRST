// ESP32 attendance hub.
// Receives "UID:<hex>" lines from an Arduino Uno + RFID reader over
// Serial2, matches the UID against /students.json (uploaded to SPIFFS),
// appends one attendance row per student per day to /attendance.csv,
// and serves a small dashboard for today's and the overall report.
//
// Wiring: Arduino Uno D3 (through a 5V->3.3V voltage divider) -> ESP32 GPIO16 (RX2)
//         Arduino Uno GND -> ESP32 GND
//
// Requires: ArduinoJson library, version 6.x (the v7 JsonDocument API differs).

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <time.h>
#include <vector>
#include <map>
#include <set>

const char *WIFI_SSID = "YOUR_WIFI_SSID";
const char *WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const long GMT_OFFSET_SEC = 5 * 3600; // Turkmenistan is UTC+5
const int DAYLIGHT_OFFSET_SEC = 0;

const char *STUDENTS_FILE = "/students.json";
const char *ATTENDANCE_FILE = "/attendance.csv";

struct Student {
  String uid;
  String id;
  String name;
};

std::vector<Student> students;
std::map<String, String> lastAttendanceDate; // uid -> last date (YYYY-MM-DD) logged
std::set<String> unknownUidsSeen;

WebServer server(80);

// ---------- students.json ----------

void loadStudents() {
  students.clear();
  File f = SPIFFS.open(STUDENTS_FILE, FILE_READ);
  if (!f) {
    Serial.println(F("No students.json found, starting with an empty roster."));
    return;
  }
  DynamicJsonDocument doc(8192);
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) {
    Serial.print(F("Failed to parse students.json: "));
    Serial.println(err.c_str());
    return;
  }
  for (JsonObject obj : doc.as<JsonArray>()) {
    Student s;
    s.uid = String(obj["uid"].as<const char *>());
    s.uid.toUpperCase();
    s.id = String(obj["id"].as<const char *>());
    s.name = String(obj["name"].as<const char *>());
    students.push_back(s);
  }
  Serial.printf("Loaded %d students.\n", students.size());
}

void saveStudents() {
  DynamicJsonDocument doc(8192);
  JsonArray arr = doc.to<JsonArray>();
  for (auto &s : students) {
    JsonObject obj = arr.createNestedObject();
    obj["uid"] = s.uid;
    obj["id"] = s.id;
    obj["name"] = s.name;
  }
  File f = SPIFFS.open(STUDENTS_FILE, FILE_WRITE);
  if (!f) {
    Serial.println(F("Failed to open students.json for writing."));
    return;
  }
  serializeJson(doc, f);
  f.close();
}

Student *findStudentByUid(const String &uid) {
  for (auto &s : students) {
    if (s.uid == uid) return &s;
  }
  return nullptr;
}

// ---------- attendance.csv ----------

String csvEscape(const String &field) {
  String out = field;
  out.replace(",", " ");
  out.replace("\n", " ");
  return out;
}

void ensureAttendanceFile() {
  if (!SPIFFS.exists(ATTENDANCE_FILE)) {
    File f = SPIFFS.open(ATTENDANCE_FILE, FILE_WRITE);
    if (f) {
      f.println(F("date,time,uid,student_id,student_name,status"));
      f.close();
    }
  }
}

// Populate lastAttendanceDate from the existing log so a reboot doesn't
// let the same tap be logged twice in one day.
void loadLastAttendanceDates() {
  File f = SPIFFS.open(ATTENDANCE_FILE, FILE_READ);
  if (!f) return;
  f.readStringUntil('\n'); // header
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    int c3 = line.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) continue;
    String date = line.substring(0, c1);
    String uid = line.substring(c2 + 1, c3);
    lastAttendanceDate[uid] = date;
  }
  f.close();
}

bool getNow(struct tm &timeinfo) {
  return getLocalTime(&timeinfo);
}

String formatDate(const struct tm &timeinfo) {
  char buf[11];
  strftime(buf, sizeof(buf), "%Y-%m-%d", &timeinfo);
  return String(buf);
}

String formatTime(const struct tm &timeinfo) {
  char buf[9];
  strftime(buf, sizeof(buf), "%H:%M:%S", &timeinfo);
  return String(buf);
}

void logAttendance(const String &uid) {
  struct tm timeinfo;
  if (!getNow(timeinfo)) {
    Serial.println(F("Time not synced yet, dropping scan."));
    return;
  }
  String date = formatDate(timeinfo);
  String time = formatTime(timeinfo);

  if (lastAttendanceDate.count(uid) && lastAttendanceDate[uid] == date) {
    Serial.println(F("Already logged today, ignoring repeat tap."));
    return;
  }

  Student *s = findStudentByUid(uid);
  String studentId = s ? s->id : "UNKNOWN";
  String studentName = s ? s->name : "UNKNOWN";
  String status = s ? "present" : "unknown_card";

  if (!s) unknownUidsSeen.insert(uid);

  File f = SPIFFS.open(ATTENDANCE_FILE, FILE_APPEND);
  if (!f) {
    Serial.println(F("Failed to open attendance.csv for appending."));
    return;
  }
  f.printf("%s,%s,%s,%s,%s,%s\n", date.c_str(), time.c_str(), uid.c_str(),
            csvEscape(studentId).c_str(), csvEscape(studentName).c_str(), status.c_str());
  f.close();

  lastAttendanceDate[uid] = date;

  Serial.printf("Logged: %s %s uid=%s -> %s (%s)\n", date.c_str(), time.c_str(),
                uid.c_str(), studentName.c_str(), status.c_str());
}

// ---------- report building ----------

struct AttendanceRow {
  String date, time, uid, studentId, studentName, status;
};

std::vector<AttendanceRow> readAllAttendance() {
  std::vector<AttendanceRow> rows;
  File f = SPIFFS.open(ATTENDANCE_FILE, FILE_READ);
  if (!f) return rows;
  f.readStringUntil('\n'); // header
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    AttendanceRow row;
    String *fields[6] = {&row.date, &row.time, &row.uid, &row.studentId, &row.studentName, &row.status};
    int start = 0;
    for (int i = 0; i < 6; i++) {
      int idx = (i == 5) ? line.length() : line.indexOf(',', start);
      if (idx < 0) idx = line.length();
      *fields[i] = line.substring(start, idx);
      start = idx + 1;
    }
    rows.push_back(row);
  }
  f.close();
  return rows;
}

// ---------- web pages ----------

String htmlPage(const String &title, const String &body) {
  String page = "<!DOCTYPE html><html><head><meta charset='utf-8'>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<title>" + title + "</title>";
  page += "<style>body{font-family:sans-serif;margin:2em;}table{border-collapse:collapse;}"
          "td,th{border:1px solid #ccc;padding:6px 10px;text-align:left;}"
          "a{color:#2563eb;}</style></head><body>";
  page += "<h1>" + title + "</h1>" + body;
  page += "<p><a href='/'>&larr; Dashboard</a></p></body></html>";
  return page;
}

void handleRoot() {
  struct tm timeinfo;
  String todayStr = getNow(timeinfo) ? formatDate(timeinfo) : "(wagt entek sinhronizlenmedi)";

  std::vector<AttendanceRow> rows = readAllAttendance();
  std::set<String> distinctDates;
  for (auto &r : rows) distinctDates.insert(r.date);

  String body = "<p>Sana: <b>" + todayStr + "</b></p>";
  body += "<p>Bellenen talyplar: <b>" + String(students.size()) + "</b></p>";
  body += "<p>Ýazgy alnan sapak günleri: <b>" + String(distinctDates.size()) + "</b></p>";
  if (!unknownUidsSeen.empty()) {
    body += "<p style='color:#b91c1c'>Tanalmadyk kartlar: ";
    for (const String &uid : unknownUidsSeen) {
      body += "<a href='/register?uid=" + uid + "'>" + uid + "</a> ";
    }
    body += "</p>";
  }
  body += "<ul>"
          "<li><a href='/today'>Şu günki gatnaşyk</a></li>"
          "<li><a href='/report'>Doly hasabat (gün sanlary)</a></li>"
          "<li><a href='/students'>Talyplaryň sanawy</a></li>"
          "<li><a href='/register'>Täze kart/talyp goş</a></li>"
          "<li><a href='/raw'>attendance.csv göçürip al</a></li>"
          "</ul>";
  server.send(200, "text/html", htmlPage("Dowamat ulgamy", body));
}

void handleToday() {
  struct tm timeinfo;
  if (!getNow(timeinfo)) {
    server.send(200, "text/html", htmlPage("Şu gün", "<p>Wagt entek sinhronizlenmedi.</p>"));
    return;
  }
  String today = formatDate(timeinfo);
  std::vector<AttendanceRow> rows = readAllAttendance();
  std::set<String> presentUids;
  for (auto &r : rows) {
    if (r.date == today && r.status == "present") presentUids.insert(r.uid);
  }

  String body = "<h2>Geldi (" + String(presentUids.size()) + ")</h2><table><tr><th>ID</th><th>Ady</th><th>Wagt</th></tr>";
  for (auto &r : rows) {
    if (r.date == today && r.status == "present") {
      body += "<tr><td>" + r.studentId + "</td><td>" + r.studentName + "</td><td>" + r.time + "</td></tr>";
    }
  }
  body += "</table><h2>Gelmedi</h2><table><tr><th>ID</th><th>Ady</th></tr>";
  for (auto &s : students) {
    if (!presentUids.count(s.uid)) {
      body += "<tr><td>" + s.id + "</td><td>" + s.name + "</td></tr>";
    }
  }
  body += "</table>";
  server.send(200, "text/html", htmlPage("Şu günki gatnaşyk (" + today + ")", body));
}

void handleReport() {
  std::vector<AttendanceRow> rows = readAllAttendance();
  std::set<String> distinctDates;
  std::map<String, int> presentCountByUid;
  for (auto &r : rows) {
    distinctDates.insert(r.date);
    if (r.status == "present") presentCountByUid[r.uid]++;
  }
  int totalDays = distinctDates.size();

  String body = "<p>Jemi ýazgy alnan sapak güni: <b>" + String(totalDays) + "</b></p>";
  body += "<table><tr><th>ID</th><th>Ady</th><th>Gatnady</th><th>Gatnamady</th><th>%</th></tr>";
  for (auto &s : students) {
    int present = presentCountByUid.count(s.uid) ? presentCountByUid[s.uid] : 0;
    int absent = totalDays - present;
    if (absent < 0) absent = 0;
    float pct = totalDays > 0 ? (100.0f * present / totalDays) : 0.0f;
    body += "<tr><td>" + s.id + "</td><td>" + s.name + "</td><td>" + String(present) +
            "</td><td>" + String(absent) + "</td><td>" + String(pct, 1) + "%</td></tr>";
  }
  body += "</table>";
  server.send(200, "text/html", htmlPage("Doly hasabat", body));
}

void handleStudents() {
  String body = "<table><tr><th>ID</th><th>Ady</th><th>UID</th></tr>";
  for (auto &s : students) {
    body += "<tr><td>" + s.id + "</td><td>" + s.name + "</td><td>" + s.uid + "</td></tr>";
  }
  body += "</table>";
  server.send(200, "text/html", htmlPage("Talyplaryň sanawy", body));
}

void handleRegister() {
  if (server.hasArg("uid") && server.hasArg("name") && server.hasArg("id")) {
    String uid = server.arg("uid");
    uid.toUpperCase();
    Student *existing = findStudentByUid(uid);
    if (existing) {
      existing->id = server.arg("id");
      existing->name = server.arg("name");
    } else {
      Student s{uid, server.arg("id"), server.arg("name")};
      students.push_back(s);
    }
    saveStudents();
    unknownUidsSeen.erase(uid);
    server.send(200, "text/html", htmlPage("Goşuldy", "<p><b>" + server.arg("name") + "</b> ugry goşuldy (UID " + uid + ").</p>"));
    return;
  }

  String prefillUid = server.hasArg("uid") ? server.arg("uid") : "";
  String body = "<form method='GET' action='/register'>"
                "<p>Kart UID: <input name='uid' value='" + prefillUid + "' required></p>"
                "<p>Talyp ID: <input name='id' required></p>"
                "<p>Ady familiýasy: <input name='name' required></p>"
                "<button type='submit'>Goş</button></form>";
  server.send(200, "text/html", htmlPage("Täze kart/talyp goş", body));
}

void handleRaw() {
  File f = SPIFFS.open(ATTENDANCE_FILE, FILE_READ);
  if (!f) {
    server.send(404, "text/plain", "attendance.csv not found");
    return;
  }
  server.streamFile(f, "text/csv");
  f.close();
}

// ---------- setup / loop ----------

void connectWifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(F("Connecting to WiFi"));
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  Serial.print(F("Connected, IP: "));
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX2=GPIO16 <- Arduino Uno TX (via divider)

  if (!SPIFFS.begin(true)) {
    Serial.println(F("SPIFFS mount failed."));
  }

  connectWifi();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");

  ensureAttendanceFile();
  loadStudents();
  loadLastAttendanceDates();

  server.on("/", handleRoot);
  server.on("/today", handleToday);
  server.on("/report", handleReport);
  server.on("/students", handleStudents);
  server.on("/register", handleRegister);
  server.on("/raw", handleRaw);
  server.begin();
  Serial.println(F("Web server started."));
}

void loop() {
  server.handleClient();

  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.startsWith("UID:")) {
      String uid = line.substring(4);
      uid.toUpperCase();
      logAttendance(uid);
    }
  }
}
