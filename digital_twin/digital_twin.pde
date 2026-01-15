// IMU Digital Twin + Live Plots (X/Y/Z angles) + Gesture message
// CSV (12 fields):
// t_ms,ax,ay,az,gx,gy,gz,angleX_deg,angleY_deg,angleY_smooth,gestureX,posSteps

import processing.serial.*;

Serial port;

final int PORT_INDEX = 2;
final int BAUD = 115200;

// Parsed from CSV
float angleX_deg = 0;         // roll (deg)
float angleY_deg = 0;         // pitch raw (deg) - optional
float angleY_smooth = 0;      // pitch filtered (deg)
float gz_dps = 0;             // gyro Z (deg/s)
int gestureX = 0;
int posSteps = 0;

// Yaw integration (relative -> drift)
float yaw_deg = 0;
int lastTms = -1;

// Gesture message timer
long lastGestureMsgMs = -99999;
final int GESTURE_MSG_MS = 1000;

// Stats
int anyLines = 0, okLines = 0, badLines = 0;
long lastAnyRxMs = 0, lastCsvRxMs = 0;
boolean gotHeader = false;
long lastHandshakeMs = 0;

boolean invertRoll = false;
boolean invertPitch = true;
boolean invertYaw = false;

// Visual smoothing (3D only)
float visRoll = 0, visPitch = 0, visYaw = 0;
final float VIS_ALPHA = 0.25;

// Camera
float camYaw = 0;
float camPitch = 0.35;
float camDist = 520;

// ===================== Live plot buffers =====================
final int N = 420;               // samples shown (~8.4s @ 50Hz)
float[] rollBuf  = new float[N]; // X
float[] pitchBuf = new float[N]; // Y (smooth)
float[] yawBuf   = new float[N]; // Z (integrated)
int bufIdx = 0;
boolean bufFilled = false;

// Plot ranges (degrees)
final float ROLL_MIN = -120, ROLL_MAX = 120;
final float PITCH_MIN = -120, PITCH_MAX = 120;
final float YAW_MIN = -180, YAW_MAX = 180;

void setup() {
  size(1200, 700, P3D);
  pixelDensity(1); // avoid HiDPI surprises
  frameRate(60);
  surface.setTitle("IMU Digital Twin + Live Plots (Serial CSV)");

  println("Serial ports:");
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) println(i + ": " + ports[i]);

  connect();
}

void draw() {
  background(18);

  boolean aliveAny = (millis() - lastAnyRxMs) < 2000;
  boolean aliveCsv = (millis() - lastCsvRxMs) < 2000;

  // handshake until header
  if (port != null && !gotHeader) {
    if (millis() - lastHandshakeMs > 500) {
      port.write('S');
      lastHandshakeMs = millis();
    }
  }

  // Convert to radians for 3D
  float roll  = radians(angleX_deg) * (invertRoll ? -1 : 1);
  float pitch = radians(angleY_smooth) * (invertPitch ? -1 : 1);
  float yaw   = radians(yaw_deg) * (invertYaw ? -1 : 1);

  visRoll  = lerp(visRoll, roll, VIS_ALPHA);
  visPitch = lerp(visPitch, pitch, VIS_ALPHA);
  visYaw   = lerp(visYaw, yaw, VIS_ALPHA);

  // ----------------- 3D Scene (left side) -----------------
  lights();
  pushMatrix();
  translate(width * 0.40, height * 0.52, 0);
  rotateY(camYaw);
  rotateX(camPitch);
  translate(0, 0, -camDist);

  drawAxes(180);

  pushMatrix();
  // yaw around Y (scene-friendly)
  rotateY(visYaw);

  // pitch (in your setup it looked like Z)
  rotateZ(visPitch);

  // roll around X
  rotateX(visRoll);

  drawIMUBoard();
  popMatrix();

  popMatrix();

  // ----------------- 2D Overlay: HUD + Plots (right side) -----------------
  hint(DISABLE_DEPTH_TEST);
  drawHUD(aliveAny, aliveCsv);
  drawPlotsPanel();
  hint(ENABLE_DEPTH_TEST);
}

void mouseDragged() {
  camYaw += (mouseX - pmouseX) * 0.01;
  camPitch += (mouseY - pmouseY) * 0.01;
  camPitch = constrain(camPitch, -1.2, 1.2);
}

void keyPressed() {
  if (key == 'c' || key == 'C') connect();
  if (key == 'i' || key == 'I') invertPitch = !invertPitch;
  if (key == 'o' || key == 'O') invertRoll = !invertRoll;
  if (key == 'u' || key == 'U') invertYaw = !invertYaw;
  if (key == 'r' || key == 'R') { yaw_deg = 0; lastTms = -1; }
}

void connect() {
  try {
    if (port != null) port.stop();

    String[] ports = Serial.list();
    println("Connecting to: " + ports[PORT_INDEX]);
    port = new Serial(this, ports[PORT_INDEX], BAUD);

    delay(2000);
    port.clear();
    port.bufferUntil('\n');

    anyLines = okLines = badLines = 0;
    lastAnyRxMs = lastCsvRxMs = 0;
    gotHeader = false;
    lastHandshakeMs = 0;

    yaw_deg = 0;
    lastTms = -1;

    // reset plot buffers
    for (int i = 0; i < N; i++) {
      rollBuf[i] = 0;
      pitchBuf[i] = 0;
      yawBuf[i] = 0;
    }
    bufIdx = 0;
    bufFilled = false;

  } catch (Exception e) {
    println("Serial connect error: " + e.getMessage());
  }
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;
  line = trim(line);
  if (line.length() == 0) return;

  anyLines++;
  lastAnyRxMs = millis();

  // Debug first lines
  if (anyLines <= 10) println("RAW: " + line);

  if (line.startsWith("t_ms")) { gotHeader = true; return; }
  if (line.startsWith("Commands:")) return;
  if (line.startsWith("[")) return;

  String[] tok = split(line, ',');

  // Expect 12 fields
  if (tok.length < 12) { badLines++; return; }
  if (!isNumeric(tok[0])) { badLines++; return; }

  try {
    int tms = int(float(tok[0]));
    gz_dps = float(tok[6]);

    // integrate yaw (relative)
    if (lastTms >= 0) {
      float dt = (tms - lastTms) * 1e-3;
      yaw_deg += gz_dps * dt;

      // wrap
      if (yaw_deg > 180) yaw_deg -= 360;
      if (yaw_deg < -180) yaw_deg += 360;
    }
    lastTms = tms;

    angleX_deg     = float(tok[7]);
    angleY_deg     = float(tok[8]);
    angleY_smooth  = float(tok[9]);
    gestureX       = int(float(tok[10]));
    posSteps       = int(float(tok[11]));

    // gesture message
    if (gestureX == 1) lastGestureMsgMs = millis();

    // push into plot buffers (angles X/Y/Z)
    rollBuf[bufIdx]  = angleX_deg;
    pitchBuf[bufIdx] = angleY_smooth;
    yawBuf[bufIdx]   = yaw_deg;

    bufIdx++;
    if (bufIdx >= N) { bufIdx = 0; bufFilled = true; }

    okLines++;
    lastCsvRxMs = millis();

  } catch (Exception e) {
    badLines++;
  }
}

boolean isNumeric(String s) {
  if (s == null || s.length() == 0) return false;
  for (int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (!(c == '-' || c == '.' || (c >= '0' && c <= '9'))) return false;
  }
  return true;
}

// ===================== Drawing helpers =====================

void drawAxes(float L) {
  strokeWeight(3);
  stroke(255, 80, 80);  line(0, 0, 0, L, 0, 0);  // X
  stroke(80, 255, 120); line(0, 0, 0, 0, L, 0);  // Y
  stroke(80, 140, 255); line(0, 0, 0, 0, 0, L);  // Z
  strokeWeight(1);
}

void drawIMUBoard() {
  noStroke();
  fill(240, 240, 245);
  box(220, 18, 140);

  // a small marker to see orientation better
  pushMatrix();
  translate(80, -14, 40);
  fill(60);
  box(40, 10, 40);
  popMatrix();
}

void drawHUD(boolean aliveAny, boolean aliveCsv) {
  // top-left HUD
  noStroke();
  fill(0, 160);
  rect(12, 12, 540, 210, 12);

  fill(255);
  textSize(14);
  text("IMU Digital Twin (Serial CSV) — No motor Y control", 24, 36);

  textSize(12);
  text("RX ANY: " + (aliveAny ? "OK" : "NO") +
       "   CSV: " + (aliveCsv ? "OK" : "NO") +
       "   header: " + (gotHeader ? "YES" : "NO"), 24, 60);

  text("anyLines=" + anyLines + "  okLines=" + okLines + "  badLines=" + badLines, 24, 78);

  text(String.format("Roll X: %.2f°   Pitch Y(smooth): %.2f°   Yaw Z(from gz): %.2f°",
    angleX_deg, angleY_smooth, yaw_deg), 24, 104);

  text("gestureX=" + gestureX + "   posSteps=" + posSteps, 24, 126);

  fill(220);
  text("Keys: C reconnect | I invert pitch | O invert roll | U invert yaw | R reset yaw | Mouse drag camera",
       24, 170);

  // Gesture message (English)
  if (millis() - lastGestureMsgMs < GESTURE_MSG_MS) {
    fill(255, 80, 80);
    textSize(18);
    text("GESTURE ACTIVATED", 24, 205);
  }
}

void drawPlotsPanel() {
  // Right-side panel
  float panelX = width * 0.70;
  float panelY = 12;
  float panelW = width * 0.28;
  float panelH = height - 24;

  noStroke();
  fill(0, 160);
  rect(panelX, panelY, panelW, panelH, 12);

  fill(255);
  textSize(14);
  text("Live Plots (Angles)", panelX + 16, panelY + 28);

  // plot areas
  float gx = panelX + 16;
  float gw = panelW - 32;
  float gh = (panelH - 70) / 3.0;
  float g1y = panelY + 45;
  float g2y = g1y + gh + 15;
  float g3y = g2y + gh + 15;

  drawPlot(gx, g1y, gw, gh, rollBuf,  ROLL_MIN,  ROLL_MAX,  "Roll (X) [deg]",  color(255, 80, 80));
  drawPlot(gx, g2y, gw, gh, pitchBuf, PITCH_MIN, PITCH_MAX, "Pitch (Y) [deg] (smooth)", color(80, 255, 120));
  drawPlot(gx, g3y, gw, gh, yawBuf,   YAW_MIN,   YAW_MAX,   "Yaw (Z) [deg] (integrated, drift)", color(80, 140, 255));
}

void drawPlot(float x, float y, float w, float h, float[] buf, float vmin, float vmax, String label, int lineColor) {
  // frame
  noFill();
  stroke(200, 120);
  rect(x, y, w, h, 8);

  // label
  fill(230);
  noStroke();
  textSize(12);
  text(label, x + 10, y + 16);

  // midline + bounds
  stroke(255, 40);
  float yMid = map(0, vmin, vmax, y + h - 10, y + 26);
  line(x + 8, yMid, x + w - 8, yMid);

  // plot
  int count = bufFilled ? N : bufIdx;
  if (count < 2) return;

  stroke(lineColor);
  noFill();

  beginShape();
  for (int i = 0; i < count; i++) {
    // oldest -> newest
    int idx = bufFilled ? (bufIdx + i) % N : i;

    float v = buf[idx];
    v = constrain(v, vmin, vmax);

    float px = map(i, 0, count - 1, x + 8, x + w - 8);
    float py = map(v, vmin, vmax, y + h - 10, y + 26);
    vertex(px, py);
  }
  endShape();

  // latest numeric
  float latest = buf[(bufIdx - 1 + N) % N];
  fill(230);
  noStroke();
  textSize(12);
  text(nf(latest, 0, 2), x + w - 70, y + 16);
}
