import processing.serial.*;
import java.util.ArrayList;

// ====================== SETTINGS ======================
static final boolean SHOW_EULER = true;  // true = lower graph shows Euler (R,P,Y). false = lower shows Quaternion again
static final int MAX_POINTS = 180;
static final String PORT_NAME = "COM80"; // change to your port
static final int BAUD = 115200;
// =====================================================

// Quaternion container (q0, q1, q2, q3) where q0 is w (or keep consistent with sender)
class Quat {
  float q0, q1, q2, q3;
  float timestamp;
  float temp;
  Quat(float timestamp, float temp, float q0, float q1, float q2, float q3) {
    this.timestamp = timestamp;
    this.temp = temp;
    this.q0 = q0;
    this.q1 = q1;
    this.q2 = q2;
    this.q3 = q3;
  }
}

// Globals
Serial serial;
ArrayList<Quat> quatLog = new ArrayList<>();
ArrayList<PVector> eulerLog = new ArrayList<>(); // x=roll, y=pitch, z=yaw (degrees)

void setup() {
  size(1200, 600);
  println("Opening port: " + PORT_NAME);
  serial = new Serial(this, PORT_NAME, BAUD);
  serial.bufferUntil('\n');
  textFont(createFont("Courier",12));
}

// Parse incoming CSV line: timestamp,temp,q0,q1,q2,q3
Quat parseLine(String line) {
  String[] v = split(trim(line), ',');
  if (v.length < 6) {
    // invalid line
    println("Invalid data (expected 6 fields):", line);
    return null;
  }
  // parse safely
  float ts = 0;
  float temp = 0;
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  try {
    ts   = float(v[0]);
    temp = float(v[1]);
    q0   = float(v[2]);
    q1   = float(v[3]);
    q2   = float(v[4]);
    q3   = float(v[5]);
  } catch (Exception e) {
    println("Parse error:", e);
    return null;
  }
  return new Quat(ts, temp, q0, q1, q2, q3);
}

// Convert quaternion (q0,q1,q2,q3) -> Euler (roll,pitch,yaw) in degrees
PVector quatToEuler(float q0, float q1, float q2, float q3) {
  // Assumes q0 = w, q1 = x, q2 = y, q3 = z
  float sinr = 2.0 * (q0 * q1 + q2 * q3);
  float cosr = 1.0 - 2.0 * (q1*q1 + q2*q2);
  float roll = atan2(sinr, cosr);

  float sinp = 2.0 * (q0 * q2 - q3 * q1);
  float pitch = abs(sinp) >= 1 ? copysign(HALF_PI, sinp) : asin(sinp);

  float siny = 2.0 * (q0 * q3 + q1 * q2);
  float cosy = 1.0 - 2.0 * (q2*q2 + q3*q3);
  float yaw = atan2(siny, cosy);

  return new PVector(degrees(roll), degrees(pitch), degrees(yaw));
}

float copysign(float a, float b) {
  return (b >= 0) ? abs(a) : -abs(a);
}

void draw() {
  background(240);

  // Read serial (non-blocking)
  while (serial.available() > 0) {
    String line = serial.readStringUntil('\n');
    if (line == null) break;
    Quat q = parseLine(line);
    if (q == null) continue;

    // store quaternion and computed euler
    quatLog.add(q);
    eulerLog.add(quatToEuler(q.q0, q.q1, q.q2, q.q3));

    // print to console (timestamp, temp, q0..q3)
    println(nf(q.timestamp,0,2) + ", " + nf(q.temp,0,2) + ", " + q.q0 + ", " + q.q1 + ", " + q.q2 + ", " + q.q3);

    // trim buffers
    if (quatLog.size() > MAX_POINTS) quatLog.remove(0);
    if (eulerLog.size() > MAX_POINTS) eulerLog.remove(0);
  }

  // Draw upper: Quaternion graph (q0,q1,q2,q3)
  drawQuaternionGraph(10, height/2 - 10);

  // Draw lower: Euler (if SHOW_EULER) else Quaternion again
  if (SHOW_EULER) {
    drawEulerGraph(height/2 + 10, height - 10);
  } else {
    drawQuaternionGraph(height/2 + 10, height - 10);
  }

  // mode label
  fill(0);
  textAlign(LEFT, TOP);
  text("Upper: Quaternion (q0,q1,q2,q3)  |  Lower: " + (SHOW_EULER ? "Euler (roll,pitch,yaw)" : "Quaternion (q0,q1,q2,q3)"), 10, 10);
}

// Draw quaternion graph in given vertical region
void drawQuaternionGraph(float yTop, float yBottom) {
  int n = quatLog.size();
  if (n < 2) return;
  float h = yBottom - yTop;
  float center = yTop + h/2.0;

  // background box
  noStroke();
  fill(255);
  rect(5, yTop-5, width-10, h+10);

  // draw lines
  for (int i = 1; i < n; i++) {
    Quat a = quatLog.get(i-1);
    Quat b = quatLog.get(i);

    float x1 = map(i-1, 0, max(1, n-1), 10, width-10);
    float x2 = map(i,   0, max(1, n-1), 10, width-10);

    // q0 (red)
    stroke(200, 30, 30);
    line(x1, center - a.q0 * h * 0.4, x2, center - b.q0 * h * 0.4);

    // q1 (green)
    stroke(30, 180, 30);
    line(x1, center - a.q1 * h * 0.4, x2, center - b.q1 * h * 0.4);

    // q2 (blue)
    stroke(30, 30, 200);
    line(x1, center - a.q2 * h * 0.4, x2, center - b.q2 * h * 0.4);

    // q3 (yellow)
    stroke(200, 180, 30);
    line(x1, center - a.q3 * h * 0.4, x2, center - b.q3 * h * 0.4);
  }

  // legend (small)
  noStroke();
  fill(0);
  textAlign(LEFT, BOTTOM);
  text("q0 (red), q1 (green), q2 (blue), q3 (yellow)", 12, yBottom - 4);
}

// Draw Euler angles graph in given vertical region (degrees)
void drawEulerGraph(float yTop, float yBottom) {
  int n = eulerLog.size();
  if (n < 2) return;
  float h = yBottom - yTop;

  // background box
  noStroke();
  fill(255);
  rect(5, yTop-5, width-10, h+10);

  // typical angle range +/-180 for safety
  float angleRange = 180;

  for (int i = 1; i < n; i++) {
    PVector a = eulerLog.get(i-1);
    PVector b = eulerLog.get(i);

    float x1 = map(i-1, 0, max(1, n-1), 10, width-10);
    float x2 = map(i,   0, max(1, n-1), 10, width-10);

    // roll (red)
    stroke(200, 30, 30);
    line(x1, map(a.x, -angleRange, angleRange, yBottom, yTop),
         x2, map(b.x, -angleRange, angleRange, yBottom, yTop));

    // pitch (green)
    stroke(30, 180, 30);
    line(x1, map(a.y, -angleRange, angleRange, yBottom, yTop),
         x2, map(b.y, -angleRange, angleRange, yBottom, yTop));

    // yaw (blue)
    stroke(30, 30, 200);
    line(x1, map(a.z, -angleRange, angleRange, yBottom, yTop),
         x2, map(b.z, -angleRange, angleRange, yBottom, yTop));
  }

  // legend
  noStroke();
  fill(0);
  textAlign(LEFT, BOTTOM);
  text("roll (red), pitch (green), yaw (blue) [deg]", 12, yBottom - 4);
}
