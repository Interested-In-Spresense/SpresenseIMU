import processing.serial.*;

Serial myPort;

class Quat {
  float w, x, y, z;
  Quat(float w_, float x_, float y_, float z_) {
    w = w_;  x = x_;  y = y_;  z = z_;
  }
  void normalize() {
    float n = sqrt(w*w + x*x + y*y + z*z);
    if (n == 0) { w = 1; x = y = z = 0; return; }
    w /= n;  x /= n;  y /= n;  z /= n;
  }
}

Quat orientation = new Quat(1, 0, 0, 0);
float timestamp = 0;
float temp = 0;

void setup() {
  size(600, 600, P3D);
  myPort = new Serial(this, "COM80", 115200);
  myPort.bufferUntil('\n');
  textFont(createFont("Courier", 12));
}

void parseLine(String line) {
  String[] s = split(trim(line), ',');
  if (s.length != 6) return;
  timestamp = float(s[0]);
  temp      = float(s[1]);
  orientation = new Quat(float(s[2]), float(s[3]), float(s[4]), float(s[5]));
  orientation.normalize();
}

float[] quatToMatrix3(Quat q) {
  float w = q.w, x = q.x, y = q.y, z = q.z;
  float xx = x*x, yy = y*y, zz = z*z;
  float xy = x*y, xz = x*z, yz = y*z;
  float wx = w*x, wy = w*y, wz = w*z;

  return new float[]{
    1 - 2*(yy + zz), 2*(xy - wz), 2*(xz + wy),
    2*(xy + wz), 1 - 2*(xx + zz), 2*(yz - wx),
    2*(xz - wy), 2*(yz + wx), 1 - 2*(xx + yy)
  };
}

PVector quatToEuler(Quat q) {
  float w = q.w, x = q.x, y = q.y, z = q.z;
  float roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
  float pitch = abs(2*(w*y - z*x)) >= 1 ? (2*(w*y - z*x) > 0 ? HALF_PI : -HALF_PI) : asin(2*(w*y - z*x));
  float yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
  return new PVector(degrees(roll), degrees(pitch), degrees(yaw));
}

void draw() {
  background(255);
  lights();
  translate(width/2, height/2, 0);

  while (myPort.available() > 0) {
    String line = myPort.readStringUntil('\n');
    if (line != null) parseLine(line);
  }

  applyRotationMatrix();
  drawModel();
  resetHUD();
  drawHUD();
  drawWorldAxis();
}

void applyRotationMatrix() {
  float[] m = quatToMatrix3(orientation);
  applyMatrix(
     m[0],   m[2],   -m[1],  0,
    -m[6],  -m[8],    m[7],  0,
    -m[3],  -m[5],    m[4],  0,
     0,      0,       0,    1
  );
}

void drawModel() {
  fill(200);
  stroke(120);
  strokeWeight(1.5);
  box(80, 20, 120);

  float axisLen = 120;
  float as = 5;
  strokeWeight(3);

  stroke(255, 80, 80);
  line(0, 0, 0, axisLen, 0, 0);
  pushMatrix();
    translate(axisLen, 0, 0);
    rotateY(HALF_PI);
    fill(255, 80, 80);
    noStroke();
    drawCone(as, as*1.6);
  popMatrix();

  stroke(80, 255, 80);
  line(0, 0, 0, 0, 0, -axisLen);
  pushMatrix();
    translate(0, 0, -axisLen);
    rotateY(-PI);
    fill(80, 255, 80);
    noStroke();
    drawCone(as, as*1.6);
  popMatrix();

  stroke(80, 80, 255);
  line(0, 0, 0, 0, axisLen, 0);
  pushMatrix();
    translate(0, axisLen, 0);
    rotateX(-HALF_PI);
    fill(80, 80, 255);
    noStroke();
    drawCone(as, as*1.6);
  popMatrix();
}

void resetHUD() {
  camera();
  hint(DISABLE_DEPTH_TEST);
  fill(0);
  textSize(13);
}

void drawHUD() {
  text("Timestamp: " + nf(timestamp, 1, 3), 20, 20);
  text("Temp: " + nf(temp, 1, 2) + " C", 20, 40);

  PVector e = quatToEuler(orientation);
  text("Roll:  " + nf(e.x, 1, 1), 20, 70);
  text("Pitch: " + nf(e.y, 1, 1), 20, 90);
  text("Yaw:   " + nf(e.z, 1, 1), 20, 110);

  text("Quat(w,x,y,z)", 20, 140);
  text(nf(orientation.w,1,4)+", "+nf(orientation.x,1,4)+", "+
       nf(orientation.y,1,4)+", "+nf(orientation.z,1,4), 20, 160);
}

void drawWorldAxis() {
  pushMatrix();
  translate(width - 120, 120, 0);
  float S = 50;

  stroke(255,0,0); strokeWeight(4);
  line(0,0,0, S,0,0);
  drawArrowHUD(new PVector(S,0,0), new PVector(1,0,0), color(255,0,0));
  fill(255,0,0); text("X", S+8, -4);

  stroke(0,255,0);
  line(0,0,0, 0,0,S);
  drawArrowHUD(new PVector(0,0,S), new PVector(0,0,1), color(0,255,0));
  fill(0,200,0); text("Y", 4, -4, S+8);

  stroke(0,0,255);
  line(0,0,0, 0,-S,0);
  drawArrowHUD(new PVector(0,-S,0), new PVector(0,-1,0), color(0,0,255));
  fill(0,0,255); text("Z", -6, -S-10);

  popMatrix();
  hint(ENABLE_DEPTH_TEST);
}

void drawArrowHUD(PVector pos, PVector dir, int c) {
  pushMatrix();
  translate(pos.x, pos.y, pos.z);
  dir.normalize();

  PVector z = new PVector(0,0,1);
  PVector axis = z.cross(dir);
  float angle = acos(z.dot(dir));
  if (axis.mag() > 0.0001) rotate(angle, axis.x, axis.y, axis.z);

  fill(c);
  noStroke();
  beginShape();
  vertex(-3,-3,0);
  vertex( 3,-3,0);
  vertex( 3, 3,0);
  vertex(-3, 3,0);
  endShape(CLOSE);

  translate(0,0,6);
  drawCone(5,12);
  popMatrix();
}

void drawCone(float r, float h) {
  int S = 12;
  beginShape(TRIANGLE_FAN);
  vertex(0,0,h);
  for (int i=0; i<=S; i++) {
    float a = TWO_PI/S*i;
    vertex(cos(a)*r, sin(a)*r, 0);
  }
  endShape();
}
