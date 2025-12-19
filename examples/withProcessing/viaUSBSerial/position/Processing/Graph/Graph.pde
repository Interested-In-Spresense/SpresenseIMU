import processing.serial.*;
import java.util.ArrayList;
import peasy.*;  // 3Dカメラ用 (スケッチ→ライブラリ→PeasyCamインストール)

// ===== SETTINGS =====
static final String PORT_NAME = "COM80";  // ポート
static final int BAUD = 115200;
static final int MAX_POINTS = 500;        // 軌跡の長さ
static final float SCALE = 200;           // 3Dスケール倍率
// ===================

Serial serial;
ArrayList<PVector> path = new ArrayList<>();
PeasyCam cam;

void setup() {
  size(900, 700, P3D);
  cam = new PeasyCam(this, 500);
  serial = new Serial(this, PORT_NAME, BAUD);
  serial.bufferUntil('\n');
}

void draw() {
  background(20);
  lights();
  drawAxis(100);

  // === シリアル受信 ===
  while (serial.available() > 0) {
    String line = serial.readStringUntil('\n');
     println(line);
    if (line == null) break;
    parseLine(line.trim());
  }

  // === 軌跡描画 ===
  noFill();
  stroke(0, 255, 255);
  strokeWeight(2);
  beginShape();
  for (PVector p : path) {
    vertex(p.x * SCALE, -p.z * SCALE, -p.y * SCALE);
  }
  endShape();

  // === 現在位置の点描画 ===
  if (path.size() > 0) {
    PVector last = path.get(path.size()-1);
    pushMatrix();
    translate(last.x * SCALE, -last.z * SCALE, -last.y * SCALE);
    fill(255, 0, 0);
    noStroke();
    sphere(5);
    popMatrix();
  }

  // === テキスト表示 ===
  hint(DISABLE_DEPTH_TEST);
  cam.beginHUD();
  fill(255);
  text("Points: " + path.size(), 10, 20);
  cam.endHUD();
  hint(ENABLE_DEPTH_TEST);
}

void parseLine(String line) {
  String[] v = split(line, ',');
  if (v.length != 4) return;  // 4カラム以外は無視

  float ts = float(v[0]);
  float x  = float(v[1]);
  float y  = float(v[2]);
  float z  = float(v[3]);

  path.add(new PVector(x, y, z));
  if (path.size() > MAX_POINTS) path.remove(0);

  println(ts + "  pos = " + x + ", " + y + ", " + z);
}

void drawAxis(float s) {
  strokeWeight(2);
  stroke(255, 0, 0); line(0, 0, 0,  s, 0, 0); drawAxisLabel("X", s, 0, 0,  color(255, 0, 0));  // X 右（赤）
  stroke(0, 255, 0); line(0, 0, 0,  0, 0,-s); drawAxisLabel("Y", 0, 0, -s, color(0, 255, 0));  // Y 奥（緑）
  stroke(0, 0, 255); line(0, 0, 0,  0,-s, 0); drawAxisLabel("Z", 0, -s, 0, color(0, 0, 255));  // Z 上（青）
}

void drawAxisLabel(String label, float x, float y, float z, int c) {
  pushMatrix();
  translate(x, y, z);

  // カメラに正対させる（PeasyCam対策）
  PMatrix3D mat = new PMatrix3D();
  getMatrix(mat);
  mat.m00 = 1; mat.m01 = 0; mat.m02 = 0;
  mat.m10 = 0; mat.m11 = 1; mat.m12 = 0;
  mat.m20 = 0; mat.m21 = 0; mat.m22 = 1;
  setMatrix(mat);

  fill(c);
  textSize(16);
  text(label, 4, -4);  // 少しずらす
  popMatrix();
}

