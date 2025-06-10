import processing.serial.*;
import java.util.ArrayList;

Serial myPort;

class SensorData {
  float timestamp;
  float temp;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;

  SensorData(){
    this.timestamp = 0.0;
    this.temp = 0.0;
    this.ax = 0.0;
    this.ay = 0.0;
    this.az = 0.0;
    this.gx = 0.0;
    this.gy = 0.0;
    this.gz = 0.0;
  }
  
  SensorData(float timestamp, float temp, float ax, float ay, float az, float gx, float gy, float gz) {
    this.timestamp = timestamp;
    this.temp = temp;
    this.ax = ax;
    this.ay = ay;
    this.az = az;
    this.gx = gx;
    this.gy = gy;
    this.gz = gz;
  }
  
}

void setup() {
  size(1200, 600);

  myPort = new Serial(this, "COM10", 115200);

}

SensorData parseSensorData(String str) {
  String[] values = split(str, ',');
  if (values.length == 8) {
    float timestamp = float(values[0]);
    float temp = float(values[1]);
    float ax = float(values[2]);
    float ay = float(values[3]);
    float az = float(values[4]);
    float gx = float(values[5]);
    float gy = float(values[6]);
    float gz = float(values[7]);

    if (Float.isNaN(ax)) ax = 0;
    if (Float.isNaN(ay)) ay = 0;
    if (Float.isNaN(az)) az = 0;
    if (Float.isNaN(gx)) gx = 0;
    if (Float.isNaN(gy)) gy = 0;
    if (Float.isNaN(gz)) gz = 0;

    SensorData data = new SensorData(timestamp, temp, ax, ay, az, gx, gy, gz);
    return data;
  } else {
    println("Invalid data format");
    return new SensorData();
  }
}

void print_data(SensorData data) {
  print(data.timestamp);print(",\t");
  print(data.temp);print(",\t");
  print(data.ax);print(",\t");
  print(data.ay);print(",\t");
  print(data.az);print(",\t");
  print(data.gx);print(",\t");
  print(data.gy);print(",\t");
  println(data.gz);  
}

ArrayList<SensorData> sensorDataList = new ArrayList<SensorData>();

void draw_graph() {
  stroke(255, 0, 0);
  noFill();
  beginShape();
  float margin = 20;
  for (int i = 0; i < sensorDataList.size(); i++) {
    SensorData data = sensorDataList.get(i);
    float x = map(i, 0, sensorDataList.size(), margin, width - margin);
    float y = map(data.ax, -margin, margin, height - margin, margin);
    vertex(x, y);
  }
  endShape();

  stroke(0, 255, 0);
  beginShape();
  for (int i = 0; i < sensorDataList.size(); i++) {
    SensorData data = sensorDataList.get(i);
    float x = map(i, 0, sensorDataList.size(), margin, width - margin);
    float y = map(data.ay, -margin, margin, height - margin, margin);
    vertex(x, y);
  }
  endShape();

  stroke(0, 0, 255);
  beginShape();
  for (int i = 0; i < sensorDataList.size(); i++) {
    SensorData data = sensorDataList.get(i);
    float x = map(i, 0, sensorDataList.size(), margin, width - margin);
    float y = map(data.az, -margin, margin, height - margin, margin);
    vertex(x, y);
  }
  endShape();

  stroke(200, 0, 200);
  beginShape();
  for (int i = 0; i < sensorDataList.size(); i++) {
    SensorData data = sensorDataList.get(i);
    float x = map(i, 0, sensorDataList.size(), margin, width - margin);
    float y = map(data.gx, -margin, margin, height - margin, margin);
    vertex(x, y);
  }
  endShape();

  stroke(50, 255, 255);
  beginShape();
  for (int i = 0; i < sensorDataList.size(); i++) {
    SensorData data = sensorDataList.get(i);
    float x = map(i, 0, sensorDataList.size(), margin, width - margin);
    float y = map(data.gy, -margin, margin, height - margin, margin);
    vertex(x, y);
  }
  endShape();

  stroke(50, 50, 50);
  beginShape();
  for (int i = 0; i < sensorDataList.size(); i++) {
    SensorData data = sensorDataList.get(i);
    float x = map(i, 0, sensorDataList.size(), margin, width - margin);
    float y = map(data.gz, -margin, margin, height - margin, margin);
    vertex(x, y);
  }
  endShape();

}

void draw() {
  background(200);

  if (myPort.available() > 0) {
    String str = myPort.readStringUntil('\n');
    if (str != null) {
      SensorData data = parseSensorData(str);
        print_data(data);
       
        if (sensorDataList.size() >= 120) {
          sensorDataList.remove(0);
        }
        sensorDataList.add(data);
      draw_graph();
    }
  }
}
