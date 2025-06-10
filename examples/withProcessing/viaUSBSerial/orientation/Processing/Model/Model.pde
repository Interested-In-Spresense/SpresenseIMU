import processing.serial.*;

Serial myPort;

class SensorData {
  float timestamp;
  float temp;
  float roll;
  float pitch;
  float yaw;

  SensorData(){
    this.timestamp = 0.0;
    this.temp = 0.0;
    this.roll = 0.0;
    this.pitch = 0.0;
    this.yaw = 0.0;
  }
  
  SensorData(float timestamp, float temp, float roll, float pitch, float yaw) {
    this.timestamp = timestamp;
    this.temp = temp;
    this.roll = roll;
    this.pitch = pitch;
    this.yaw = yaw;
  }

}

void setup() {
  size(600, 600, P3D);
  frameRate(60);

  myPort = new Serial(this, "COM43", 115200);

}

SensorData parseSensorData(String str) {
  String[] values = split(str, ',');
  if (values.length == 5) {
    float timestamp = float(values[0]);
    float temp = float(values[1]);
    float roll = float(values[2]);
    float pitch = float(values[3]);
    float yaw = float(values[4]);

    if (Float.isNaN(roll)) roll = 0;
    if (Float.isNaN(pitch)) pitch = 0;
    if (Float.isNaN(yaw)) yaw = 0;

    SensorData data = new SensorData(timestamp, temp, roll, pitch, yaw);
    return data;
  } else {
    println("Invalid data format");
    return new SensorData();
  }
}

void print_data(SensorData data) {
  print(data.timestamp);print(",\t");
  print(data.temp);print(",\t");
  print(data.roll);print(",\t");
  print(data.pitch);print(",\t");
  println(data.yaw);
}

void draw() {
  background(200);
  translate(width / 2, height / 2);

  if (myPort.available() > 0) {
    String str = myPort.readStringUntil('\n');
    if (str != null) {
      SensorData data = parseSensorData(str);
      print_data(data);

      rotateZ(radians(-data.roll));
      rotateX(radians(-data.pitch));
      rotateY(radians(data.yaw-60));

      fill(50, 50, 50);
      box(150,30,200);

    }
  }
}
