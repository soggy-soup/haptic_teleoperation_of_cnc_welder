void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

String t1 = "0a0a"; 
String machine_pos = "0a0";
void serial_read_machine(){

  if (Serial.available()){
    machine_pos = Serial.readString();
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
    machine_pos = Serial.readString();  //gets one byte from serial buffer
  }
  Serial.println(t1.substring(0,6));
}
