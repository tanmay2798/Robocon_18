int LSA_0 = A0, LSA_1 = A2, LSA_2 = A4, LSA_3 = A5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0,INPUT);
  pinMode(A2,INPUT);
  pinMode(A1,OUTPUT);
  pinMode(A3,OUTPUT);
  pinMode(A4,OUTPUT);
  pinMode(A5,OUTPUT);
  pinMode(A6,OUTPUT);
  pinMode(A7,OUTPUT);
  pinMode(A8,OUTPUT);
  pinMode(A9,OUTPUT);
  pinMode(A10,OUTPUT);
  pinMode(A11,OUTPUT);
  digitalWrite(A1,LOW);
  digitalWrite(A3,LOW);
  digitalWrite(A4,LOW);
  digitalWrite(A5,LOW);
  digitalWrite(A6,LOW);
  digitalWrite(A7,LOW);
  digitalWrite(A8,LOW);
  digitalWrite(A9,LOW);
  digitalWrite(A10,LOW);
  digitalWrite(A11,LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("LSA_0: ");
  Serial.print(analogRead(LSA_0));
  delay(10);
  Serial.print("\tLSA_1: ");
  Serial.println(analogRead(LSA_1));
  delay(10);
//  Serial.print("\tLSA_2: ");
//  Serial.print(analogRead(LSA_2));
//  delay(10);
//  Serial.print("\tLSA_3: ");
//  Serial.println(analogRead(LSA_3));
//  delay(10);
}
