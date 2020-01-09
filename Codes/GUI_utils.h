
uint8_t firstbyte;
void senddata(void);
void senddatadebug(void);
uint8_t LPC(uint8_t tocheck[bytestobechecked]);
int movecursor(int axis, float angle);
uint8_t writebooleantobyte(bool *FirstTouch, bool *SecondTouch,
                           bool *ButtonisPressed);
void checktouch(void);
void calculatecoordinates(void);
uint8_t summing(uint8_t *tocheck);

void senddata(void) {
  Serial.write(FrameID);
  Serial.write(firstbyte);
  Serial.write(X1);
  Serial.write(Y1);
  Serial.write(X2);
  Serial.write(Y2);
  Serial.write(checksum);
  ButtonisPressed = false;
}

void senddatadebug(void) {
  Serial.print(FrameID, HEX);
  Serial.print(",");
  // Serial.write(FirstTouch);
  // Serial.write(SecondTouch);
  // Serial.write(ButtonisPressed);
  Serial.print(firstbyte, BIN);
  Serial.print(",");
  Serial.print(X1);
  Serial.print(",");
  Serial.print(Y1);
  Serial.print(",");
  Serial.print(X2);
  Serial.print(",");
  Serial.print(Y2);
  Serial.print(",");
  Serial.println(checksum);
  ButtonisPressed = false;
}

//Needs Revisiting
int movecursor(int axis, float angle) {
  int output;
  float temp;
  temp = angle;
  if (axis == 0) {
    // output = map(temp, -60, -range, 60, range);
    if (temp > 10) {
      output = 1;
    } else if (temp < -10) {
      output = -1;
    } else {
      output = 0;
    }
  } else {
    // output = map(temp, -90, -range, 90, range);
    if (temp > 20) {
      output = -1;
    } else if (temp < -20) {
      output = 1;
    } else {
      output = 0;
    }
  }
  return output;
}

// Needs Revisiting
void calculatecoordinates(void) {
  int movex;
  int movey;
  movex = movecursor(0, angle[0]);
  movey = movecursor(1, angle[1]);
  if (((previousx == 0) && (movex < 0)) ||
      ((previousx == 255) && (movex > 0))) {
    // currentx = previousx;
    movex = 0;
  }
  // else {
  //   currentx = previousx + movex;
  // }
  if (((previousy == 0) && (movey < 0)) ||
      ((previousy == 255) && (movey > 0))) {
    // currenty = previousy;
    movey = 0;
  }
  //  else {
  //   currenty = previousy + movey;
  // }
  currentx = movex + previousx;
  currenty = movey + previousy;
}

uint8_t writebooleantobyte(bool FirstTouch, bool SecondTouch,
                           bool ButtonisPressed) {
  uint8_t output = 0x00;
  if (FirstTouch == true) {
    output |= 1UL << 0;
  }
  if (SecondTouch == true) {
    output |= 1UL << 1;
  }
  if (ButtonisPressed == true) {
    output |= 1UL << 2;
  }
  return output;
}

void populateLPC(void) {
  tocheck[0] = firstbyte;
  tocheck[1] = X1;
  tocheck[2] = Y1;
  tocheck[3] = X2;
  tocheck[4] = Y2;
}

void checktouch(void) {
  int buttonstate;
  buttonstate = digitalRead(touchbutton);
  if (!buttonstate) {
    ButtonisPressed = true;
  }
}

// Old function for calculating the sum
// check variable; For proper checksum just
// sum all the variables in one unsigned byte
uint8_t summing(uint8_t *tocheck) {
  uint8_t sum;
  if (firstbyte + X1 + X2 + Y1 + Y2 > 255) {
    sum = 255;
  } else {
    sum = firstbyte + X1 + X2 + Y1 + Y2;
  }
  return sum;
}

void receivedata(void) {
  while (Serial.available()) {
    Serial.read();
  }
}

// DO NOT USE:
// Old function for calculating the sum
// check variable; For proper checksum just
// sum all the variables in one unsigned byte
uint8_t LPC(uint8_t *tocheck) {
  uint8_t checked = 0x00;
  for (uint8_t i = 0; i < 5; i++) {
    checked = checked ^ tocheck[i];
  }
  return checked;
}
