#include "DFRobotDFPlayerMini.h"

// Use Serial1 instead of SoftwareSerial
#define DFPLAYER_RX 0  // Nano 33 BLE Sense RX (connect to DFPlayer TX)
#define DFPLAYER_TX 1  // Nano 33 BLE Sense TX (connect to DFPlayer RX)

DFRobotDFPlayerMini myDFPlayer;
String line;
char command;
int pause = 0;
int repeat = 0;

void setup() {
  // Initialize hardware serial for DFPlayer Mini
  Serial1.begin(9600);
  // Initialize Arduino serial for debugging
  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini"));
  Serial.println(F("Initializing DFPlayer module ... Wait!"));

  if (!myDFPlayer.begin(Serial1)) {  // Use Serial1 instead of SoftwareSerial
    Serial.println(F("Not initialized:"));
    Serial.println(F("1. Check the DFPlayer Mini connections"));
    Serial.println(F("2. Insert an SD card"));
    while (true);
  }

  Serial.println();
  Serial.println(F("DFPlayer Mini module initialized!"));

  // Initial settings
  myDFPlayer.setTimeOut(500);  // Serial timeout 500ms
  myDFPlayer.volume(5);        // Volume 5
  myDFPlayer.EQ(0);            // Normal equalization

  menu_options();
}

void loop() {
  while (Serial.available() > 0) {
    command = Serial.peek();
    line = Serial.readStringUntil('\n');

    if ((command >= '1') && (command <= '9')) {
      Serial.print("Music reproduction ");
      Serial.println(command);
      command = command - '0';
      myDFPlayer.play(command);
      menu_options();
    }

    if (command == 'f') {
      int indexF = line.indexOf('f');
      int indexS = line.indexOf('s');
      if (indexF != -1 && indexS != -1 && indexF < indexS) {
        int folder = line.substring(indexF + 1, indexS).toInt();
        int song = line.substring(indexS + 1).toInt();
        Serial.print("From folder: ");
        Serial.print(folder);
        Serial.print(", playing song: ");
        Serial.println(song);
        myDFPlayer.playFolder(folder, song);
      } else {
        Serial.println("Incomplete 'f' command. Specify both folder and song numbers.");
      }
      menu_options();
    }

    if (command == 's') {
      myDFPlayer.stop();
      Serial.println("Music Stopped!");
      menu_options();
    }

    if (command == 'p') {
      pause = !pause;
      if (pause == 0) {
        Serial.println("Continue...");
        myDFPlayer.start();
      } else {
        Serial.println("Music Paused!");
        myDFPlayer.pause();
      }
      menu_options();
    }

    if (command == 'r') {
      repeat = !repeat;
      if (repeat == 1) {
        myDFPlayer.enableLoop();
        Serial.println("Repeat mode enabled.");
      } else {
        myDFPlayer.disableLoop();
        Serial.println("Repeat mode disabled.");
      }
      menu_options();
    }

    if (command == 'v') {
      int myVolume = line.substring(1).toInt();
      if (myVolume >= 0 && myVolume <= 30) {
        myDFPlayer.volume(myVolume);
        Serial.print("Current Volume:");
        Serial.println(myDFPlayer.readVolume());
      } else {
        Serial.println("Invalid volume level, choose a number between 0-30.");
      }
      menu_options();
    }

    if (command == '+') {
      myDFPlayer.volumeUp();
      Serial.print("Current Volume:");
      Serial.println(myDFPlayer.readVolume());
      menu_options();
    }

    if (command == '-') {
      myDFPlayer.volumeDown();
      Serial.print("Current Volume:");
      Serial.println(myDFPlayer.readVolume());
      menu_options();
    }

    if (command == '<') {
      myDFPlayer.previous();
      Serial.println("Previous:");
      Serial.print("Current track:");
      Serial.println(myDFPlayer.readCurrentFileNumber() - 1);
      menu_options();
    }

    if (command == '>') {
      myDFPlayer.next();
      Serial.println("Next:");
      Serial.print("Current track:");
      Serial.println(myDFPlayer.readCurrentFileNumber() + 1);
      menu_options();
    }
  }
}

void menu_options() {
  Serial.println();
  Serial.println(F("=================================================================================================================================="));
  Serial.println(F("Commands:"));
  Serial.println(F(" [1-9] To select the MP3 file"));
  Serial.println(F(" [fXsY] Play song from folder X, song Y"));
  Serial.println(F(" [s] stopping reproduction"));
  Serial.println(F(" [p] pause/continue music"));
  Serial.println(F(" [r] toggle repeat mode"));
  Serial.println(F(" [vX] set volume to X"));
  Serial.println(F(" [+ or -] increases or decreases the volume"));
  Serial.println(F(" [< or >] forwards or backwards the track"));
  Serial.println();
  Serial.println(F("================================================================================================================================="));
}