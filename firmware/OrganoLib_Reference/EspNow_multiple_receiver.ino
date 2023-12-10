
#include <esp_now.h>
#include <WiFi.h>


// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
    int id; // must be unique for each sender board
     //
    float sensor1; 
    float sensor2;
    //add more snesors.  
}struct_message;

// Create a struct_message called myData
struct_message myData;

// Create a structure to hold the readings from each board (now we have two)
struct_message board1;
struct_message board2;
//struct_message board3;

// Create an array with all the structures (now we have two)
struct_message boardsStruct[2] = {board1, board2};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  
  //Get peer MAC address
  //char macStr[18];
  //Serial.print("Packet received from: ");
  //snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(macStr);

  //Copy content incomingData into myData variable
  memcpy(&myData, incomingData, sizeof(myData));

  
  // Update the structures with the new incoming data
  boardsStruct[myData.id-1].hallx = myData.sensor1;
  boardsStruct[myData.id-1].hally = myData.sensor2;




  Serial.printf("%u ", myData.id);
  Serial.printf("%.2f ", boardsStruct[myData.id-1].sensor1);
  Serial.printf("%.2f ", boardsStruct[myData.id-1].sensor2);

/*Serial.write((const uint8_t*)&myData.id, sizeof(myData.id));
Serial.write((const uint8_t*)&boardsStruct[myData.id-1].hall, sizeof(boardsStruct[myData.id-1].sensor1));
Serial.write((const uint8_t*)&boardsStruct[myData.id-1].hallx, sizeof(boardsStruct[myData.id-1].sensor2));
Serial.write("\r");*/
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // Acess the variables for each board
  /*int board1X = boardsStruct[0].x;
  int board1Y = boardsStruct[0].y;
  int board2X = boardsStruct[1].x;
  int board2Y = boardsStruct[1].y;
  int board3X = boardsStruct[2].x;
  int board3Y = boardsStruct[2].y;*/

  //delay(10000);  
}
