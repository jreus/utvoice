#include <esp_now.h>
#include <WiFi.h>

//timer for transmission freq
unsigned long timer = 0;

// REPLACE WITH THE RECEIVER'S MAC Address
uint8_t broadcastAddress[] = {0x94, 0xB9, 0x7E, 0xD2, 0x14, 0x1C};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id; // must be unique for each sender board
     //magnet
    float sensor1; 
    float sensor2;

} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);


// Init ESP-NOW
if (esp_now_init() != ESP_OK) {
  Serial.println("Error initializing ESP-NOW");
  return;
   }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  //esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {

  if((millis()-timer)>10)
  
  {

 
  //Read from sensors

  // Set values to send
  myData.id = 1; //Set ID board number
  myData.sensor1 = //sensor1 readings;
  myData.sensor2 = //sensor2 readings;
  
 
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Delivered");
  }
  else {
    Serial.println("Error sending the data");
  }
  timer = millis();
  }
}
