#include "primary_mirror_network.h"
#include "primary_mirror_global.h"
#include <NativeEthernet.h>


//IPAddress ip(IP);

byte mac[] = MAC;

bool networkInit() {
    
    Ethernet.init(32); //Temporary, review in assembly

    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
    }

    // start the Ethernet connection:
    Serial.println("Initialize Ethernet with DHCP:");
    if (Ethernet.begin(mac) == 0) {
        Serial.println("Failed to configure Ethernet using DHCP");
        // Check for Ethernet hardware present
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
        while (true) {
            delay(1); // do nothing, no point running without Ethernet hardware
        }
        }
        if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
        }
        // try to congifure using IP address instead of DHCP:
        //Ethernet.begin(mac, ip);
        Serial.println("Failed to configure Ethernet using IP");
        Serial.print("My IP address: ");
        Serial.println(Ethernet.localIP());
    } 
    else {
        Serial.print("  DHCP assigned IP ");
        Serial.println(Ethernet.localIP());
    }
    // give the Ethernet shield a second to initialize:
    delay(1000);

}