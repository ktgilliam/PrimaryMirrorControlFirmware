/*******************************************************************************
Copyright 2022
Steward Observatory Engineering & Technical Services, University of Arizona
This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <https://www.gnu.org/licenses/>.
*******************************************************************************/

/**
@brief 
@author Nestor Garcia
@date October 17, 2022
@file primary_mirror_network.cpp

*/

#include "primary_mirror_network.h"
#include "primary_mirror_global.h"
#include <NativeEthernet.h>
#include <string.h>
#include <map>
//#include <NetComms.h>


IPAddress ip(IPAdd);
byte mac[] = MAC;
EthernetClient PMC_client;

bool networkInit() {
    
    Ethernet.init(32); //Temporary, review in assembly

    // wait for serial port to connect. Needed for native USB port only
    // while (!Serial) { ; }

    // Start the Ethernet connection:
    // Try to initialize Ethernet with DHCP
    // Serial.println("Initialize Ethernet with DHCP:");
    if (Ethernet.begin(mac) == 0) {
        Serial.println("Failed to configure Ethernet using DHCP");
        // Check for Ethernet hardware present
        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
            Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
            return(false);

        }
        if (Ethernet.linkStatus() == LinkOFF) {
            Serial.println("Ethernet cable is not connected.");
            return(false);
        }
        // try to congifure using IP address instead of DHCP:
        Ethernet.begin(mac, ip);
        if (Ethernet.localIP() != ip) {
            Serial.println("Failed to configure Ethernet using IP");
            return(false);
        }
        Serial.print("My IP address: ");
        Serial.println(Ethernet.localIP());
    } 
    else {
        Serial.print("  DHCP assigned IP ");
        Serial.println(Ethernet.localIP()); 
    }
    // give the Ethernet shield a second to initialize:
    delay(1000);

    if (!(PMC_client.connect(Ethernet.localIP(), PORT))) {
        Serial.print("  Connected as PMC_client. ");
    }
    return(true);
}

void read_command() {

    uint8_t data;

    while((PMC_client.available()) != 0) {

        data = PMC_client.read();
        Serial.println(data);
    }
    //return(data);
}

void messageParser() {

}